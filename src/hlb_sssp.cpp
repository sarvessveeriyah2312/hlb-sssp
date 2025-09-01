#include "hlb/hlb_sssp.hpp"
#include <queue>
#include <deque>

namespace hlb {

static std::vector<int> Q_s, nextQ_s, R_s;

void sssp_u32(const CSR& g, int s, const Params& p, std::vector<uint32_t>& dist) {
    const uint32_t INF = 0xFFFFFFFFu;
    dist.assign(g.n, INF);
    dist[s] = 0;

    // pick Δ0
    uint32_t d0 = (p.delta0 > 0) ? (uint32_t)p.delta0 : ([&]{
        if (g.w.empty()) return (uint32_t)1;
        uint64_t tot = 0;
        for (int ww : g.w) tot += (uint32_t)ww;
        double avg = double(tot) / double(g.w.size());
        int a = (int)llround(avg);
        return (uint32_t) (a > 0 ? a : 1);
    })();

    int Ls = std::max(3, p.levels);
    std::vector<uint32_t> D(Ls, 0);
    D[0] = d0;
    for (int L=1; L<Ls; ++L) {
        uint32_t cand = (uint32_t) std::max(1, (int) llround((double)D[L-1]*p.ratio));
        D[L] = std::max<uint32_t>(D[L-1]+1, cand);
    }
    int tau0 = p.tau0 > 0 ? p.tau0 : std::max(64, (int)std::sqrt((double)std::max(1, g.n)));
    std::vector<int> T(Ls,0);
    for (int L=1; L<Ls; ++L)
        T[L] = std::max(1, (int) llround((double)tau0 * std::pow(p.tau_growth, (double)(L-1))));

    // Buckets per level
    std::vector<size_t> cap(Ls, 1u<<14);
    std::vector<std::vector<std::vector<int>>> B(Ls);
    std::vector<std::vector<char>> inH(Ls);
    std::vector<std::priority_queue<int, std::vector<int>, std::greater<int>>> PQ(Ls);
    for (int L=0; L<Ls; ++L) { B[L].resize(cap[L]); inH[L].assign(cap[L], 0); }

    auto ensure = [&](int L, uint64_t idx){
        size_t i = (size_t)idx;
        if (i < cap[L]) return;
        size_t need = i+1, grow = cap[L] ? cap[L] : 1;
        while (grow < need) grow <<= 1;
        B[L].resize(grow); inH[L].resize(grow, 0); cap[L] = grow;
    };
    auto bidx = [&](uint32_t d, uint32_t w)->uint32_t { if(!w) w=1; return d / w; };
    auto push = [&](int L, uint64_t i, int u){
        ensure(L,i);
        auto ii = (size_t)i;
        B[L][ii].push_back(u);
        if (!inH[L][ii]) { PQ[L].push((int)i); inH[L][ii] = 1; }
    };

    int coarse = Ls - 1;
    push(coarse, 0, s);

    auto level0 = [&](uint64_t i){
        uint32_t w = D[0];
        auto& BL = B[0];
        uint64_t low=i*(uint64_t)w, high=(i+1)*(uint64_t)w;
        uint32_t K = w + 1;
        std::vector<std::deque<int>> ring(K);
        uint64_t key=low;

        auto& R = R_s; R.clear();
        auto& Q = Q_s; Q.clear(); Q.swap(BL[(size_t)i]);

        for (int u: Q) {
            uint32_t k = dist[u];
            if ((uint64_t)k < high) ring[k%K].push_back(u);
            else { uint32_t j = bidx(dist[u], w); push(0, j, u); }
        }

        while (key < high) {
            auto& slot = ring[(size_t)(key%K)];
            if (slot.empty()) { key++; continue; }
            auto& curr = nextQ_s; curr.clear();
            while (!slot.empty()) { curr.push_back(slot.front()); slot.pop_front(); }
            for (int u: curr) {
                R.push_back(u);
                uint32_t du = dist[u];
                for (int ei=g.indptr[u]; ei<g.indptr[u+1]; ++ei) {
                    int v = g.to[ei];
                    uint32_t ww = (uint32_t)g.w[ei];
                    if (ww <= w) {
                        uint32_t nd = du + ww;
                        if (nd < dist[v]) {
                            dist[v] = nd;
                            if ((uint64_t)nd < high) ring[nd%K].push_back(v);
                            else { uint32_t j=bidx(nd,w); push(0,j,v); }
                        }
                    }
                }
            }
            key++;
        }
        // heavy once
        for (int u: R) {
            uint32_t du=dist[u];
            for (int ei=g.indptr[u]; ei<g.indptr[u+1]; ++ei) {
                int v=g.to[ei]; uint32_t ww=(uint32_t)g.w[ei];
                if (ww > w) {
                    uint32_t nd=du+ww;
                    if (nd < dist[v]) { dist[v]=nd; uint32_t j=bidx(nd,w); push(0,j,v); }
                }
            }
        }
    };

    auto coarseL = [&](int L, uint64_t i){
        uint32_t w = D[L];
        auto& BL=B[L]; auto& IH=inH[L];
        auto& R=R_s; R.clear();
        auto& Q=Q_s; Q.clear(); Q.swap(BL[(size_t)i]);

        while (!Q.empty()) {
            auto& next = nextQ_s; next.clear();
            for (int u: Q) {
                R.push_back(u);
                uint32_t du=dist[u];
                for (int ei=g.indptr[u]; ei<g.indptr[u+1]; ++ei) {
                    int v=g.to[ei]; uint32_t ww=(uint32_t)g.w[ei];
                    if (ww <= w) {
                        uint32_t nd=du+ww;
                        if (nd < dist[v]) {
                            dist[v]=nd;
                            uint32_t j=bidx(nd,w); ensure(L,j);
                            BL[(size_t)j].push_back(v);
                            if (j==i) next.push_back(v);
                            else if (!IH[(size_t)j]) { PQ[L].push((int)j); IH[(size_t)j]=1; }
                        }
                    }
                }
            }
            Q.swap(next);
        }
        // heavy once
        for (int u: R) {
            uint32_t du=dist[u];
            for (int ei=g.indptr[u]; ei<g.indptr[u+1]; ++ei) {
                int v=g.to[ei]; uint32_t ww=(uint32_t)g.w[ei];
                if (ww > w) {
                    uint32_t nd=du+ww;
                    if (nd < dist[v]) { dist[v]=nd; uint32_t j=bidx(nd,w); push(L,j,v); }
                }
            }
        }
    };

    auto refine = [&](int L, uint64_t i){
        int F=L-1; uint32_t w=D[F];
        auto& BL=B[L]; auto& BF=B[F]; auto& IHF=inH[F];
        std::vector<int> moved; moved.swap(BL[(size_t)i]);
        for (int u: moved) {
            uint32_t j = w ? (dist[u] / w) : dist[u];
            ensure(F,j); BF[(size_t)j].push_back(u);
            if (!IHF[(size_t)j]) { PQ[F].push((int)j); IHF[(size_t)j]=1; }
        }
    };

    // main loop
    while (true) {
        bool prog=false;
        for (int L=0; L<Ls; ++L) {
            auto& PQL = PQ[L];
            auto& IHL = inH[L];
            auto& BL  = B[L];
            while (!PQL.empty()) {
                int i=PQL.top(); PQL.pop();
                if ((size_t)i<IHL.size()) IHL[(size_t)i]=0;
                if ((size_t)i>=BL.size() || BL[(size_t)i].empty()) continue;
                if (L>0 && (int)BL[(size_t)i].size()>=T[L]) { refine(L,(uint64_t)i); prog=true; break; }
                if (L==0) level0((uint64_t)i); else coarseL(L,(uint64_t)i);
                prog=true; break;
            }
            if (prog) break;
        }
        if (!prog) break;
    }
}

} // namespace hlb
