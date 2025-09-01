#include "hlb/hlb_c.h"
#include "hlb/hlb_sssp.hpp"

using namespace hlb;

int hlb_sssp_u32_c(
    int n,
    const int* indptr,
    const int* to,
    const int* w,
    int source,
    const hlb_params_t* params,
    uint32_t* dist_out
){
    if (!indptr || !to || !w || !dist_out || !params) return 1;
    if (n <= 0 || source < 0 || source >= n) return 2;

    CSR g;
    g.n = n;
    g.indptr.assign(indptr, indptr + (size_t)n + 1);
    int m = g.indptr.back();
    if (m < 0) return 3;

    g.to.assign(to, to + (size_t)m);
    g.w.assign(w, w + (size_t)m);
    if (!g.valid()) return 4;

    Params p;
    p.levels = params->levels;
    p.ratio = params->ratio;
    p.tau0 = params->tau0;
    p.tau_growth = params->tau_growth;
    p.delta0 = params->delta0;

    std::vector<uint32_t> dist;
    sssp_u32(g, source, p, dist);
    std::copy(dist.begin(), dist.end(), dist_out);
    return 0;
}
