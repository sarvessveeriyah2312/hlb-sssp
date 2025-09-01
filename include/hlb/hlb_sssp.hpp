#pragma once
#include <cstdint>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>

namespace hlb {

struct CSR {
    int n = 0;
    std::vector<int> indptr; // size n+1
    std::vector<int> to;     // size m
    std::vector<int> w;      // size m (positive integers)

    bool valid() const {
        if (n <= 0) return false;
        if ((int)indptr.size() != n+1) return false;
        if (to.size() != w.size()) return false;
        if (indptr.front()!=0) return false;
        if (indptr.back() != (int)to.size()) return false;
        return true;
    }
};

struct Params {
    int   levels      = 3;      // try 4 for very large graphs (n >= 1e6)
    double ratio      = 12.0;   // 8..16
    int   tau0        = 256;    // 128..512 or sqrt(n)
    double tau_growth = 2.0;    // 1.5..2.5
    int   delta0      = -1;     // -1 => use avg edge weight
};

/// Runs HLB-SSSP v2.5 (serial, 32-bit distances).
/// Preconditions:
///  - g.valid() == true
///  - all weights w[i] > 0 and small-ish integers
///  - 0 <= source < g.n
/// Output: dist.size()==g.n; unreachable nodes set to 0xFFFFFFFF
void sssp_u32(const CSR& g, int source, const Params& p, std::vector<uint32_t>& dist);

} // namespace hlb
