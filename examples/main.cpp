#include <hlb/hlb_sssp.hpp>
#include <iostream>
int main(){
    hlb::CSR g;
    g.n = 4;
    g.indptr = {0,2,3,3,4};
    g.to     = {1,2,2, 3};
    g.w      = {5,1,2, 1};
    hlb::Params p;
    std::vector<uint32_t> dist;
    hlb::sssp_u32(g, 0, p, dist);
    for (auto d : dist) std::cout << d << " ";
    std::cout << "\n";
    return 0;
}
