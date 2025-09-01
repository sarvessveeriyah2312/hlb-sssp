#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include "hlb/hlb_sssp.hpp"

namespace py = pybind11;
using namespace hlb;

py::array_t<uint32_t> sssp_py(
    py::array_t<int, py::array::c_style | py::array::forcecast> indptr,
    py::array_t<int, py::array::c_style | py::array::forcecast> to,
    py::array_t<int, py::array::c_style | py::array::forcecast> w,
    int n, int source,
    int levels, double ratio, int tau0, double tau_growth, int delta0
){
    if (n <= 0) throw std::runtime_error("n must be > 0");
    auto I = indptr.unchecked<1>();
    auto T = to.unchecked<1>();
    auto W = w.unchecked<1>();

    CSR g; g.n = n;
    g.indptr.assign(&I(0), &I(0) + (size_t)I.shape(0));
    int m = g.indptr.back();
    if ((int)T.shape(0) != m || (int)W.shape(0) != m) throw std::runtime_error("m mismatch");
    g.to.assign(&T(0), &T(0) + m);
    g.w.assign(&W(0), &W(0) + m);
    if (!g.valid()) throw std::runtime_error("invalid CSR");

    Params p; p.levels=levels; p.ratio=ratio; p.tau0=tau0; p.tau_growth=tau_growth; p.delta0=delta0;

    std::vector<uint32_t> dist;
    sssp_u32(g, source, p, dist);

    auto out = py::array_t<uint32_t>((size_t)n);
    auto M = out.mutable_unchecked<1>();
    for (int i=0;i<n;++i) M(i) = dist[i];
    return out;
}

PYBIND11_MODULE(hlb_py, m){
    m.doc() = "HLB-SSSP v2.5 Python binding";
    m.def("sssp", &sssp_py,
          py::arg("indptr"), py::arg("to"), py::arg("w"),
          py::arg("n"), py::arg("source"),
          py::arg("levels")=3, py::arg("ratio")=12.0,
          py::arg("tau0")=256, py::arg("tau_growth")=2.0, py::arg("delta0")=-1);
}
