# HLB-SSSP v2.5 (C++ Library)

High-performance multi-level bucket SSSP for positive integer weights.
Includes C API (FFI) and optional Python binding (pybind11).

## Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release -j
sudo cmake --install build
```

Options:
- `-DHLB_BUILD_SHARED=ON` (default)
- `-DHLB_BUILD_STATIC=ON` (default)
- `-DHLB_BUILD_PYTHON=ON` (requires pybind11)

## C++ Usage

```cpp
#include <hlb/hlb_sssp.hpp>
hlb::CSR g; /* fill n, indptr, to, w */ 
hlb::Params p; // defaults: levels=3, ratio=12, tau0=256, tau_growth=2.0
std::vector<uint32_t> dist;
hlb::sssp_u32(g, 0, p, dist);
```

## C API

```c
#include <hlb/hlb_c.h>
uint32_t* dist = malloc(n*sizeof(uint32_t));
hlb_params_t p = {3,12.0,256,2.0,-1};
hlb_sssp_u32_c(n, indptr, to, w, 0, &p, dist);
```

## Python

```bash
pip install pybind11
cmake -S . -B build -DHLB_BUILD_PYTHON=ON
cmake --build build --config Release
python -c "import hlb_py"
```

## License

MIT (suggested) — edit as you prefer.
