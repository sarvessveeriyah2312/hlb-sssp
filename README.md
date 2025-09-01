# HLB-SSSP (C++ Library)

High-performance multi-level bucket SSSP for positive integer weights.
Includes **C++ API**, **C API (FFI)**, and an optional **Python binding (pybind11)**.

## Supported programming languages

* C++
* C (FFI)
* Python (pybind11 module or `ctypes` via C ABI)
* Go (cgo, via C ABI)
* Rust (bindgen, via C ABI)
* Java (JNI, via C ABI)
* C# / .NET (P/Invoke, via C ABI)

> Build once as a shared library (`libhlb_sssp.dylib` / `libhlb_sssp.so` / `hlb_sssp.dll`), then link or load it from any of the above languages.

---

## Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release -j
sudo cmake --install build
```

Options:

* `-DHLB_BUILD_SHARED=ON` (default)
* `-DHLB_BUILD_STATIC=ON` (default)
* `-DHLB_BUILD_PYTHON=ON` (requires pybind11)

---

## C++ usage

```cpp
#include <hlb/hlb_sssp.hpp>
#include <vector>
int main(){
    hlb::CSR g;
    g.n = 4;
    g.indptr = {0,2,3,3,4};          // CSR row pointers (n+1)
    g.to     = {1,2,2,3};            // column indices
    g.w      = {5,1,2,1};            // positive integer weights

    hlb::Params p;                   // defaults: levels=3, ratio=12, tau0=256, tau_growth=2.0
    std::vector<uint32_t> dist;
    hlb::sssp_u32(g, /*source=*/0, p, dist);
}
```

Compile & link (example):

```bash
c++ -std=c++20 your_app.cpp -I/path/to/hlb-sssp/include -L/path/to/hlb-sssp -lhlb_sssp -O3 -o your_app
```

---

## C usage (FFI)

```c
#include <hlb/hlb_c.h>
#include <stdint.h>
#include <stdlib.h>

int main(){
  int n=4;
  int indptr[5]={0,2,3,3,4};
  int to[4]={1,2,2,3};
  int w[4]={5,1,2,1};
  uint32_t dist[4];

  hlb_params_t p = (hlb_params_t){3, 12.0, 256, 2.0, -1};
  int rc = hlb_sssp_u32_c(n, indptr, to, w, /*source=*/0, &p, dist);
  return rc;
}
```

Compile & link (Linux example):

```bash
cc c_example.c -I/path/to/hlb-sssp/include -L/path/to/hlb-sssp -lhlb_sssp -O3 -o c_example
```

---

## Python

### Option A: pybind11 module (nicer API)

```bash
pip install pybind11
cmake -S . -B build -DHLB_BUILD_PYTHON=ON
cmake --build build --config Release
python -c "import hlb_py; print('ok')"
```

```python
import numpy as np
import hlb_py

indptr = np.array([0,2,3,3,4], dtype=np.int32)
to     = np.array([1,2,2,3],   dtype=np.int32)
w      = np.array([5,1,2,1],   dtype=np.int32)

dist = hlb_py.sssp(indptr, to, w, n=4, source=0)   # default params inside
print(dist)  # numpy array of uint32
```

### Option B: ctypes (uses the C ABI, no compile step)

```python
import ctypes, numpy as np, os
lib = ctypes.CDLL(os.path.abspath("libhlb_sssp.dylib"))  # use .so/.dll on Linux/Windows

class Params(ctypes.Structure):
    _fields_ = [("levels", ctypes.c_int),
                ("ratio", ctypes.c_double),
                ("tau0", ctypes.c_int),
                ("tau_growth", ctypes.c_double),
                ("delta0", ctypes.c_int)]

lib.hlb_sssp_u32_c.argtypes = [
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_int),
    ctypes.POINTER(ctypes.c_int),
    ctypes.POINTER(ctypes.c_int),
    ctypes.c_int,
    ctypes.POINTER(Params),
    ctypes.POINTER(ctypes.c_uint32),
]

indptr = np.array([0,2,3,3,4], dtype=np.int32)
to     = np.array([1,2,2,3],   dtype=np.int32)
w      = np.array([5,1,2,1],   dtype=np.int32)
dist   = np.zeros(4, dtype=np.uint32)

p = Params(3, 12.0, 256, 2.0, -1)
rc = lib.hlb_sssp_u32_c(4,
    indptr.ctypes.data_as(ctypes.POINTER(ctypes.c_int)),
    to.ctypes.data_as(ctypes.POINTER(ctypes.c_int)),
    w.ctypes.data_as(ctypes.POINTER(ctypes.c_int)),
    0, ctypes.byref(p),
    dist.ctypes.data_as(ctypes.POINTER(ctypes.c_uint32)))
print(rc, dist)
```

---

## Go (cgo)

`main.go`:

```go
package main

/*
#cgo CFLAGS: -I./include
#cgo LDFLAGS: -L. -lhlb_sssp
#include "hlb/hlb_c.h"
*/
import "C"
import "fmt"

func main(){
    indptr := []C.int{0,2,3,3,4}
    to     := []C.int{1,2,2,3}
    w      := []C.int{5,1,2,1}
    dist   := make([]C.uint, 4)
    p := C.hlb_params_t{levels:3, ratio:12.0, tau0:256, tau_growth:2.0, delta0:-1}
    rc := C.hlb_sssp_u32_c(4, &indptr[0], &to[0], &w[0], 0, &p, &dist[0])
    fmt.Println("rc=", rc, "dist=", dist)
}
```

Run:

```bash
# ensure libhlb_sssp is built and in current dir or on DYLD_LIBRARY_PATH/LD_LIBRARY_PATH
CGO_ENABLED=1 go run main.go
```

---

## Rust (bindgen)

`Cargo.toml`:

```toml
[package]
name = "hlb_demo"
version = "0.1.0"
edition = "2021"

[build-dependencies]
bindgen = "0.69"
```

`build.rs`:

```rust
fn main() {
    println!("cargo:rustc-link-search=native=.");
    println!("cargo:rustc-link-lib=hlb_sssp");
    let bindings = bindgen::Builder::default()
        .header("include/hlb/hlb_c.h")
        .generate()
        .expect("bindgen failed");
    std::fs::create_dir_all("src").unwrap();
    bindings.write_to_file("src/hlb_bindings.rs").unwrap();
}
```

`src/main.rs`:

```rust
mod hlb { include!("hlb_bindings.rs"); }
use hlb::*;
fn main(){
    let indptr = [0,2,3,3,4];
    let to     = [1,2,2,3];
    let w      = [5,1,2,1];
    let mut dist = [0u32; 4];
    let p = hlb_params_t{ levels:3, ratio:12.0, tau0:256, tau_growth:2.0, delta0:-1 };
    let rc = unsafe {
        hlb_sssp_u32_c(4, indptr.as_ptr(), to.as_ptr(), w.as_ptr(), 0, &p, dist.as_mut_ptr())
    };
    println!("rc={rc} dist={:?}", dist);
}
```

Build & run:

```bash
RUSTFLAGS="-L native=." cargo run
```

---

## Java (JNI)

Java class:

```java
public class HLB {
  static { System.loadLibrary("hlb_sssp"); }  // ensure the lib is on java.library.path
  private static native int hlb_sssp_u32_c(int n, int[] indptr, int[] to, int[] w,
                                           int source, int levels, double ratio,
                                           int tau0, double tau_growth, int delta0,
                                           int[] dist);
  public static void main(String[] args){
    int[] indptr={0,2,3,3,4}, to={1,2,2,3}, w={5,1,2,1}, dist=new int[4];
    int rc = hlb_sssp_u32_c(4, indptr, to, w, 0, 3, 12.0, 256, 2.0, -1, dist);
    System.out.println("rc="+rc+" dist="+java.util.Arrays.toString(dist));
  }
}
```

JNI C glue (example signature mapping to C ABI):

```c
#include <jni.h>
#include "hlb/hlb_c.h"

JNIEXPORT jint JNICALL Java_HLB_hlb_1sssp_1u32_1c
  (JNIEnv* env, jclass, jint n, jintArray jindptr, jintArray jto, jintArray jw, jint source,
   jint levels, jdouble ratio, jint tau0, jdouble tau_growth, jint delta0, jintArray jdist)
{
  hlb_params_t p = { levels, ratio, tau0, tau_growth, delta0 };
  jint* indptr = (*env)->GetIntArrayElements(env, jindptr, NULL);
  jint* to     = (*env)->GetIntArrayElements(env, jto,     NULL);
  jint* w      = (*env)->GetIntArrayElements(env, jw,      NULL);
  jint* dist   = (*env)->GetIntArrayElements(env, jdist,   NULL);
  int rc = hlb_sssp_u32_c(n, indptr, to, w, source, &p, (uint32_t*)dist);
  (*env)->ReleaseIntArrayElements(env, jindptr, indptr, 0);
  (*env)->ReleaseIntArrayElements(env, jto,     to,     0);
  (*env)->ReleaseIntArrayElements(env, jw,      w,      0);
  (*env)->ReleaseIntArrayElements(env, jdist,   dist,   0);
  return rc;
}
```

Build the JNI `.so/.dylib` and ensure both it and `libhlb_sssp` are loadable by the JVM.

---

## C# / .NET (P/Invoke)

```csharp
using System;
using System.Runtime.InteropServices;

[StructLayout(LayoutKind.Sequential)]
struct HLBParams { public int levels; public double ratio; public int tau0; public double tau_growth; public int delta0; }

class Program {
  [DllImport("hlb_sssp", CallingConvention = CallingConvention.Cdecl)]
  static extern int hlb_sssp_u32_c(int n, int[] indptr, int[] to, int[] w, int source, ref HLBParams p, uint[] dist);

  static void Main(){
    int[] indptr={0,2,3,3,4}, to={1,2,2,3}, w={5,1,2,1};
    uint[] dist=new uint[4];
    var p = new HLBParams{ levels=3, ratio=12.0, tau0=256, tau_growth=2.0, delta0=-1 };
    int rc = hlb_sssp_u32_c(4, indptr, to, w, 0, ref p, dist);
    Console.WriteLine($"rc={rc} dist=[{string.Join(",",dist)}]");
  }
}
```

> On Windows, ensure the DLL is named `hlb_sssp.dll` and resides alongside the executable or on `PATH`. On macOS/Linux, ensure `DYLD_LIBRARY_PATH`/`LD_LIBRARY_PATH` (or rpath) includes your library directory.

---

## License

[MIT](https://github.com/sarvessveeriyah2312/hlb-sssp/blob/main/LICENSE)
