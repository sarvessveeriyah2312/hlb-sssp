#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int   levels;
    double ratio;
    int   tau0;
    double tau_growth;
    int   delta0;
} hlb_params_t;

int hlb_sssp_u32_c(
    int n,
    const int* indptr,
    const int* to,
    const int* w,
    int source,
    const hlb_params_t* params,
    uint32_t* dist_out
);

#ifdef __cplusplus
}
#endif
