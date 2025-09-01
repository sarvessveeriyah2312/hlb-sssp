file(REMOVE_RECURSE
  "libhlb_sssp.a"
  "libhlb_sssp.pdb"
)

# Per-language clean rules from dependency scanning.
foreach(lang CXX)
  include(CMakeFiles/hlb_sssp_static.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
