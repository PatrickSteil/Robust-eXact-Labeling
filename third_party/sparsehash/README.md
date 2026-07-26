# Vendored sparsehash (google::sparse_hash_map)

Source: https://github.com/sparsehash/sparsehash
Commit: `1dffea3d917445d70d33d0c7492919fc4408fe5c` (2020-08-12), version 2.0.2.
License: BSD 3-Clause, see `COPYING`.

Only `sparsehash/sparse_hash_map` is used by this project (see
`include/sample_tree_storage.h`, Appendix A.2 of the paper), so only its
include closure is vendored, rather than the whole library:

```
sparsehash/sparse_hash_map
sparsehash/sparsetable
sparsehash/template_util.h
sparsehash/type_traits.h
sparsehash/internal/sparsehashtable.h
sparsehash/internal/hashtable-common.h
sparsehash/internal/libc_allocator_with_realloc.h
sparsehash/internal/sparseconfig.h
```

`dense_hash_map`, `dense_hash_set`, and `sparse_hash_set` are not vendored
since nothing here uses them.

## About `sparseconfig.h`

Upstream sparsehash is normally built with autotools: `./configure` probes
the platform and `make` distills the result down to
`sparsehash/internal/sparseconfig.h` (see upstream's `Makefile.am`, target
`src/sparsehash/internal/sparseconfig.h`). That step needs autoconf/automake
installed, which is a heavy, non-CMake dependency to impose on everyone who
just wants to build RXL.

Instead, the `sparseconfig.h` checked in here is that same generated output,
produced once (via `./configure && make src/sparsehash/internal/sparseconfig.h`
on upstream commit `1dffea3`) and committed directly. Its contents boil down
to "this is a standard POSIX-ish system with `<stdint.h>`, `<sys/types.h>`,
`long long`, and `std::hash` in `<functional>`", which holds for the
Linux/macOS/BSD toolchains (glibc, musl, libc++) this project targets. It is
not autodetected per-build.

If you ever need to build on a platform where those assumptions don't hold,
regenerate it: clone sparsehash, run `./configure`, then
`make src/sparsehash/internal/sparseconfig.h`, and copy the result over this
file.
