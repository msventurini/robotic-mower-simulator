All changes:
```
hyperfine --warmup 2 "./target/release/mestrado-rust"
Benchmark 1: ./target/release/mestrado-rust
  Time (mean ± σ):      2.389 s ±  0.262 s    [User: 2.391 s, System: 1.081 s]
  Range (min … max):    2.101 s …  2.865 s    10 runs
```

No cpu native:
```
ruan@Nanas-MacBook-Air mestrado-rust % hyperfine --warmup 2 "./target/release/mestrado-rust"
Benchmark 1: ./target/release/mestrado-rust
  Time (mean ± σ):      2.419 s ±  0.172 s    [User: 2.398 s, System: 1.092 s]
  Range (min … max):    2.178 s …  2.721 s    10 runs
```

no build opttimisations:
```
ruan@Nanas-MacBook-Air mestrado-rust % hyperfine --warmup 2 "./target/release/mestrado-rust"
Benchmark 1: ./target/release/mestrado-rust
  Time (mean ± σ):      2.670 s ±  0.293 s    [User: 2.401 s, System: 1.410 s]
  Range (min … max):    2.227 s …  3.082 s    10 runs
```

no grassmap mods (but build optimisations)"
```
ruan@Nanas-MacBook-Air mestrado-rust % hyperfine --warmup 2 "./target/release/mestrado-rust"
Benchmark 1: ./target/release/mestrado-rust
  Time (mean ± σ):      2.725 s ±  0.214 s    [User: 2.555 s, System: 1.261 s]
  Range (min … max):    2.505 s …  3.175 s    10 runs
```

no grassmap and not build""
```
ruan@Nanas-MacBook-Air mestrado-rust % hyperfine --warmup 2 "./target/release/mestrado-rust"
Benchmark 1: ./target/release/mestrado-rust
  Time (mean ± σ):      2.671 s ±  0.117 s    [User: 2.520 s, System: 1.211 s]
  Range (min … max):    2.546 s …  2.854 s    10 runs
```