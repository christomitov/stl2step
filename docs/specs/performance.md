# Performance & QA Targets

## Runtime & Memory

| Dataset | Mode A Target | Mode B Target | Notes |
| --- | --- | --- | --- |
| 200k-tri mechanical part | < 30 s | < 2 min | Laptop-class 8-core CPU. |
| 2M-tri consumer enclosure | < 2 min | < 10 min | Requires 16–32 GB RAM. |
| Memory ceiling | ≤ 3× input size | ≤ 5× input size | Streaming chunker kicks in above 2M tris. |

Parallelize per-region operations and spill intermediate buffers to disk when exceeding memory watermark. Provide telemetry counters for time spent per stage.

## QA Outputs

- STEP validation: closed shell or solid flag, self-intersection status.
- Numeric tolerances: max/mean point-to-surface deviation, edge gap statistics, condition numbers.
- Breakdown of analytic vs freeform surface area.
- Optional preview bundle for wgpu viewer containing deviation heatmaps (`.ply` or `.glb`).

## Telemetry & Reporting

- Opt-in only; prompt on first run and store preference in `$HOME/.m2b/settings.toml`.
- Capture anonymized performance counters (tri count, runtime per stage, success/failure reason).
- Crash handler writes `.partial` files and stack metadata; user can opt to share via CLI flag (`m2b convert ... --send-crash`).

