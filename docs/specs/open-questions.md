# Open Questions & Next Steps

## Outstanding Decisions

1. **Tolerance presets:** Do we ship coarse/medium/fine profiles or require explicit `--tol` input for every run?
2. **GUI priority:** Should the egui/wgpu preview land during M2 or wait until Mode B stabilizes?
3. **Commercial differentiation:** Keep core Apache-2.0 and sell OCCT bundle/support, or open-source everything?
4. **Cloud/remote mode:** Provide turnkey job queue (Redis/NATS) in the daemon, or stay local-only initially?

## Immediate Next Steps

1. Confirm answers to the questions above to unblock roadmap sequencing beyond M2.
2. Approve milestone timeline and dataset coverage (reference meshes for regression tests).
3. Kick off **M0**: scaffold the Rust workspace, import sample meshes, establish CI, and ensure `cargo test` passes with placeholder crates.

Remember: ship features sequentially, add/extend tests after each step, and keep the project compiling before progressing.

