# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.0] - 2026-07-06

### Added

- Initial release: `no_std` embedded-hal 1.0 driver for the ADXL362.
- Async (default) and blocking (`blocking` feature) surfaces via `maybe-async`.
- Activity/inactivity detection, autosleep, FIFO (stream/FIFO/triggered modes).
- Integer milli-g output by default; `f32` g/°C conversions via the `float` feature.
- `defmt::Format` derives on public types via the `defmt` feature.

[Unreleased]: https://github.com/danielegarciav/adxl362-rs/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/danielegarciav/adxl362-rs/releases/tag/v0.1.0
