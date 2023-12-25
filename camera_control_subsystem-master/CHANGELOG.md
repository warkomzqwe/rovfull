# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.2.0] - 2019-10-07
### Added
- This changelog.
- **New** reset position api for camera.
- **New** clear all telemetry key/value pairs from dictionary.

### Changed
- **API change** Target messages are now steps messages using integers for steps.
- Telemetry Listener only sends new incoming data to callbacks instead of whole dictionary.