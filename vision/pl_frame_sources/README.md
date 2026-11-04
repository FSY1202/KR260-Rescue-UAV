# Frozen FRM2 inputs

The six RTL sources and `constraints/adapter20261026.xdc` were copied byte for byte from `fpga/kr260_camera_stream`. `source_manifest.json` records their original SHA-256 values, byte counts, source paths and frozen destinations. `SHA256SUMS` lists paths relative to the servo candidate root. The manifest also records the source project-generator and device-tree hashes used when preparing the candidate.

The candidate builds from this directory and never sources the sibling camera project. Do not edit these files to implement servo changes; use the candidate's own `rtl`, `constraints` and `vivado` directories.
