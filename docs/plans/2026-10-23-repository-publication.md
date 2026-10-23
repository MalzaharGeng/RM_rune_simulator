# Repository Publication Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Publish the existing Windows simulator with complete sources, dependencies, runnable release, and verifiable documentation.

**Architecture:** Keep the original live project untouched. Place Rust sources and embedded assets at the repository root, add rm_interfaces source and locked environment manifests, distribute binary dependencies through a versioned GitHub Release.

**Tech Stack:** Rust 1.98.1, Bevy 0.17.2, Avian3D 0.4, ROS 2 Jazzy, MSVC, LLVM, Pixi.

### Task 1: Audit and stage
Read current source, original upstream and local snapshots; copy only active source/assets, messages and lockfiles. Compare SHA256 of all copied simulator files.

### Task 2: Make deployment reproducible
Add config/defaults.json and ignored local.json overrides. Add scripts/bootstrap-windows.ps1, build.ps1, run.ps1 and download-runtime.ps1. Fix explicit std_msgs declaration in rm_interfaces. Keep simulator .rs behavior unchanged.

### Task 3: Validate
Run existing spin unit tests, inspect PowerShell syntax, test dependency closure, start an isolated runtime with a clean PATH, receive and validate UDP ground truth. Do not stop existing user processes.

### Task 4: Document and publish
Write README.md, docs/ENVIRONMENT.md, docs/UPSTREAM_COMPARISON.md, docs/TESTING.md and evidence summaries. Scan tracked files for secrets/backups. Commit, create the authorized GitHub repository, push main and upload the runtime release. Verify remote commit and release hashes. Copy README and work report to Desktop.
