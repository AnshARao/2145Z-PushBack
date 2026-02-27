# AGENTS.md

## Cursor Cloud specific instructions

This is a VEX V5 Robotics Competition (V5RC) codebase for team 2145Z. It contains multiple independent embedded C++ robot firmware projects targeting the VEX V5 Brain (ARM Cortex-A9).

### Project Types

There are two toolchain families:

- **PROS projects** (majority): Use `arm-none-eabi-gcc` cross-compiler with GNU Make. Build with `make` or `prosv5 make` from each project directory.
- **VEXcode projects** (`2145-RwTemp`, `RW-LVGL`): Use `clang` + proprietary VEX SDK. These **cannot be built** in this environment because the VEX SDK (`$HOME/sdk/`) is proprietary and not included. Skip these when building.

### Building

Each project is independent — there is no top-level build system. To build a PROS project, `cd` into its directory and run `make`. The primary/most complete project is `2145Z-Highlander`.

Successfully buildable PROS projects:
- `2145Z-Highlander` — main competition code
- `2145Z-EzTemp` — EZ-Template based competition code
- `2145Z-Highlander/EZ-Template-Example-Project` — example/template project

Other PROS projects (`2145Z-MOA`, `2145Z-Ri3d`, `2145-Template`, `2145Z-LemLib`) have pre-existing compile or link errors in the source code.

`ellensburger-main` has no build system (no Makefile, no `project.pros`).

### Linting and Testing

There are no automated test suites or linting configurations in this codebase. The code is validated through successful compilation (`make` producing `.bin` files). There are no `eslint`, `prettier`, `clang-tidy`, or similar configs.

### Key Commands

- **Build a PROS project:** `cd <project-dir> && make`
- **Clean build:** `cd <project-dir> && make clean && make`
- **Build with PROS CLI:** `cd <project-dir> && prosv5 make`
- **PROS CLI** is installed at `~/.local/bin/prosv5` — ensure `$HOME/.local/bin` is on `PATH`.

### Notes

- Build output (`.bin` files) goes to `<project-dir>/bin/`. These are firmware binaries meant to be uploaded to the VEX V5 Brain via USB, not executed on the host.
- The `arm-none-eabi-gcc` cross-compiler and `pros-cli` Python package must be installed (handled by the update script).
