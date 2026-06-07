<!--
SPDX-License-Identifier: Apache-2.0
SPDX-FileCopyrightText: Copyright (c) 2025 Synopsys, Inc.
-->

# UART Echo - RISC-V AIA SMP Demo

Demonstrates RISC-V AIA with multiple harts (SMP configuration).

## Overview

Shows AIA features in SMP:
- Multi-hart boot with per-hart IMSIC initialization
- APLIC MSI routing to specific harts
- Software MSI injection (GENMSI) targeting individual harts
- Dynamic interrupt routing between harts

**Note**: IMSIC EIID enables are local to each hart. The demo pins an
initialization thread to hart 1 so both IMSIC files enable the UART EIID before
the APLIC routing tests target either hart.

## Building

```bash
west build -p -b qemu_riscv32/qemu_virt_riscv32/aia-imsic/smp samples/drivers/interrupt_controller/intc_riscv_aia_smp
```

## Running

```bash
west build -t run
```

## Configuration

- **Board**: qemu_riscv32/qemu_virt_riscv32/aia-imsic/smp
- **CPUs**: 2 harts
- **IMSIC**: One per hart (0x24000000, 0x24001000)
- **APLIC**: Routes to either hart dynamically
