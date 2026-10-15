# Bare-Metal Preemptive & Cooperative Micro-RTOS for ARM Cortex-M3

A lightweight, deterministic real-time operating system engineered from first principles for the **STM32F103RB (ARM Cortex-M3)**. This project implements exception-driven context switching, dual-stack isolation, atomic synchronization, and deterministic timekeeping without third-party vendor HAL libraries.

## Table of Contents

- [1. Architectural Overview](#1-architectural-overview)
- [2. Memory Geometry and Stack Partitioning](#2-memory-geometry-and-stack-partitioning)
- [3. Task Control Block (TCB) Architecture](#3-task-control-block-tcb-architecture)
- [4. Synthetic Exception Frames](#4-synthetic-exception-frames)
- [5. System Bootstrap and ISB Synchronization](#5-system-bootstrap-and-isb-synchronization)
- [6. PendSV Context-Switching Engine](#6-pendsv-context-switching-engine)
- [7. Atomic Synchronization and Memory Ordering](#7-atomic-synchronization-and-memory-ordering)
- [8. Preemptive Timekeeping with SysTick](#8-preemptive-timekeeping-with-systick)
  
---

## 1. Architectural Overview

### Key Features

- **Hybrid Scheduling:** 1 ms preemptive time-slicing via SysTick, paired with low-overhead voluntary yielding via PendSV.
- **Context Management:** Dual-frame model with hardware-stacked caller registers and software-saved callee registers (R4–R11).
- **Dual-Stack Operation:** Uses the Main Stack Pointer (MSP) for kernel exceptions and private Process Stack Pointer (PSP) frames for user tasks.
- **Atomic Synchronization:** Non-blocking mutex acquisition using ARMv7-M exclusive-access instructions (`LDREX` / `STREX`).
- **Memory Consistency:** Explicit Data Memory Barriers (`DMB`) and GCC inline assembly constraints.
- **Task Delay and State Management:** Millisecond countdown timers and automatic blocked-to-ready state transitions.
- **Stack Watermarking:** Canary signature patterns for inspecting stack usage in the debugger.

### Target Hardware and Serial Interface

| Component         | Configuration                        |
| ----------------- | ------------------------------------ |
| Development board | STMicroelectronics NUCLEO-F103RB     |
| Processor core    | ARM Cortex-M3 (ARMv7-M architecture) |
| SRAM              | 20 KB                                |
| Flash             | 128 KB                               |
| System clock      | 8 MHz internal RC oscillator (HSI)   |
| Serial interface  | USART2                               |
| UART pins         | PA2 (TX), PA3 (RX)                   |
| Baud rate         | 115200 baud                          |
| Serial format     | 8-N-1                                |

### USART2 Baud Rate Derivation

The USART2 peripheral clock is assumed to be 8 MHz. With the STM32 USART baud-rate generator operating in its standard 16-times oversampling mode:

[
USARTDIV= (f\_{PCLK}) / {16 \* Baud_rate }
]

Substituting the target clock and baud rate:

[
USARTDIV=(8,000,000) / (16 \* 115,200)
]

[
USARTDIV = 4.3402778
]

The `USART2_BRR` register encodes the mantissa and fractional portions of this divisor.

| Field                | Calculation                               | Value |
| -------------------- | ----------------------------------------- | ----- |
| Mantissa (bits 15:4) | Integer part of `USARTDIV`                | `0x4` |
| Fraction (bits 3:0)  | 0.3402778 \* 16 = 5.444 ==> rounded off 5 | `0x5` |

Therefore:

```c
USART2->BRR = 0x0045;
```

This value assumes an 8 MHz USART2 peripheral clock. The actual peripheral clock configuration must match this assumption.

---

## 2. Memory Geometry and Stack Partitioning

The Cortex-M3 uses a **full-descending stack model**: stack allocation proceeds toward lower addresses, with the stack pointer decremented when space is allocated.

### SRAM Address Map

The STM32F103RB provides 20 KB of SRAM, spanning:

- **SRAM start:** `0x20000000`
- **SRAM end (exclusive):** `0x20005000`
- **Initial MSP value:** `0x20005000`

### Proposed Memory Partition

| Region                           | Address Range             |  Size |
| -------------------------------- | ------------------------- | ----: |
| Kernel / exception-handler stack | `0x20004800`–`0x20005000` |  2 KB |
| Task 0 private stack             | `0x20004400`–`0x20004800` |  1 KB |
| Task 1 private stack             | `0x20004000`–`0x20004400` |  1 KB |
| Task 2 private stack             | `0x20003C00`–`0x20004000` |  1 KB |
| Task 3 private stack             | `0x20003800`–`0x20003C00` |  1 KB |
| Remaining SRAM                   | `0x20000000`–`0x20003800` | 14 KB |

The remaining 14 KB is intended for `.data`, `.bss`, and other static structures.

The final linker configuration must ensure that these regions do not overlap with global variables, heap allocations, startup data, or other SRAM objects. The reserved 14 KB must not be considered available in its entirety unless the actual linker map confirms that it is free.

---

## 3. Task Control Block (TCB) Architecture

The `TCB_t` structure stores execution state and scheduling metadata for each task.

### TCB Memory Layout

| Offset        | Field           | Description                                       |
| ------------- | --------------- | ------------------------------------------------- |
| `0x00`        | `psp_value`     | Saved task stack pointer                          |
| `0x04`        | `task_handler`  | Task entry-point function pointer                 |
| `0x08`        | `state`         | Task state (`TASK_READY = 0`, `TASK_BLOCKED = 1`) |
| `0x0C`        | `ticks_to_wait` | Millisecond sleep countdown                       |
| `0x10`–`0x1F` | Reserved        | Four 32-bit padding words                         |

The intended TCB size is **32 bytes**, equivalent to (2^5) bytes.

This allows assembly code to calculate the TCB address using a left shift by five bits.

```asm
; R1 = base address of user_tasks
; R2 = task index

ADD R3, R1, R2, LSL #5
```

The address calculation is:

[
R3 =user_tasks+(task_index \* 32)
]

---

## 4. Synthetic Exception Frames

A newly created task has never been preempted. Without initialization, attempting to restore its execution context would load invalid values into registers and potentially trigger a fault.

The `init_tasks_stack()` function prepares a synthetic stack frame so that a new task can enter the normal context-restoration mechanism.

### 4.1 Hardware-Saved Frame

The Cortex-M exception mechanism stacks eight 32-bit registers.

| Register | Purpose                        |
| -------- | ------------------------------ |
| R0–R3    | Argument and working registers |
| R12      | Scratch register               |
| LR       | Link register                  |
| PC       | Program counter                |
| xPSR     | Program status register        |

The synthetic frame initializes the following values:

- **xPSR:** `0x01000000`, setting the Thumb-state bit.
- **PC:** Task entry address, represented according to the toolchain's Thumb function-pointer conventions.
- **LR:** `0xFFFFFFFD`, an `EXC_RETURN` value indicating a return to Thread mode using PSP.

### 4.2 Software-Saved Frame

The RTOS separately saves R4–R11 in software. The initial frame must contain these eight registers in the order expected by the context-switching assembly.

The proposed synthetic frame occupies 16 words, or 64 bytes:

| Frame component      |                    Size | Purpose                  |
| -------------------- | ----------------------: | ------------------------ |
| Hardware-saved frame |      8 words / 32 bytes | R0–R3, R12, LR, PC, xPSR |
| Software-saved frame |      8 words / 32 bytes | R4–R11                   |
| **Total**            | **16 words / 64 bytes** | Initial task context     |

The initialized software-saved frame can contain recognizable canary signatures for debugging.

The resulting stack pointer is stored in:

```c
user_tasks[i].psp_value
```

The synthetic frame layout must match the precise register-save and register-restore order implemented by `PendSV_Handler` and `launch_task_0()`.

---

## 5. System Bootstrap and ISB Synchronization

Task 0 is started through `launch_task_0()` without requiring a previously running task to be preempted.

### Initialization Sequence

1. Retrieve Task 0's saved stack pointer from its TCB.
2. Restore R4–R11 using `LDMIA`.
3. Load the appropriate stack address into the hardware PSP using `MSR PSP`.
4. Set `CONTROL.SPSEL = 1` to select PSP for Thread mode.
5. Execute an Instruction Synchronization Barrier (`ISB`).
6. Restore the remaining registers according to the synthetic stack-frame layout.
7. Transfer execution to Task 0 using the intended launch sequence.

### Why ISB Is Required

After writing `CONTROL`, an `ISB` ensures subsequent instruction execution observes the updated architectural state.

```asm
MSR CONTROL, R0     ; Select PSP for Thread mode
ISB                 ; Synchronize subsequent instruction execution
```

### Initial Stack Restoration

The launch routine may restore the synthetic frame manually, but every stack adjustment must correspond to the actual frame layout.

For example:

```asm
ADD SP, SP, #4
```

is valid for skipping a word only if that word genuinely occupies the current stack position and the subsequent branch uses the correct restored program counter or exception-return value.

The initialization routine and launch routine must agree on which stack pointer is active and where each register is stored.

---

## 6. PendSV Context-Switching Engine

`PendSV_Handler` performs task context switching. A naked assembly routine can be used to prevent compiler-generated prologue and epilogue instructions from interfering with manually managed stack frames.

### Phase 1: Save the Outgoing Context

1. Read the active task PSP using `MRS R0, PSP`.
2. Save R4–R11 using `STMDB`.
3. Calculate the outgoing task's TCB address.
4. Store the updated PSP in `user_tasks[current_task].psp_value`.

Example:

```asm
MRS R0, PSP
STMDB R0!, {R4-R11}

LDR R1, =user_tasks
ADD R3, R1, R2, LSL #5
STR R0, [R3]
```

Here, `R2` must contain the outgoing task index.

### Phase 2: Select the Next Ready Task

The scheduler searches for a task whose state is `TASK_READY`.

For four tasks, the round-robin index can wrap using a bit mask:

```asm
ADD R3, R3, #1
AND R3, R3, #0x03
```

The scheduler checks the task-state field at offset `0x08` and continues searching until it finds a runnable task.

The implementation must guarantee that at least one task remains runnable or provide an idle task for situations in which all other tasks are blocked.

### Phase 3: Restore the Incoming Context

1. Calculate the incoming task's TCB address.
2. Load its saved PSP value.
3. Restore R4–R11 using `LDMIA`.
4. Update the hardware PSP using `MSR PSP`.
5. Load the appropriate `EXC_RETURN` value into LR.
6. Execute `BX LR` to initiate exception return.

Example TCB lookup:

```asm
LDR R1, =user_tasks
ADD R0, R1, R3, LSL #5
LDR R0, [R0]
LDMIA R0!, {R4-R11}
MSR PSP, R0
```

This example assumes that the first word of each TCB contains `psp_value`. The actual assembly must preserve the selected task index and all values still needed by the handler.

### Exception Return and Tail-Chaining

During a valid exception return, the Cortex-M hardware restores the hardware-stacked registers and resumes the selected execution context.

ARM tail-chaining can reduce exception-handling latency when another exception is pending. It does not guarantee that a complete task switch finishes in six clock cycles; actual timing depends on the instructions executed, memory behavior, exception state, and processor implementation.

---

## 7. Atomic Synchronization and Memory Ordering

Thread synchronization is implemented using a non-blocking mutex based on ARMv7-M exclusive-access instructions.

### 7.1 Hardware Test-and-Set

The intended `Hardware_Try_Lock()` sequence is:

1. `LDREX` reads the lock and establishes an exclusive reservation.
2. `CMP` checks whether the lock equals zero.
3. `IT EQ` conditionally executes the next instruction.
4. `STREXEQ` attempts to write `1` only when the lock is free and the exclusive reservation remains valid.

The status returned by `STREX` must be checked. If the exclusive store fails, the caller must retry or report that acquisition failed.

### 7.2 Preemption Safety

The exclusive monitor determines whether an exclusive store can complete successfully. Interrupts and exceptions can affect exclusive-access behavior, so the implementation must not assume that a reservation remains valid across arbitrary exception handling or scheduling activity.

A failed exclusive store must be handled explicitly.

### 7.3 GCC Inline Assembly Constraints

| Constraint | Purpose                                                                                           |
| ---------- | ------------------------------------------------------------------------------------------------- |
| `=&r`      | Early-clobber output constraint that prevents unsafe register overlap with inputs                 |
| `cc`       | Declares that condition-code flags are modified                                                   |
| `memory`   | Compiler barrier indicating that the assembly may access memory beyond explicitly listed operands |

These constraints control compiler optimization and register allocation. They do not, by themselves, provide hardware memory ordering.

### 7.4 Acquire-Release Semantics

Appropriate `DMB` instructions can enforce memory ordering around successful lock acquisition and release.

**Acquire barrier:**

```c
__asm volatile ("DMB" ::: "memory");
```

This orders protected memory accesses after successful acquisition.

**Release barrier:**

```c
__asm volatile ("DMB" ::: "memory");
mutex->lock = 0;
```

This ensures preceding protected memory accesses are ordered before the lock is released.

A complete mutex implementation must ensure that:

- Only the owner releases the lock.
- Failed acquisitions never enter the critical section.
- The `STREX` result is checked.
- The lock's memory access and barrier sequence matches the intended synchronization design.

On contention, a task may yield by setting `PENDSVSET` in `SCB_ICSR`, provided the scheduler and task-state logic make this appropriate. Requesting a context switch alone does not block the task or guarantee that another task acquires the lock.

---

## 8. Preemptive Timekeeping with SysTick

SysTick provides periodic timekeeping for task delays and scheduler wake-ups.

### 8.1 Periodic Tick Configuration

For an 8 MHz processor clock and a 1 ms tick:

[
T\_{\text{tick}}=0.001\text{ s}
]

The number of clock cycles per tick is:

[
N=f\_{\text{CPU}}\times T\_{\text{tick}}
]

[
N=8,000,000\times0.001=8,000
]

The SysTick reload register is configured as follows:

```c
SysTick->LOAD = 8000 - 1;  /* 1 ms at 8 MHz */
```

The reload value is `7999` because the timer counts through the reload value, including zero.

This assumes SysTick uses the 8 MHz processor clock and that the clock configuration is correct.

### 8.2 SysTick Handler

On each tick, `SysTick_Handler`:

1. Decrements `ticks_to_wait` for blocked tasks.
2. Transitions tasks to `TASK_READY` when their delay expires.
3. Sets the PendSV pending bit when scheduling is required.

The exact scheduling policy determines whether a context switch occurs on every tick or only when necessary.

### 8.3 Exception Priority Architecture

PendSV is assigned the lowest priority so context switching can be deferred until higher-priority interrupts have been serviced.

Example configuration:

- **PendSV:** Lowest priority, using the appropriate priority encoding.
- **SysTick:** Higher priority than PendSV.

If SysTick preempts an active PendSV handler, the interrupted handler context is stacked on MSP because both execute in Handler mode.

The nested exception return uses:

```asm
EXC_RETURN = 0xFFFFFFF1
```

This value represents a return to Handler mode using MSP. The processor restores the interrupted handler context and resumes PendSV.

---

## Learning Objectives

This project develops practical understanding of:

- ARM Cortex-M exception handling and exception return.
- Task context switching through PendSV.
- MSP and PSP stack management.
- Interrupt-driven scheduling and task delays.
- Atomic synchronization using exclusive-access instructions.
- Memory ordering and GCC inline assembly constraints.
- Bare-metal embedded software development without vendor HAL libraries.

