# DIY U2D2 Clone — STM32F411 USB-to-Dynamixel Bridge

A low-cost USB-to-Dynamixel bridge built on an STM32F411CEU6 (BlackPill) board. Acts as a transparent USB CDC ↔ Single-Wire Half-Duplex UART bridge for controlling DYNAMIXEL servos (e.g. MX-28T) at **1 Mbps, Protocol 2.0** from a PC — a functional replacement for the ROBOTIS U2D2.

Works with **DYNAMIXEL Wizard 2.0**

---

## How It Works

```
┌─────────┐   USB    ┌──────────────────────┐  Single-Wire ┌──────────┐
│   PC    │◄───────► │  STM32F411 BlackPill │  Half-Duplex │  MX-28T  │
│ (Ubuntu)│  CDC VCP │  USB CDC ↔ USART1    │◄───────────► │  Servo   │
└─────────┘          └──────────────────────┘   TTL 1 Mbps └──────────┘
```

The STM32 is a **transparent bridge**: all DYNAMIXEL protocol logic stays on the PC. The firmware only forwards bytes between the USB Virtual COM Port and the single-wire UART bus, handling the half-duplex direction switching internally.

- **USB side:** USB CDC (Virtual COM Port) → appears as `/dev/ttyACM0`
- **UART side:** USART1 in Single-Wire (Half-Duplex) mode on **PA9**
- **Data path:** DMA + UART Idle Line Detection (non-blocking, no dropped packets at 1 Mbps)

---

## Hardware

### Bill of Materials

| Component | Qty | Notes |
|---|---|---|
| STM32F411CEU6 (BlackPill) | 1 | USB Type-C version |
| Resistor 4.7 kΩ | 1 | Data line pull-up |
| 12 V power supply (≥2 A) | 1 | For the servo (separate from USB) |
| 3-pin DYNAMIXEL cable | 1 | To servo |
| ST-Link V2 | 1 | For flashing |

### Pin Config

| STM32F411 Pin | Dynamixel MX-28T Pin | Electrical Connection / External Component | Function |
| :--- | :--- | :--- | :--- |
| **3.3V** | — | Connected to the top terminal of the **4.7k Ω Resistor** | Pull-up voltage bias source |
| **PA9** | **DATA** (White/Yellow) | Connected to the bottom terminal of the **4.7k Ω Resistor** | Main data line (Half-Duplex Open-Drain) |
| **GND** | **GND** (Black) | Tied directly to the **GND (-) of the 12V Adaptor** | Common Ground (Mandatory for signal reference) |
| — | **VCC** (Red) | Connected to **(+) 12V Adaptor** via an **ON/OFF Switch** | Main actuator power supply |


### ⚠️ Critical Hardware Notes (learned the hard way)

These two points destroyed a BlackPill **and** a servo during development. Do not skip them.

1. **Pull-up MUST go to 3.3 V, not 5 V.**
   PA9 is 5 V-tolerant *statically*, but a 5 V pull-up keeps the internal clamp diode conducting (idle reads ~4.5–4.8 V instead of 5 V). Under switching + servo current draw, transient spikes exceed the absolute max and the clamp diode eventually fails — killing the chip. With a 3.3 V pull-up the diode never conducts. MX-28T logic-high threshold is ~2 V, so 3.3 V is more than enough.

2. **Common ground is mandatory.** Servo GND, BlackPill GND, and 12 V supply GND must meet at one solid point with thick, short wire. A floating or thin ground forces motor return current through the PA9 data line.

3. **Servo power is separate.** Never power the servo from the BlackPill's USB 5 V. Only the ground is shared.

---

## STM32CubeIDE Configuration (.ioc)

### USB
- **Connectivity → USB_OTG_FS:** Mode = `Device_Only`
- **Middleware → USB_DEVICE:** Class = `Communication Device Class (Virtual Port COM)`

### USART1 (Single-Wire Half-Duplex)
- **Connectivity → USART1:** Mode = `Single Wire (Half-Duplex)` (PA9 becomes the bidirectional data pin; PA10 is freed)
- Parameters:
  - Baud Rate: `1000000`
  - Word Length: `8 Bits`
  - Parity: `None`
  - Stop Bits: `1`
- **GPIO (PA9):** Alternate Function Open-Drain, No internal pull-up/pull-down (external 4.7 kΩ handles it), Speed = Very High

### DMA (USART1 → DMA2)
| Stream | Direction | Mode | Data Width |
|---|---|---|---|
| DMA2 (e.g. Stream 2 / 5) | USART1_RX | Circular | Byte |
| DMA2 (e.g. Stream 7 / 6) | USART1_TX | Normal | Byte |

### NVIC — Interrupt Priorities (important!)
Default priorities are all `0`, which lets the USB ISR preempt the UART ISR mid-`CDC_Transmit_FS` → buffer corruption / deadlock. Set:

| Interrupt | Preemption Priority |
|---|---|
| USART1 global | 1 |
| DMA2 stream (RX) global | 1 |
| DMA2 stream (TX) global | 1 |
| **USB OTG FS global** | **2** (lower priority than UART/DMA) |

### Clock Configuration
HSE 25 MHz crystal → must produce **exactly 48 MHz** on the USB clock (CLK48), and ~96 MHz SYSCLK:

```
PLLM = /25, PLLN = ×192, PLLP = /2  → SYSCLK = 96 MHz
PLLQ = /4                            → USB    = 48 MHz (exact)
```

---

## Firmware Overview

Three files are touched: `main.c`, `usbd_cdc_if.c` (and the CubeMX-generated `usart.c` / `dma.c`).

### Buffers (`main.c`, USER CODE BEGIN PV)
```c
#define RX_BUF_SIZE 1024
#define TX_BUF_SIZE 1024

uint8_t uart_rx_buf[RX_BUF_SIZE];
uint8_t uart_tx_buf[TX_BUF_SIZE];
volatile uint8_t tx_active = 0;
```
> Buffer size matters: 256 bytes was too small for full control-table reads and caused intermittent timeouts. 1024 is safe.
### Startup (`main.c`, USER CODE BEGIN 2)
```c
HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rx_buf, RX_BUF_SIZE);
__HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);   // disable half-transfer IRQ
```

### Callbacks (`main.c`, USER CODE BEGIN 4)
```c
// TX to servo done → return to listen (RX) mode
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        tx_active = 0;
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);   // heartbeat LED (optional)
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rx_buf, RX_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
}

// Servo replied (idle line detected) → forward to PC over USB
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART1) {
        if (Size > 0 && Size <= RX_BUF_SIZE)
            CDC_Transmit_FS(uart_rx_buf, Size);   // non-blocking, ignore return
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rx_buf, RX_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
}

// Recovery from overrun/noise/framing errors (e.g. data-cable hot-plug)
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        volatile uint32_t tmp;
        tmp = huart->Instance->SR;
        tmp = huart->Instance->DR;   // SR→DR read clears error flags on F4
        (void)tmp;
        tx_active = 0;
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rx_buf, RX_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
}
```

### PC → Servo path (`usbd_cdc_if.c`, CDC_Receive_FS)
```c
// USER CODE BEGIN PRIVATE_VARIABLES
extern uint8_t uart_tx_buf[];
extern volatile uint8_t tx_active;
extern uint8_t uart_rx_buf[];
#define TX_BUF_SIZE 1024
#define RX_BUF_SIZE 1024
// USER CODE END PRIVATE_VARIABLES

static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len) {
    uint32_t length = *Len;
    if (length > 0 && length <= TX_BUF_SIZE && !tx_active) {
        tx_active = 1;
        HAL_UART_DMAStop(&huart1);          // stop RX before TX (half-duplex)
        memcpy(uart_tx_buf, Buf, length);   // copy: USB buffer may be overwritten
        if (HAL_UART_Transmit_DMA(&huart1, uart_tx_buf, length) != HAL_OK) {
            tx_active = 0;                  // failsafe: re-open RX if TX failed
            HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rx_buf, RX_BUF_SIZE);
            __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
        }
    }
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, Buf);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    return USBD_OK;
}
```
Also add `#include "usbd_cdc_if.h"` to `main.c` and `extern UART_HandleTypeDef huart1;` to `usbd_cdc_if.c`.

---

## Build & Flash

1. Open the project in **STM32CubeIDE**, build (`Ctrl+B`).
2. Flash with **ST-Link** (or DFU via BOOT0).
3. Unplug ST-Link, connect the BlackPill USB to the PC.
4. Verify enumeration on Linux:
   ```bash
   ls /dev/ttyACM*
   lsusb | grep -i stm     # → STMicroelectronics Virtual COM Port (0483:5740)
   ```

---

## Usage

### DYNAMIXEL Wizard 2.0
1. Power on the 12 V servo supply.
2. Select port `ttyACM0`.
3. Scan with **1 Mbps, Protocol 2.0** (or scan all baud/protocol combos if unsure).
4. The servo appears and the full control table is readable/writable.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Port not in `/dev/ttyACM*` | USB clock not 48 MHz, or firmware not flashed | Check PLLQ → 48 MHz exact |
| `implicit declaration of CDC_Transmit_FS` | Missing include | `#include "usbd_cdc_if.h"` in `main.c` |
| Scan finds servo, but read times out | RX buffer too small / `FLUSH_DRREGISTER` dropping first byte | Use 1024 buffer; do **not** flush DR in TxCplt |
| Intermittent timeouts at 1 Mbps | Idle line ~4.5 V (5 V pull-up) | Move pull-up to 3.3 V |
| BlackPill dies when servo active | 5 V pull-up clamp-diode stress + no series resistor | 3.3 V pull-up + solid common GND |
| Deadlock / corruption under load | USB ISR preempting UART ISR | Set USB NVIC priority *below* UART/DMA |

---

## Lessons Learned

- **Single-Wire Half-Duplex needs no external direction-control IC** (no 74HC126 / MAX485). The STM32 handles TX/RX direction internally; PA9 goes high-Z when not transmitting.
- **MX-28T is TTL** ("T"), not RS-485 ("R") — no transceiver IC required.
- The biggest risks here are **electrical, not firmware**: a 5 V pull-up and a missing series resistor cost one BlackPill during development. Protect the data pin.
- Recovery/bootloader mode has much stricter timing than normal operation; a DIY bridge may handle normal comms perfectly yet struggle with firmware recovery.

---

