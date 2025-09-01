# EE 329 – Lab A9: I²C EEPROM Interface (STM32L4)

This project demonstrates **direct-register I²C communication** between the STM32L4A6ZG Nucleo board and a **24LC256 EEPROM**.  
The program writes a test byte to EEPROM memory, reads it back, and uses an on-board LED as a pass/fail indicator.  
It also includes a **SysTick-based microsecond delay driver** to satisfy EEPROM timing constraints.

---

## 🎯 Project Overview
- **EEPROM Device:** 24LC256 (I²C, 7-bit address = 0x57)  
- **I²C Bus:** Configured on PB8 (SCL) and PB9 (SDA) using I2C1 peripheral  
- **Write/Read Test:**  
  - Write a byte (0xDA) to EEPROM at address 0x6084  
  - Read back the byte  
  - If data matches, PB14 LED is lit  
- **Delay Utility:** SysTick microsecond delay for EEPROM internal write cycle timing  
- **Direct Register Access:** GPIO and I²C configured without HAL drivers for precise control  

---

## 🧩 Source Files
- `main.c / main.h` – Application entry, LED control, test loop【179†source】【180†source】  
- `I2C.c / I2C.h` – EEPROM I²C driver (init, byte read/write)【177†source】【178†source】  
- `delay.c / delay.h` – Microsecond delay implementation using SysTick【175†source】【176†source】  

---

## ⚡ Hardware Setup
- **Board:** STM32L4A6ZG Nucleo  
- **Connections:**  
  - PB8 → I²C1_SCL → EEPROM SCL  
  - PB9 → I²C1_SDA → EEPROM SDA  
  - PB14 → LED indicator  
  - VCC/GND → EEPROM power pins  
- **EEPROM:** 24LC256 or compatible I²C EEPROM (address 0x57)  

---

## 🚀 How to Run
1. Import the sources into **STM32CubeIDE**.  
2. Build and flash to the **NUCLEO-L4A6ZG** board.  
3. Connect EEPROM to PB8/PB9 and power rails.  
4. Observe PB14 LED:  
   - **ON** → EEPROM write/read passed  
   - **OFF** → EEPROM mismatch or no response  

---

## 📂 Repository Structure
```
.
├── main.c / main.h          # Application logic and LED control
├── I2C.c / I2C.h            # EEPROM driver with direct I²C register access
├── delay.c / delay.h        # SysTick microsecond delay
├── .gitignore               # Ignore build artifacts
└── README.md                # Project documentation
```

---

## ✅ Learning Outcomes
- Implementing **I²C communication** with external EEPROM devices  
- Using **direct register access** for peripheral configuration  
- Enforcing **EEPROM timing requirements** with SysTick delays  
- Verifying memory operations with hardware indicators (LEDs)  
- Structuring a professional embedded project for GitHub  

---

## 📜 License
This project is licensed under the MIT License – see [LICENSE](LICENSE).

---

👤 **Author:** Nathan Heil  
📅 **Course:** EE‑329 (Embedded Systems)  
