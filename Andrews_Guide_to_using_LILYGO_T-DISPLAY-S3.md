# Andrew's Guide to Using the LilyGO T-Display S3 v1.2

## Table of Contents
- [Overview](#overview)
- [Hardware Specifications](#hardware-specifications)
- [Power Input and Output](#power-input-and-output)
- [GPIO Pin Assignments](#gpio-pin-assignments)
- [Available GPIO Pins](#available-gpio-pins)
- [Critical Configuration Details](#critical-configuration-details)
- [Setting Up TFT_eSPI Library from Scratch](#setting-up-tft_espi-library-from-scratch)
- [Basic Sketch Template](#basic-sketch-template)
- [Additional Operations](#additional-operations)
- [Advanced Features](#advanced-features)
- [Troubleshooting](#troubleshooting)
- [References and Resources](#references-and-resources)

---

## Overview

The **LilyGO T-Display S3 v1.2** is an ESP32-S3 based development board featuring:
- ESP32-S3R8 chip (dual-core Xtensa LX7, 240 MHz, WiFi, Bluetooth LE)
- 16MB Flash memory
- 8MB PSRAM (OPI mode - Octal Parallel Interface)
- 1.9-inch ST7789 LCD display (170×320 pixels)
- **8-bit parallel interface** to the display (NOT SPI!)
- Built-in battery management (LiPo charging circuit)
- USB-C connector with native USB support
- Dual programmable buttons
- Expansion headers for external peripherals

### Key Differences from Other Versions

**CRITICAL:** The v1.2 board uses an **8-bit parallel interface** to communicate with the LCD, not SPI. This is fundamentally different from many other ESP32 display boards and requires specific configuration.

The "8bit LCD" designation refers to the **8-bit parallel data bus** (D0-D7 pins carrying data simultaneously), not the color depth of the display.

---

## Hardware Specifications

| Component | Specification |
|-----------|--------------|
| MCU | ESP32-S3R8 (Dual-core Xtensa LX7, 240 MHz) |
| Flash | 16MB |
| PSRAM | 8MB (Octal SPI) |
| Display | 1.9" ST7789 LCD |
| Resolution | 170 × 320 pixels |
| Interface | 8-bit parallel |
| Display Power | GPIO 15 (must be HIGH) |
| Backlight | GPIO 38 (PWM capable) |
| Battery | Built-in LiPo charging (500mA default) |
| USB | USB-C (native USB support) |
| Operating Voltage | 3.3V (logic), 5V (power input) |

---

## Power Input and Output

### Power Input Options

The board can be powered in two ways:

#### 1. USB-C Connector (Standard Method)
- **Voltage:** 5V via USB-C
- **Usage:** Programming, debugging, and general operation
- **Note:** When using external 5V pin power (option 2), disable USB CDC to prevent boot delays

#### 2. External 5V Pin (Recommended for Production)
- **Location:** Left side header, marked **5V** (red on pinout diagram)
- **Input Voltage:** 4.5V - 5.5V regulated
- **Maximum Current:** Depends on regulator capacity (typically 1-2A safe)
- **Usage:** When you want to power the board via pins instead of USB-C
- **Circuit:** Connect regulated 5V to the 5V pin, GND to any GND pin

**Example 5V Power Connection:**
```
External 5V Regulator → 5V pin (left header)
External GND         → GND pin (left header)
```

**IMPORTANT:** When using external 5V power:
1. Use a clean, regulated 5V supply
2. Connect GND between your power supply and the board
3. In Arduino IDE, set: **USB CDC On Boot: Disabled** (to avoid boot delays)

### Power Output Pins

The board provides two regulated power outputs for external devices:

#### 3.3V Output Pins
- **Location:** Right side header, marked **3V** (red on pinout diagram)
- **Count:** Two 3.3V pins available
- **Output Voltage:** 3.3V (regulated by onboard LDO)
- **Maximum Current:** ~500mA total (check your specific board's regulator)
- **Usage:** Power I2C devices, sensors, and other 3.3V peripherals

**Typical 3.3V Load Capacity:**
- I2C sensors (BME280, MPU6050, etc.): ~5-10mA each ✓
- Small OLED displays: ~20-50mA ✓
- Multiple I2C devices: Usually safe up to 200-300mA total ✓
- High-current devices (motors, servos): NOT recommended ✗

#### 5V Pass-Through
- **Location:** Left side header, marked **5V**
- **Function:** Can also be used as an OUTPUT when powered via USB-C
- **Current:** Limited by USB port capability (~500mA for USB 2.0)
- **Usage:** Power 5V devices when USB-C is the power source

### Ground Pins

Multiple GND connections are available for robust grounding:

| Location | Pin Label | Count | Notes |
|----------|-----------|-------|-------|
| Left Header | GND | 1 | Near 5V pin |
| Right Header | GND | 4 | Two pairs, well distributed |

**Best Practice:**
- Always connect GND when interfacing external devices
- Use the GND pin closest to your power connection
- For I2C devices, use GND from the same header as 3.3V

### Battery Power

| Feature | Specification |
|---------|--------------|
| Connector | JST 1.25mm 2-pin |
| Battery Type | Single-cell LiPo (3.7V nominal) |
| Charging Current | 500mA (default, adjustable) |
| Voltage Monitoring | GPIO 4 (ADC) |
| Protection | Overcharge, over-discharge, short circuit |

**Battery Voltage Reading:**
```cpp
float getBatteryVoltage() {
  uint32_t raw = analogRead(4);  // GPIO 4 = battery voltage divider
  // Battery voltage is divided by 2, ADC is 12-bit (0-4095), 3.3V reference
  return (raw * 2 * 3.3) / 4096.0;
}
```

---

## GPIO Pin Assignments

### Complete Pin Map

#### Left Side Header (from top to bottom)

| Pin # | GPIO | Function | Notes |
|-------|------|----------|-------|
| 1 | GPIO 1 | ADC1_CH0 | Available GPIO |
| 2 | GPIO 2 | ADC1_CH1 | Available GPIO |
| 3 | GPIO 3 | ADC1_CH2 | Available GPIO |
| 10 | GPIO 10 | SPI_CS / ADC1_CH9 | Available GPIO (SPI capable) |
| 11 | GPIO 11 | SPI_D / ADC2_CH0 | Available GPIO (SPI capable) |
| 12 | GPIO 12 | SPI_CLK / ADC2_CH1 | Available GPIO (SPI capable) |
| 13 | GPIO 13 | SPI_Q / ADC2_CH2 | Available GPIO (SPI capable) |
| - | NC | Not Connected | |
| - | NC | Not Connected | |
| - | **GND** | Ground | Power ground |
| - | **5V** | Power In/Out | 5V input or USB pass-through |

#### Right Side Header (from top to bottom)

| Pin # | GPIO | Function | Notes |
|-------|------|----------|-------|
| - | **3V** | 3.3V Output | Regulated 3.3V for peripherals |
| - | **GND** | Ground | Power ground |
| - | **GND** | Ground | Power ground |
| 43 | GPIO 43 | CLK_OUT1 | Available GPIO |
| 44 | GPIO 44 | CLK_OUT2 | Available GPIO |
| 18 | GPIO 18 | U1_RXD / I2C_SDA / ADC2_CH7 | **I2C Data (recommended)** |
| 17 | GPIO 17 | U1_TXD / I2C_SCL / ADC2_CH6 | **I2C Clock (recommended)** |
| 21 | GPIO 21 | - | Available GPIO |
| 16 | GPIO 16 | ADC2_CH5 | Available GPIO |
| - | NC | Not Connected | |
| - | **GND** | Ground | Power ground |
| - | **GND** | Ground | Power ground |
| - | **3V** | 3.3V Output | Regulated 3.3V for peripherals |

#### Display Interface Pins (Not on Headers - Internal)

| GPIO | Function | Description |
|------|----------|-------------|
| **5** | TFT_RST | Display reset (active LOW) |
| **6** | TFT_CS | Chip select (active LOW) |
| **7** | TFT_DC | Data/Command select |
| **8** | TFT_WR | Write strobe (parallel) |
| **9** | TFT_RD | Read strobe (parallel) |
| **39** | TFT_D0 | Data bit 0 |
| **40** | TFT_D1 | Data bit 1 |
| **41** | TFT_D2 | Data bit 2 |
| **42** | TFT_D3 | Data bit 3 |
| **45** | TFT_D4 | Data bit 4 |
| **46** | TFT_D5 | Data bit 5 |
| **47** | TFT_D6 | Data bit 6 |
| **48** | TFT_D7 | Data bit 7 |

#### Special Function Pins (Not on Headers - Internal)

| GPIO | Function | Description |
|------|----------|-------------|
| **15** | LCD Power | Display VDD power (MUST be HIGH before init) |
| **38** | LCD Backlight | PWM-capable backlight control |
| **0** | BOOT Button | Boot mode button (built-in) |
| **14** | User Button | General purpose button (IO14) |
| **4** | Battery ADC | Battery voltage monitoring (2:1 divider) |

---

## Available GPIO Pins

### Fully Available GPIOs

These pins are completely free for your use:

| GPIO | Location | Special Functions | Best For |
|------|----------|-------------------|----------|
| **1** | Left header pin 1 | ADC1_CH0 | Analog input, digital I/O |
| **2** | Left header pin 2 | ADC1_CH1 | Analog input, digital I/O |
| **3** | Left header pin 3 | ADC1_CH2 | Analog input, digital I/O |
| **10** | Left header pin 4 | SPI_CS, ADC1_CH9 | Digital I/O, SPI, analog input |
| **11** | Left header pin 5 | SPI_D, ADC2_CH0 | Digital I/O, SPI, analog input |
| **12** | Left header pin 6 | SPI_CLK, ADC2_CH1 | Digital I/O, SPI, analog input |
| **13** | Left header pin 7 | SPI_Q, ADC2_CH2 | Digital I/O, SPI, analog input |
| **16** | Right header | ADC2_CH5 | Digital I/O, analog input |
| **21** | Right header | - | Digital I/O |
| **43** | Right header | CLK_OUT1 | Digital I/O, UART |
| **44** | Right header | CLK_OUT2 | Digital I/O, UART |

### Recommended Pin Usage

| Use Case | Recommended GPIOs | Notes |
|----------|-------------------|-------|
| **I2C Bus** | GPIO 17 (SCL), GPIO 18 (SDA) | Pre-designated, convenient location |
| **General Digital I/O** | GPIO 1, 2, 3, 16, 21, 43, 44 | Freely available |
| **Analog Input** | GPIO 1, 2, 3, 10, 11, 12, 13, 16 | ADC1: 1,2,3,10  ADC2: 11,12,13,16 |
| **PWM Output** | Any available GPIO | All GPIOs support PWM (LEDC) |
| **SPI Devices** | GPIO 10-13 | Hardware SPI capable (MOSI, MISO, CLK, CS) |
| **Hardware UART** | Any available GPIO | ESP32-S3 has 3 UARTs mappable to any GPIO |

### UART (Serial Communication) Details

The ESP32-S3 has **3 hardware UART controllers** that can be mapped to almost any GPIO pin via the flexible GPIO matrix:

**UART Controllers:**
- **UART0:** Used for USB-CDC programming/debugging (not available for general use)
- **UART1:** Fully available, can be mapped to any GPIO pair
- **UART2:** Fully available, can be mapped to any GPIO pair

#### Default Serial Communication Path

**The `Serial` object (Serial.println(), etc.) uses:**

**Native USB-CDC** (when "USB CDC On Boot: Enabled" - recommended setting):
- ✓ Uses the **USB-C connector** directly via native USB hardware
- ✓ Does **NOT** use any UART controller
- ✓ Does **NOT** use any GPIO pins
- ✓ Provides direct USB communication to your computer
- ✓ **This is the default and recommended configuration**

**Example - Default Serial via USB:**
```cpp
void setup() {
  Serial.begin(115200);  // Uses USB-C connector (native USB)
  Serial.println("Hello from USB-CDC!");
  // No GPIO pins used - this is native USB communication
}
```

**Important Notes:**
- The ESP32-S3 has **native USB support** built into the chip
- When USB CDC is enabled, `Serial` communicates directly via USB hardware
- UART0, UART1, and UART2 remain fully available for other uses
- If you power via external 5V (not USB-C), you may want to disable USB CDC to avoid boot delays

#### Hardware UART for External Devices

For communicating with GPS modules, sensors, or other serial devices, use UART1 or UART2:

**Key Points:**
- ✓ **Hardware UART** - NOT software serial, full hardware support with FIFO buffers
- ✓ **Flexible mapping** - Can use ANY available GPIO pins via GPIO matrix
- ✓ **Full features** - Hardware flow control (RTS/CTS), parity, multiple stop bits
- ✓ **High speed** - Up to 5 Mbps with hardware support
- ✗ **Software serial not needed** - Hardware UART can use any pins

**Example - Hardware UART on Custom Pins:**
```cpp
// Use UART1 on GPIO 43 (TX) and GPIO 44 (RX)
#define UART1_TX 43
#define UART1_RX 44

HardwareSerial MySerial(1);  // Use UART1

void setup() {
  // Initialize UART1 on custom pins
  MySerial.begin(115200, SERIAL_8N1, UART1_RX, UART1_TX);
  MySerial.println("UART1 on GPIO 43/44");
}

void loop() {
  if (MySerial.available()) {
    char c = MySerial.read();
    MySerial.write(c);  // Echo
  }
}
```

**Recommended GPIO Pairs for UART:**
- **GPIO 43 (TX), 44 (RX)** - Right header, convenient pairing
- **GPIO 17 (TX), 18 (RX)** - Can be used if not using I2C
- **GPIO 1 (TX), 2 (RX)** - Left header, good for GPS or other serial devices

**Multiple Hardware UARTs Example:**
```cpp
// UART1 for GPS on GPIO 1/2
HardwareSerial GPS(1);
#define GPS_TX 1
#define GPS_RX 2

// UART2 for secondary device on GPIO 43/44
HardwareSerial Device(2);
#define DEV_TX 43
#define DEV_RX 44

void setup() {
  GPS.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Device.begin(115200, SERIAL_8N1, DEV_RX, DEV_TX);
}
```

### Reserved/Unavailable Pins

**DO NOT USE** these pins for general I/O (used by display and system):

- **GPIO 0:** Boot button (special boot mode pin)
- **GPIO 4:** Battery voltage ADC (in use)
- **GPIO 5-9, 15, 38-42, 45-48:** Display interface (critical)
- **GPIO 14:** User button (can be used but button will trigger)

---

## Critical Configuration Details

### 1. Interface Type: 8-bit Parallel

**This is the most important detail!** The v1.2 board does NOT use SPI. It uses an 8-bit parallel interface where:
- 8 GPIO pins (D0-D7) carry data simultaneously
- WR (write) and RD (read) pins strobe the data
- This is faster than SPI but uses more GPIO pins

### 2. Two-Stage Power Sequence

The display requires TWO separate power/control operations:

1. **GPIO 15 (Display Power)** - Powers the LCD panel itself
   - MUST be set HIGH before any display operations
   - Without this, the display won't respond at all
   - This powers the VDD rail of the LCD

2. **GPIO 38 (Backlight)** - Controls the LED backlight
   - Should be set HIGH AFTER display initialization
   - Can be PWM controlled for brightness (0-255)
   - Only affects brightness, not display function

### 3. Custom LCD Initialization Sequence

The v1.2's specific LCD panel requires custom ST7789 initialization commands beyond the standard TFT_eSPI initialization. This is handled by the `LCD_MODULE_CMD_1` configuration.

### 4. Display Orientation

The display is 170×320 pixels in portrait mode, but typically used in landscape:
- `setRotation(0)` - Portrait, USB-C at bottom, 170×320
- `setRotation(1)` - Landscape, USB-C on left, 320×170
- `setRotation(2)` - Portrait inverted, USB-C at top, 170×320
- `setRotation(3)` - Landscape inverted, USB-C on right, 320×170

### 5. RGB/BGR Color Channel Swap (v1.2 Hardware Issue)

**CRITICAL: The LilyGO T-Display S3 v1.2 with ST7789 panel has RED and BLUE color channels physically swapped.**

This is a hardware-level issue where the ST7789 controller's color channel mapping does not match the standard RGB565 format. When you request RED (0xF800), the display shows BLUE, and vice versa.

#### Confirmed Behavior:
- `TFT_RED` (0xF800) → Displays as **BLUE**
- `TFT_BLUE` (0x001F) → Displays as **RED**
- `TFT_NAVY` (0x000F, dark blue) → Displays as **DARK RED/MAROON**
- `TFT_MAROON` (0x7800, dark red) → Displays as **DARK BLUE/NAVY**
- `TFT_GREEN` (0x07E0) → Displays correctly (green channel unaffected)
- `TFT_WHITE` (0xFFFF) → Displays correctly (all channels)
- `TFT_BLACK` (0x0000) → Displays correctly (all channels off)

#### Root Cause:
The issue stems from the ST7789 MADCTL (Memory Access Control) register configuration in the custom initialization sequence. The v1.2 panel batch has the RGB/BGR bit set incorrectly, or the panel itself expects BGR format instead of RGB.

#### Workaround Method 1: Swap Color Constants (Recommended)

Instead of fixing the hardware initialization, swap the color values in your code:

```cpp
// Define color swap constants for v1.2 hardware
#define COLOR_RED       TFT_BLUE      // Request BLUE to display RED
#define COLOR_BLUE      TFT_RED       // Request RED to display BLUE
#define COLOR_NAVY      TFT_MAROON    // Request MAROON to display NAVY
#define COLOR_DARKBLUE  TFT_MAROON    // Request MAROON to display dark blue
#define COLOR_GREEN     TFT_GREEN     // Green works correctly
#define COLOR_WHITE     TFT_WHITE     // White works correctly
#define COLOR_BLACK     TFT_BLACK     // Black works correctly
#define COLOR_CYAN      TFT_CYAN      // Cyan works correctly
#define COLOR_MAGENTA   TFT_MAGENTA   // Magenta works correctly
#define COLOR_YELLOW    TFT_YELLOW    // Yellow works correctly

// Use swapped constants
tft.setTextColor(COLOR_RED, COLOR_BLACK);    // Displays as red
tft.drawString("Red Text", 10, 10);
```

#### Workaround Method 2: Modify MADCTL Register (Advanced)

Alternatively, you can try modifying the ST7789 MADCTL register bit 3 (RGB/BGR order) after initialization:

```cpp
// After tft.begin() and custom LCD init commands
tft.writecommand(0x36);  // MADCTL register
tft.writedata(0x00);     // Try different values: 0x00, 0x08, 0x48, 0x40
// Note: This may require experimentation and could affect rotation
```

**Warning:** Modifying MADCTL can affect display rotation and mirroring. The color swap workaround (Method 1) is safer and more reliable.

#### RGB565 Format Reference:

Standard RGB565 format (16-bit color):
```
Bits:  15 14 13 12 11 | 10 09 08 07 06 05 | 04 03 02 01 00
       R  R  R  R  R  | G  G  G  G  G  G  | B  B  B  B  B
```

The v1.2 panel interprets this as:
```
Bits:  15 14 13 12 11 | 10 09 08 07 06 05 | 04 03 02 01 00
       B  B  B  B  B  | G  G  G  G  G  G  | R  R  R  R  R  (swapped!)
```

#### Testing Colors:

Use this test sketch to verify color behavior:

```cpp
void testColors() {
  tft.fillScreen(TFT_BLACK);
  int yPos = 10;

  // Test RED channel
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.drawString("TFT_RED (should be BLUE on v1.2)", 10, yPos);
  yPos += 20;

  // Test BLUE channel
  tft.setTextColor(TFT_BLUE, TFT_BLACK);
  tft.drawString("TFT_BLUE (should be RED on v1.2)", 10, yPos);
  yPos += 20;

  // Test GREEN channel
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString("TFT_GREEN (should be GREEN)", 10, yPos);
  yPos += 20;

  // Draw color bars
  tft.fillRect(10, 80, 50, 30, TFT_RED);      // Should show BLUE
  tft.fillRect(70, 80, 50, 30, TFT_GREEN);    // Should show GREEN
  tft.fillRect(130, 80, 50, 30, TFT_BLUE);    // Should show RED
}
```

#### Summary:
- **Problem:** ST7789 v1.2 panel has swapped RED/BLUE channels
- **Impact:** Colors display incorrectly unless compensated
- **Solution:** Swap RED↔BLUE in your code (use TFT_BLUE for red, TFT_RED for blue)
- **Green Channel:** Unaffected, works correctly
- **Scope:** Only affects v1.2 batch; other versions may not have this issue

---

## Setting Up TFT_eSPI Library from Scratch

Follow these steps to configure a brand new TFT_eSPI library installation for the LilyGO T-Display S3 v1.2.

### Step 1: Install TFT_eSPI Library

**Arduino IDE (all versions):**
1. Open Arduino IDE
2. Go to **Sketch → Include Library → Manage Libraries**
3. Search for "TFT_eSPI"
4. Click Install (tested with v2.5.0 and later)

### Step 2: Locate TFT_eSPI Library Folder on macOS

The TFT_eSPI library can be in different locations depending on your Arduino setup:

#### Primary Location (Custom Arduino Directory - RECOMMENDED)
This is the custom library location being used for this project:
```
$HOME/ArduinoLocal/libraries/TFT_eSPI
```

For user "andrewward", the full path is:
```
/Users/andrewward/ArduinoLocal/libraries/TFT_eSPI
```

**This is the preferred location** as it keeps libraries separate from system-managed installations.

#### Secondary Location (macOS Default for Arduino IDE 2.x)
Arduino IDE 2.x typically uses the user's Documents folder:

```
$HOME/Documents/Arduino/libraries/TFT_eSPI
```

Or for macOS with iCloud Drive enabled:
```
$HOME/Library/Mobile Documents/com~apple~CloudDocs/Documents/Arduino/libraries/TFT_eSPI
```

Full path example:
```
/Users/andrewward/Documents/Arduino/libraries/TFT_eSPI
```

#### Tertiary Location (Arduino IDE System Libraries)
System-wide libraries managed by Arduino IDE:
```
$HOME/Library/Arduino15/libraries/TFT_eSPI
```

Full path example:
```
/Users/andrewward/Library/Arduino15/libraries/TFT_eSPI
```

**How to Verify Your Location:**
1. Open Arduino IDE
2. Go to **File → Preferences**
3. Note the "Sketchbook location" path
4. Your libraries are in `[Sketchbook location]/libraries/`

**For this guide, we use:**
```
/Users/andrewward/ArduinoLocal/libraries/TFT_eSPI
```

### Step 3: Configure Setup206_LilyGo_T_Display_S3.h

Navigate to the `User_Setups` folder and edit (or create) `Setup206_LilyGo_T_Display_S3.h`:

**File location:**
```
/Users/andrewward/ArduinoLocal/libraries/TFT_eSPI/User_Setups/Setup206_LilyGo_T_Display_S3.h
```

**Complete file contents:**

```cpp
// Setup for the ESP32 S3 LilyGo T-Display 170 x 320
// ST7789 using 8-bit PARALLEL interface (NOT SPI)

#define USER_SETUP_ID 206
#define ESP32_PARALLEL         // <-- CRITICAL: Use 8-bit parallel interface
#define INIT_SEQUENCE_3        // <-- CRITICAL: T-Display S3 specific init

#define ST7789_DRIVER          // ST7789 display controller

#define TFT_WIDTH  170
#define TFT_HEIGHT 320

#define CGRAM_OFFSET           // Library will add offsets required

#define TFT_RGB_ORDER TFT_BGR  // Colour order Blue-Green-Red
#define TFT_INVERSION_ON       // Display inversion ON

// *** LilyGo T-Display S3 v1.2 8-BIT PARALLEL PINS ***

// Control pins
#define TFT_CS    6            // Chip select
#define TFT_DC    7            // Data/Command
#define TFT_RST   5            // Reset pin
#define TFT_WR    8            // Write strobe
#define TFT_RD    9            // Read strobe

// 8-bit parallel data bus
#define TFT_D0   39
#define TFT_D1   40
#define TFT_D2   41
#define TFT_D3   42
#define TFT_D4   45
#define TFT_D5   46
#define TFT_D6   47
#define TFT_D7   48

// Backlight control
#define TFT_BL   38            // LED back-light (PWM capable)
#define TFT_BACKLIGHT_ON HIGH  // HIGH = backlight on

// CRITICAL: GPIO 15 is DISPLAY POWER - Must be set HIGH in sketch BEFORE init!

// Font loading
#define LOAD_GLCD              // Font 1. Original Adafruit 8 pixel font
#define LOAD_FONT2             // Font 2. Small 16 pixel high font
#define LOAD_FONT4             // Font 4. Medium 26 pixel high font
#define LOAD_FONT6             // Font 6. Large 48 pixel font
#define LOAD_FONT7             // Font 7. 7 segment 48 pixel font
#define LOAD_FONT8             // Font 8. Large 75 pixel font
#define LOAD_GFXFF             // FreeFonts

#define SMOOTH_FONT            // Enable anti-aliased fonts
```

### Step 4: Enable Setup206 in User_Setup_Select.h

Edit the main configuration file:

**File location:**
```
/Users/andrewward/ArduinoLocal/libraries/TFT_eSPI/User_Setup_Select.h
```

**Steps:**
1. Open `User_Setup_Select.h`
2. Find and **comment out** the default setup (around line 27):
   ```cpp
   //#include <User_Setup.h>           // Default setup - DISABLED
   ```

3. Find the line for Setup206 (around line 133) and **uncomment it**:
   ```cpp
   #include <User_Setups/Setup206_LilyGo_T_Display_S3.h>     // For the LilyGo T-Display S3
   ```

4. Make sure ALL other setup includes are commented out (only ONE should be active)

5. Save the file

### Step 5: Verify Configuration

Create a test sketch to verify the configuration compiled correctly:

```cpp
#include <TFT_eSPI.h>

TFT_eSPI tft = TFT_eSPI();

void setup() {
  Serial.begin(115200);

  // Check that the correct setup is loaded
  Serial.println("TFT_eSPI User Setup ID: " + String(USER_SETUP_ID));
  Serial.println("Expected: 206");

  #ifdef ESP32_PARALLEL
  Serial.println("✓ ESP32_PARALLEL defined - Correct!");
  #else
  Serial.println("✗ ESP32_PARALLEL NOT defined - Wrong setup!");
  #endif

  Serial.print("TFT Width: "); Serial.println(TFT_WIDTH);
  Serial.print("TFT Height: "); Serial.println(TFT_HEIGHT);
}

void loop() {}
```

**Expected Serial Output:**
```
TFT_eSPI User Setup ID: 206
Expected: 206
✓ ESP32_PARALLEL defined - Correct!
TFT Width: 170
TFT Height: 320
```

---

## Basic Sketch Template

Use this template as a starting point for all your LilyGO T-Display S3 v1.2 projects:

```cpp
#include <TFT_eSPI.h>

TFT_eSPI tft = TFT_eSPI();

// Pin definitions
#define PIN_POWER_ON 15  // Display power (VDD_LCD)
#define PIN_LCD_BL   38  // Backlight

// Custom LCD initialization for v1.2 panel
// The LCD_MODULE_CMD_1 macro enables a custom initialization sequence
// required for this specific LCD panel variant. Different batches of
// T-Display S3 boards may use slightly different ST7789 panels that
// require specific register settings for proper operation. This init
// sequence sets up timing, power, gamma, and other panel parameters.
#define LCD_MODULE_CMD_1

#if defined(LCD_MODULE_CMD_1)
typedef struct {
    uint8_t cmd;       // ST7789 register/command
    uint8_t data[14];  // Data bytes for the command
    uint8_t len;       // Number of data bytes (bit 7 = delay flag)
} lcd_cmd_t;

// ST7789 initialization command sequence for v1.2 LCD panel
lcd_cmd_t lcd_st7789v[] = {
    {0x11, {0}, 0 | 0x80},        // Sleep Out + 120ms delay
    {0x3A, {0X05}, 1},            // Interface Pixel Format: 16-bit/pixel
    {0xB2, {0X0B, 0X0B, 0X00, 0X33, 0X33}, 5},  // Porch Setting
    {0xB7, {0X75}, 1},            // Gate Control
    {0xBB, {0X28}, 1},            // VCOMS Setting
    {0xC0, {0X2C}, 1},            // LCM Control
    {0xC2, {0X01}, 1},            // VDV and VRH Command Enable
    {0xC3, {0X1F}, 1},            // VRH Set
    {0xC6, {0X13}, 1},            // FR Control 2
    {0xD0, {0XA7}, 1},            // Power Control 1
    {0xD0, {0XA4, 0XA1}, 2},      // Power Control 1 (alternate)
    {0xD6, {0XA1}, 1},            // Unknown command
    {0xE0, {0XF0, 0X05, 0X0A, 0X06, 0X06, 0X03, 0X2B, 0X32, 0X43, 0X36, 0X11, 0X10, 0X2B, 0X32}, 14},  // Positive Gamma
    {0xE1, {0XF0, 0X08, 0X0C, 0X0B, 0X09, 0X24, 0X2B, 0X22, 0X43, 0X38, 0X15, 0X16, 0X2F, 0X37}, 14},  // Negative Gamma
};
#endif

void setup() {
  // STEP 1: Turn on display power FIRST (critical!)
  pinMode(PIN_POWER_ON, OUTPUT);
  digitalWrite(PIN_POWER_ON, HIGH);
  delay(100);  // Allow power to stabilize

  // Initialize Serial for debugging
  Serial.begin(115200);
  Serial.println("LilyGO T-Display S3 v1.2 Starting...");

  // STEP 2: Initialize TFT
  tft.begin();
  Serial.println("TFT begin() complete");

  // STEP 3: Send custom LCD initialization commands
  #if defined(LCD_MODULE_CMD_1)
  for (uint8_t i = 0; i < (sizeof(lcd_st7789v) / sizeof(lcd_cmd_t)); i++) {
      tft.writecommand(lcd_st7789v[i].cmd);
      for (int j = 0; j < (lcd_st7789v[i].len & 0x7f); j++) {
          tft.writedata(lcd_st7789v[i].data[j]);
      }
      if (lcd_st7789v[i].len & 0x80) {
          delay(120);  // Delay if bit 7 is set
      }
  }
  Serial.println("Custom LCD init complete");
  #endif

  // STEP 4: Configure display orientation
  tft.setRotation(3);  // Landscape, USB on right (use 1 for USB on left)

  // STEP 5: Clear screen
  tft.fillScreen(TFT_BLACK);

  // STEP 6: Turn on backlight
  pinMode(PIN_LCD_BL, OUTPUT);
  digitalWrite(PIN_LCD_BL, HIGH);
  Serial.println("Backlight on");

  // STEP 7: Draw your content
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.drawString("Hello World!", 10, 10);

  Serial.println("Setup complete!");
}

void loop() {
  // Your code here
}
```

### Initialization Sequence Explanation

**Critical Order:**
1. **GPIO 15 HIGH** → Powers the LCD panel (without this, nothing works)
2. **delay(100)** → Let power stabilize
3. **tft.begin()** → Initialize TFT_eSPI and hardware
4. **Custom commands** → Send LCD-specific initialization (ST7789 register setup)
5. **Configure display** → Rotation, colors, etc.
6. **GPIO 38 HIGH** → Turn on backlight (do this AFTER init to avoid flicker)
7. **Draw content** → Now you can use all TFT_eSPI functions

---

## Additional Operations

### Display Rotation

The display can be used in 4 different orientations. Use `tft.setRotation()` to change orientation:

```cpp
// Function to demonstrate all rotation options
void demonstrateRotations() {
  for (int rotation = 0; rotation < 4; rotation++) {
    tft.setRotation(rotation);
    tft.fillScreen(TFT_BLACK);

    // Display rotation info
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(2);
    tft.setCursor(10, 10);
    tft.printf("Rotation: %d\n", rotation);
    tft.printf("Width: %d\n", tft.width());
    tft.printf("Height: %d\n", tft.height());

    // Draw corner indicators
    tft.fillCircle(5, 5, 5, TFT_RED);           // Top-left corner
    tft.fillCircle(tft.width()-5, 5, 5, TFT_GREEN);  // Top-right
    tft.fillCircle(5, tft.height()-5, 5, TFT_BLUE);  // Bottom-left
    tft.fillCircle(tft.width()-5, tft.height()-5, 5, TFT_YELLOW);  // Bottom-right

    delay(3000);
  }
}
```

**Rotation Reference:**

| Rotation | Orientation | USB-C Position | Width | Height | Use Case |
|----------|-------------|----------------|-------|--------|----------|
| **0** | Portrait | Bottom | 170 | 320 | Vertical displays, lists |
| **1** | Landscape | Left | 320 | 170 | Games, wide data displays |
| **2** | Portrait Inverted | Top | 170 | 320 | Upside-down mounting |
| **3** | Landscape Inverted | Right | 320 | 170 | **Most common** for handheld |

**Example Usage:**
```cpp
void setup() {
  // ... power and init code ...

  // Choose your orientation:
  tft.setRotation(3);  // Landscape, USB on right (recommended)

  // Now screen is 320 wide × 170 tall
  Serial.printf("Display: %d x %d\n", tft.width(), tft.height());
}
```

**Dynamic Rotation:**
```cpp
// Rotate display on button press
void loop() {
  static int currentRotation = 3;

  if (buttonPressed()) {
    currentRotation = (currentRotation + 1) % 4;  // Cycle 0-3
    tft.setRotation(currentRotation);
    redrawScreen();  // Your function to redraw content
  }
}
```

### Font Information and Sizes

TFT_eSPI provides multiple font options with different sizes and capabilities.

#### Built-in Fonts (Defined in Setup206)

These fonts are loaded by default in Setup206:

| Font Constant | Number | Height (px) | Characters | Best For | Memory (bytes) |
|---------------|--------|-------------|------------|----------|----------------|
| **GLCD** | 1 | 8 | Full ASCII | Small text, debug | ~1,820 |
| **FONT2** | 2 | 16 | 96 chars | Labels, UI elements | ~3,534 |
| **FONT4** | 4 | 26 | 96 chars | Headers, emphasis | ~5,848 |
| **FONT6** | 6 | 48 | Numbers + : - . apm | Large clocks, values | ~2,666 |
| **FONT7** | 7 | 48 | Seven-segment style | Digital clocks | ~2,438 |
| **FONT8** | 8 | 75 | Numbers + : - . | Very large displays | ~3,256 |
| **GFXFF** | - | Variable | Full Unicode subset | Professional text | Depends on font |

#### Using Built-in Fonts

```cpp
void demonstrateFonts() {
  tft.fillScreen(TFT_BLACK);
  int yPos = 5;

  // Font 1 (GLCD) - 8 pixel height
  tft.setTextFont(1);
  tft.setTextSize(1);  // Can be scaled 1x, 2x, 3x, etc.
  tft.setTextColor(TFT_WHITE);
  tft.drawString("Font 1: GLCD 8px", 5, yPos);
  yPos += 15;

  // Font 2 - 16 pixel height
  tft.setTextFont(2);
  tft.setTextSize(1);
  tft.setTextColor(TFT_GREEN);
  tft.drawString("Font 2: 16px", 5, yPos);
  yPos += 20;

  // Font 4 - 26 pixel height
  tft.setTextFont(4);
  tft.setTextSize(1);
  tft.setTextColor(TFT_CYAN);
  tft.drawString("Font 4: 26px", 5, yPos);
  yPos += 30;

  // Font 6 - Large numbers only
  tft.setTextFont(6);
  tft.setTextColor(TFT_YELLOW);
  tft.drawString("12:34.56", 5, yPos);
  yPos += 55;

  // Font 7 - Seven segment style
  tft.setTextFont(7);
  tft.setTextColor(TFT_RED);
  tft.drawString("88:88", 5, yPos);
}
```

#### Font Scaling with setTextSize()

All built-in fonts can be scaled:

```cpp
void demonstrateScaling() {
  tft.setTextFont(2);  // Use Font 2 (16px base)

  // Different scales
  tft.setTextSize(1);  // Normal (16px)
  tft.drawString("Size 1x", 10, 10);

  tft.setTextSize(2);  // Double (32px)
  tft.drawString("Size 2x", 10, 30);

  tft.setTextSize(3);  // Triple (48px)
  tft.drawString("Size 3x", 10, 70);

  // Note: Larger sizes use more memory and are slower
}
```

#### Free Fonts (GFXFF - TrueType)

Free fonts provide anti-aliased, scalable text:

```cpp
#include <TFT_eSPI.h>
// Include specific font files (from TFT_eSPI/Fonts/GFFF/ directory)
#include "Free_Fonts.h"  // Includes all Free Font definitions

void setup() {
  // ... display init ...

  // Use Free Sans 9pt
  tft.setFreeFont(&FreeSans9pt7b);
  tft.setTextColor(TFT_WHITE);
  tft.drawString("FreeSans 9pt", 10, 10);

  // Use Free Serif 12pt Bold
  tft.setFreeFont(&FreeSerifBold12pt7b);
  tft.drawString("Serif Bold 12pt", 10, 40);

  // Use Free Mono 18pt
  tft.setFreeFont(&FreeMono18pt7b);
  tft.drawString("Mono 18pt", 10, 80);
}
```

**Available Free Font Families:**
- **FreeSans** - Clean sans-serif (9, 12, 18, 24pt)
- **FreeSerif** - Traditional serif (9, 12, 18, 24pt)
- **FreeMono** - Monospace (9, 12, 18, 24pt)
- Each in **Regular**, **Bold**, **Italic**, **Bold Italic**

#### Practical Font Selection Guide

```cpp
// Display temperature reading
void showTemperature(float temp) {
  tft.setTextFont(7);        // Seven-segment for numbers
  tft.setTextSize(1);
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.drawString(String(temp, 1), 50, 50);

  tft.setTextFont(2);        // Smaller font for units
  tft.drawString("C", 180, 70);
}

// Display status message
void showStatus(String msg) {
  tft.setTextFont(4);        // Medium readable font
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString(msg, 10, 10);
}

// Display debug info
void debugPrint(String info) {
  tft.setTextFont(1);        // Tiny font to fit more
  tft.setTextSize(1);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.println(info);         // Uses println for auto line-wrap
}

// Display clock
void showClock(int hour, int min) {
  tft.setTextFont(8);        // Huge font for time
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  char timeStr[6];
  sprintf(timeStr, "%02d:%02d", hour, min);
  tft.drawString(timeStr, 80, 40);

  // AM/PM indicator with smaller font
  tft.setTextFont(2);
  tft.drawString(hour >= 12 ? "PM" : "AM", 270, 80);
}
```

#### Text Alignment and Positioning

```cpp
void demonstrateAlignment() {
  tft.setTextFont(4);

  // Set datum (reference point for text positioning)
  tft.setTextDatum(TL_DATUM);  // Top-Left (default)
  tft.drawString("Top Left", 0, 0);

  tft.setTextDatum(TC_DATUM);  // Top-Center
  tft.drawString("Top Center", tft.width()/2, 30);

  tft.setTextDatum(TR_DATUM);  // Top-Right
  tft.drawString("Top Right", tft.width(), 60);

  tft.setTextDatum(MC_DATUM);  // Middle-Center
  tft.drawString("CENTER", tft.width()/2, tft.height()/2);

  tft.setTextDatum(BR_DATUM);  // Bottom-Right
  tft.drawString("Bottom Right", tft.width(), tft.height());

  // Reset to default
  tft.setTextDatum(TL_DATUM);
}
```

**Available Text Datums:**
- `TL_DATUM` - Top Left
- `TC_DATUM` - Top Center
- `TR_DATUM` - Top Right
- `ML_DATUM` - Middle Left
- `MC_DATUM` - Middle Center (useful for centering)
- `MR_DATUM` - Middle Right
- `BL_DATUM` - Bottom Left
- `BC_DATUM` - Bottom Center
- `BR_DATUM` - Bottom Right

---

## Advanced Features

### PWM Backlight Control

Control backlight brightness with PWM (0-255):

```cpp
// Setup PWM for backlight (ESP-IDF 5.x / Arduino ESP32 3.x)
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,0,0)
  ledcAttach(PIN_LCD_BL, 2000, 8);  // 2kHz, 8-bit resolution
  ledcWrite(PIN_LCD_BL, 128);       // 50% brightness
#else
  // ESP-IDF 4.x / Arduino ESP32 2.x
  ledcSetup(0, 2000, 8);            // Channel 0, 2kHz, 8-bit
  ledcAttachPin(PIN_LCD_BL, 0);     // Attach to channel 0
  ledcWrite(0, 128);                // 50% brightness
#endif
```

**Brightness values:**
- `0` = Backlight off
- `128` = 50% brightness
- `255` = 100% brightness (maximum)

**Smooth Fade Effect:**
```cpp
void fadeBrightness(int fromLevel, int toLevel, int stepDelay) {
  if (fromLevel < toLevel) {
    for (int i = fromLevel; i <= toLevel; i++) {
      ledcWrite(PIN_LCD_BL, i);
      delay(stepDelay);
    }
  } else {
    for (int i = fromLevel; i >= toLevel; i--) {
      ledcWrite(PIN_LCD_BL, i);
      delay(stepDelay);
    }
  }
}

// Usage: Fade from off to full brightness over 1 second
fadeBrightness(0, 255, 4);  // 255 steps * 4ms = ~1 second
```

### Battery Voltage Reading

Read battery voltage on GPIO 4:

```cpp
float getBatteryVoltage() {
  uint32_t raw = analogRead(4);  // GPIO 4 has 2:1 voltage divider
  // Battery voltage is divided by 2, ADC is 12-bit (0-4095) with 3.3V reference
  float voltage = (raw * 2 * 3.3) / 4096.0;
  return voltage;
}

void setup() {
  pinMode(4, INPUT);
  // ... rest of setup
}

void loop() {
  float battVolt = getBatteryVoltage();
  Serial.printf("Battery: %.2f V\n", battVolt);

  // Display battery status
  tft.setTextFont(2);
  if (battVolt > 4.1) {
    tft.setTextColor(TFT_GREEN);
    tft.drawString("Battery: Full", 10, 10);
  } else if (battVolt > 3.7) {
    tft.setTextColor(TFT_YELLOW);
    tft.drawString("Battery: Good", 10, 10);
  } else if (battVolt > 3.4) {
    tft.setTextColor(TFT_ORANGE);
    tft.drawString("Battery: Low", 10, 10);
  } else {
    tft.setTextColor(TFT_RED);
    tft.drawString("Battery: Critical", 10, 10);
  }

  delay(5000);
}
```

**Battery Voltage Reference:**
- **4.2V** - Fully charged
- **3.7V** - Nominal voltage
- **3.4V** - Low (charge soon)
- **3.0V** - Critical (shutting down)

### Deep Sleep with Wake on Button

```cpp
void goToSleep() {
  // Turn off display
  digitalWrite(PIN_LCD_BL, LOW);
  digitalWrite(PIN_POWER_ON, LOW);

  // Wake on button 2 press (GPIO 14)
  esp_sleep_enable_ext0_wakeup((gpio_num_t)14, 0);  // Wake on LOW

  Serial.println("Going to sleep...");
  delay(100);
  esp_deep_sleep_start();
}

void loop() {
  // Go to sleep on button press
  if (digitalRead(14) == LOW) {
    delay(50);  // Debounce
    if (digitalRead(14) == LOW) {
      goToSleep();
    }
  }
}
```

### Using I2C Devices

The board has dedicated I2C pins (GPIO 17 = SCL, GPIO 18 = SDA) that are easily accessible on the right header.

#### I2C Bus Configuration

**Standard I2C Setup:**
```cpp
#include <Wire.h>

#define I2C_SDA 18
#define I2C_SCL 17

void setup() {
  // Initialize I2C bus with 100kHz (standard mode)
  Wire.begin(I2C_SDA, I2C_SCL, 100000);

  // Or use default 100kHz:
  Wire.begin(I2C_SDA, I2C_SCL);

  // For fast mode (400kHz):
  Wire.begin(I2C_SDA, I2C_SCL, 400000);
}
```

#### Pull-up Resistors

**The ESP32-S3 has internal pull-ups (~45kΩ), but external pull-ups are recommended for reliability:**

| Bus Speed | Recommended Pull-up | Required? | Notes |
|-----------|---------------------|-----------|-------|
| 100kHz (Standard) | 4.7kΩ - 10kΩ | Optional | Internal pull-ups often sufficient |
| 400kHz (Fast) | 2.2kΩ - 4.7kΩ | **Recommended** | External pull-ups improve signal integrity |
| 1MHz (Fast+) | 1kΩ - 2.2kΩ | **Required** | Must use external pull-ups |

**Pull-up Resistor Wiring:**
```
         3.3V (from board header)
           |
           ├── 4.7kΩ resistor ── SDA (GPIO 18)
           |
           └── 4.7kΩ resistor ── SCL (GPIO 17)
```

**When to add external pull-ups:**
- ✓ Using multiple I2C devices (>2 devices)
- ✓ Long wire runs (>15cm / 6 inches)
- ✓ Fast mode (400kHz) or faster
- ✓ Unreliable communication with internal pull-ups
- ✗ Single sensor with short wires usually works with internal pull-ups

**Enabling Internal Pull-ups (if not using external):**
```cpp
void setup() {
  pinMode(I2C_SDA, INPUT_PULLUP);
  pinMode(I2C_SCL, INPUT_PULLUP);
  Wire.begin(I2C_SDA, I2C_SCL);
}
```

#### Bus Termination

I2C does **NOT require termination resistors** (unlike CAN bus or RS-485). The pull-up resistors ARE the termination.

**Common Mistakes to Avoid:**
- ✗ Don't use multiple sets of pull-ups (check if your sensor breakout already has them)
- ✗ Don't use pull-ups that are too strong (<1kΩ) - wastes power
- ✗ Don't use pull-ups that are too weak (>10kΩ at 400kHz) - unreliable
- ✓ One set of pull-ups per I2C bus is sufficient, even with multiple devices

#### Powering I2C Devices from the Board

**Typical Setup:**
```
T-Display S3          I2C Sensor
--------------       --------------
3V (3.3V) ────────── VCC/VDD
GND ──────────────── GND
GPIO 18 ──────────── SDA
GPIO 17 ──────────── SCL
```

**Power Budget:**
- Available 3.3V current: ~500mA total (shared with ESP32 and display)
- Typical I2C sensor: 1-10mA (very low power)
- **Safe total load: <200mA for external devices** (leaves margin for ESP32)

**Multiple I2C Devices:**
```
T-Display S3          Sensor 1          Sensor 2          Sensor 3
--------------       -----------       -----------       -----------
3.3V ────────┬────── VCC               VCC               VCC
             │   ┌── 4.7kΩ to 3.3V
GPIO 18 ─────┼───┴── SDA ──────────── SDA ──────────── SDA
             │   ┌── 4.7kΩ to 3.3V
GPIO 17 ─────┼───┴── SCL ──────────── SCL ──────────── SCL
GND ─────────┴────── GND ──────────── GND ──────────── GND

Note: Pull-up resistors only needed once per bus (not per device)
```

#### Scanning for I2C Devices

```cpp
void scanI2C() {
  Serial.println("Scanning I2C bus...");
  int deviceCount = 0;

  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    byte error = Wire.endTransmission();

    if (error == 0) {
      Serial.printf("Device found at 0x%02X\n", addr);
      deviceCount++;
    }
  }

  if (deviceCount == 0) {
    Serial.println("No I2C devices found");
  } else {
    Serial.printf("Found %d device(s)\n", deviceCount);
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(1000);
  scanI2C();
}
```

#### Example: Reading BME280 Sensor

```cpp
#include <Wire.h>
#include <Adafruit_BME280.h>

#define I2C_SDA 18
#define I2C_SCL 17

Adafruit_BME280 bme;

void setup() {
  Serial.begin(115200);

  // Initialize I2C
  Wire.begin(I2C_SDA, I2C_SCL);

  // Initialize BME280 sensor
  if (!bme.begin(0x76)) {  // BME280 address (try 0x77 if this fails)
    Serial.println("BME280 sensor not found!");
    while (1);
  }

  Serial.println("BME280 initialized");
}

void loop() {
  float temp = bme.readTemperature();
  float pressure = bme.readPressure() / 100.0F;
  float humidity = bme.readHumidity();

  // Display on screen
  tft.fillScreen(TFT_BLACK);
  tft.setTextFont(4);
  tft.setTextColor(TFT_WHITE);

  tft.drawString("Temperature:", 10, 20);
  tft.drawString(String(temp, 1) + " C", 10, 50);

  tft.drawString("Humidity:", 10, 90);
  tft.drawString(String(humidity, 1) + " %", 10, 120);

  delay(2000);
}
```

**Common I2C Device Addresses:**
| Device | Default Address(es) | Alt Address |
|--------|---------------------|-------------|
| BME280 | 0x76 | 0x77 |
| BME680 | 0x76 | 0x77 |
| BMP280 | 0x76 | 0x77 |
| MPU6050 | 0x68 | 0x69 |
| ADS1115 | 0x48 | 0x49, 0x4A, 0x4B |
| SSD1306 OLED | 0x3C | 0x3D |
| PCF8574 | 0x20-0x27 | 0x38-0x3F |

---

## Troubleshooting

### Problem: Display stays black, backlight is on

**Symptoms:**
- Backlight turns on
- Serial monitor shows "Setup complete"
- Screen remains completely black

**Causes:**
1. Wrong TFT_eSPI configuration (SPI instead of parallel)
2. Custom LCD init commands not sent
3. GPIO 15 not set HIGH before init

**Solution:**
1. Verify Setup206 is configured for `ESP32_PARALLEL` (not SPI)
2. Verify `User_Setup_Select.h` includes Setup206
3. Ensure `LCD_MODULE_CMD_1` is defined and commands are sent
4. Verify GPIO 15 is set HIGH before `tft.begin()`

### Problem: ESP32 crashes/reboots during display init

**Symptoms:**
- Serial shows initialization starting
- ESP32 resets before "Setup complete"
- Brownout detector triggered

**Causes:**
1. GPIO 15 not powered on
2. Insufficient USB power
3. Display init timing issues

**Solution:**
1. Ensure GPIO 15 is HIGH before any TFT operations
2. Use good quality USB cable
3. Add longer delay after GPIO 15 HIGH (try 200ms)
4. Power from external 5V supply if using USB hub

### Problem: Garbled display or wrong colors

**Symptoms:**
- Display shows content but colors are wrong (RED appears as BLUE, BLUE appears as RED)
- Text is readable but shifted/garbled
- Random pixels or noise

**Causes:**
1. **RGB/BGR color channel swap (MOST COMMON for v1.2)**
2. Wrong byte swapping
3. Incorrect data bus pin mapping

**Solution:**

**For v1.2 Color Swap Issue (RED↔BLUE):**

The LilyGO T-Display S3 v1.2 has a **known hardware issue** where RED and BLUE channels are swapped. This is NORMAL for v1.2 and requires a workaround in your code.

**See:** [Critical Configuration Details - Section 5: RGB/BGR Color Channel Swap](#5-rgbbgr-color-channel-swap-v12-hardware-issue)

**Quick Fix:** Swap color constants in your code:
```cpp
// Use TFT_BLUE to display RED
tft.setTextColor(TFT_BLUE, TFT_BLACK);  // Displays as RED
tft.drawString("Red Text", 10, 10);

// Use TFT_MAROON to display NAVY/dark blue
tft.setTextColor(TFT_MAROON, TFT_BLACK);  // Displays as NAVY
tft.drawString("Navy Text", 10, 30);
```

**For Other Color Issues:**

1. Try changing `TFT_RGB_ORDER TFT_BGR` to `TFT_RGB_ORDER TFT_RGB` in Setup206
2. Add `tft.setSwapBytes(true);` after `tft.begin()`
3. Verify all D0-D7 pins match exactly (GPIO 39-42, 45-48)

**Note:** If RED displays as BLUE and BLUE displays as RED, this is the v1.2 color swap issue and is expected. Use the color constant workaround above.

### Problem: Display works but text/graphics are rotated wrong

**Symptoms:**
- Everything displays but orientation is incorrect
- USB port is in wrong position

**Solution:**
Try different rotation values:
```cpp
tft.setRotation(0);  // Portrait, USB at bottom
tft.setRotation(1);  // Landscape, USB on left
tft.setRotation(2);  // Portrait, USB at top
tft.setRotation(3);  // Landscape, USB on right
```

### Problem: Compilation errors about undefined pins

**Symptoms:**
```
error: 'TFT_D0' was not declared in this scope
error: 'TFT_WR' was not declared in this scope
```

**Causes:**
- Wrong User_Setup file selected
- Setup206 not properly configured

**Solution:**
1. Open `User_Setup_Select.h`
2. Verify ONLY Setup206 is uncommented
3. Verify Setup206 has all parallel pins defined (D0-D7, WR, RD)
4. Clean and rebuild project

### Problem: Arduino IDE can't find board

**Symptoms:**
- Board not showing in Arduino IDE
- Upload fails

**Solution:**
1. Install ESP32 board support:
   - Go to File → Preferences
   - Add URL: `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
   - Tools → Board → Boards Manager → Search "ESP32" → Install
2. Select board: **ESP32S3 Dev Module**
3. Configure settings:
   - USB CDC On Boot: "Enabled"
   - USB Mode: "Hardware CDC and JTAG"
   - PSRAM: "OPI PSRAM"
   - Flash Size: "16MB (128Mb)"
   - Partition Scheme: "16M Flash (3MB APP/9.9MB FATFS)"

### Problem: I2C devices not responding

**Symptoms:**
- I2C scan finds no devices
- Sensor initialization fails

**Solution:**
1. Check wiring: SDA=18, SCL=17, power and GND
2. Run I2C scanner (see Advanced Features section)
3. Add external 4.7kΩ pull-up resistors if not present
4. Try slower I2C speed: `Wire.begin(I2C_SDA, I2C_SCL, 100000);`
5. Verify device address (some sensors have alternate addresses)

---

## References and Resources

### Official LilyGO Resources

**GitHub Repository:**
- https://github.com/Xinyuan-LilyGO/T-Display-S3
- Contains examples, schematics, and documentation
- See `/examples/tft/tft.ino` for working TFT_eSPI example
- See `/examples/factory/pin_config.h` for complete pin definitions

**Product Page:**
- https://www.lilygo.cc/products/t-display-s3
- Specifications and purchasing information

**Pinout Diagram:**
- https://github.com/Xinyuan-LilyGO/T-Display-S3/blob/main/image/T-DISPLAY-S3.jpg
- Complete visual pin reference

**Schematic:**
- https://github.com/Xinyuan-LilyGO/T-Display-S3/blob/main/schematic/T_Display_S3.pdf
- Official circuit schematic

### TFT_eSPI Library

**Main Repository:**
- https://github.com/Bodmer/TFT_eSPI
- Comprehensive graphics library for ESP32

**Documentation:**
- https://github.com/Bodmer/TFT_eSPI/blob/master/User_Setup_Select.h
- Configuration examples and setup guides

**Font Documentation:**
- https://github.com/Bodmer/TFT_eSPI/tree/master/Fonts/GFXFF
- Free font files and usage examples

### Hardware Documentation

**ESP32-S3 Datasheet:**
- https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf
- Complete ESP32-S3 specifications

**ESP32-S3 Technical Reference:**
- https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf
- Detailed peripheral documentation

**ST7789 Display Controller:**
- https://www.displayfuture.com/Display/datasheet/controller/ST7789.pdf
- LCD controller chip documentation

### Specific Discovery Details

**Key Resources for v1.2 Configuration:**

1. **Official TFT_eSPI Example:**
   - File: `T-Display-S3/examples/tft/tft.ino`
   - Location: https://github.com/Xinyuan-LilyGO/T-Display-S3/blob/main/examples/tft/tft.ino
   - Shows: Custom LCD init sequence and parallel pin usage

2. **Pin Configuration Reference:**
   - File: `T-Display-S3/examples/factory/pin_config.h`
   - Location: https://github.com/Xinyuan-LilyGO/T-Display-S3/blob/main/examples/factory/pin_config.h
   - Shows: Complete GPIO mapping for v1.2

3. **Setup206 Configuration:**
   - File: `TFT_eSPI/User_Setups/Setup206_LilyGo_T_Display_S3.h`
   - Original (incorrect): Configured for SPI interface
   - Corrected: 8-bit parallel interface as documented in this guide

4. **Critical Forum/Issue Discussions:**
   - External 5V Power: https://github.com/Xinyuan-LilyGO/T-Display-S3/issues/205
   - LCD Module Variants: Documented in factory.ino example (LCD_MODULE_CMD_1 macro)
   - TFT_eSPI Compatibility: https://github.com/Bodmer/TFT_eSPI/issues/3329

### Community Resources

**Arduino ESP32 Forum:**
- https://esp32.com/
- Community support for ESP32 projects

**LilyGO Community:**
- https://github.com/Xinyuan-LilyGO/T-Display-S3/issues
- Specific issues and solutions for T-Display S3

### Arduino IDE Resources

**Arduino ESP32 Documentation:**
- https://docs.espressif.com/projects/arduino-esp32/en/latest/
- Official ESP32 Arduino core documentation

**Arduino IDE Download:**
- https://www.arduino.cc/en/software
- Latest Arduino IDE (2.3.0 or later recommended)

---

## Version History

**v1.0 - 2025-11-06**
- Initial guide created based on successful configuration of LilyGO T-Display S3 v1.2
- Documented 8-bit parallel interface discovery
- Complete pin mappings verified
- Working template and initialization sequence confirmed

**v1.1 - 2025-11-06**
- Removed Windows/Linux references, macOS only
- Added power input/output section with detailed 5V/3.3V specifications
- Added complete available GPIO pins section
- Added I2C configuration with pull-up resistor guidance
- Added display rotation examples and comprehensive font documentation
- Enhanced with text alignment and datum examples
- Added all ground pin locations
- Removed License section
- Updated library path priorities for custom ArduinoLocal directory
- Added explanatory comments for LCD_MODULE_CMD_1 initialization

**v1.2 - 2025-11-06**
- Removed all SD shield and touch shield references
- Moved GPIOs 10, 11, 12, 13, 16, 21 to fully available (no longer conditional)
- Eliminated "Conditionally Available GPIOs" section
- Updated pin tables to show all GPIOs as available for general use
- Updated recommended pin usage to include all available analog input channels
- Added comprehensive UART section explaining ESP32-S3 hardware UART capabilities
- Clarified that ESP32-S3 has 3 hardware UARTs (not software serial)
- Added UART mapping examples showing flexible GPIO matrix usage
- Documented multiple hardware UART usage examples
- Clarified Serial.println() path: uses native USB-CDC via USB-C (not UART, no GPIO)
- Explained difference between Serial object (USB) and HardwareSerial (UART) objects

**v1.3 - 2025-11-06**
- Added Critical Configuration Details Section 5: RGB/BGR Color Channel Swap (v1.2 Hardware Issue)
- Documented confirmed RED↔BLUE channel swap in ST7789 v1.2 panel
- Added detailed color swap behavior table (TFT_RED→BLUE, TFT_BLUE→RED, etc.)
- Included two workaround methods: color constant swapping (recommended) and MADCTL modification
- Added RGB565 format reference showing bit-level channel swap
- Included color testing sketch to verify swap behavior
- Updated Troubleshooting section "Garbled display or wrong colors" with v1.2 color swap fix
- Cross-referenced Critical Configuration Details from Troubleshooting section

---

**Document created by:** Andrew Ward
**Date:** November 6, 2025
**Hardware:** LilyGO T-Display S3 ESP32-S3R8 v1.2 with 1.9" 8-bit LCD
**Working Configuration:** TFT_eSPI with 8-bit parallel interface
