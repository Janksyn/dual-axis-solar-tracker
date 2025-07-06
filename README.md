# Arduino Solar Tracker

This project implements an automatic solar tracking system using an Arduino microcontroller. It calculates the sun's position based on geographical coordinates and current time, then adjusts the angle of a solar panel to maximize solar energy capture. The system also includes power-saving features to reduce energy consumption when the panel is stationary or at night.

---

## Features

* **Automatic Sun Tracking:** Calculates solar elevation and azimuth in real-time.
* **Dual-Axis Control:** Uses two DC motors for precise movement along X (horizontal/azimuth) and Y (vertical/elevation) axes.
* **RTC Integration:** Employs a DS1307 Real-Time Clock module for accurate timekeeping, independent of the Arduino's power cycle.
* **Calibration:** Automatically calibrates motor positions to known end-stops during startup.
* **Power Management:** Utilizes the `LowPower` library to put the Arduino into sleep mode, significantly reducing power consumption when the panel doesn't need to move or during nighttime.
* **Debug Output:** Provides serial output for tracking solar position, motor movements, and system status.

---

## Hardware Requirements

* Arduino Uno (or compatible ATmega328P-based board)
* RTC Module (DS1307 recommended, compatible with `RTClib` library)
* Two DC Motors
* Two Motor Drivers
* Solar Panel
* Breadboard and Jumper Wires
* Power Supply for motors (appropriate for your chosen motors)

---

## Software Requirements

* Arduino IDE
* **Libraries:**
    * `Wire.h` (usually built-in)
    * `TimeLib.h` (Install via Arduino IDE Library Manager)
    * `SolarPosition.h` (Install via Arduino IDE Library Manager)
    * `RTClib.h` (Install via Arduino IDE Library Manager)
    * `LowPower.h` (Install via Arduino IDE Library Manager)

---

## Installation and Setup

### 1. Wiring

Connect the components as follows:

* **RTC Module (DS1307):**
    * SDA to Arduino A4
    * SCL to Arduino A5
    * VCC to Arduino 5V
    * GND to Arduino GND

* **Motor X (Azimuth):**
    * `IN1_X` (Motor Driver Input 1) to Arduino Pin 10
    * `IN2_X` (Motor Driver Input 2) to Arduino Pin 9
    * `EN_X` (Motor Driver Enable) to Arduino Pin 11
    * Motor power supply connected to motor driver (refer to your specific driver's datasheet)

* **Motor Y (Elevation):**
    * `IN1_Y` (Motor Driver Input 1) to Arduino Pin 5
    * `IN2_Y` (Motor Driver Input 2) to Arduino Pin 6
    * `EN_Y` (Motor Driver Enable) to Arduino Pin 2
    * Motor power supply connected to motor driver

* **Global Enable Pin (`en`):**
    * `en` to Arduino Pin A0 (This pin is used to enable/disable the motor drivers' power for power saving).

### 2. Arduino IDE Setup

1.  Open your Arduino IDE.
2.  Go to **Sketch > Include Library > Manage Libraries...**
3.  Search for and install the following libraries:
    * `TimeLib` by Michael Margolis
    * `SolarPosition` by mikalhart
    * `RTClib` by Adafruit
    * `LowPower` by rocketscream
4.  Open the provided `.ino` sketch in the Arduino IDE.

### 3. Configuration

Adjust the following constants in the code to match your setup and location:

* **`latitude` and `longitude`:** Set these to your exact geographical coordinates.
    ```cpp
    double latitude = -23.3045; // Londrina, Brazil
    double longitude = -51.1696; // Londrina, Brazil
    ```
* **`UTC_OFFSET`:** Your local UTC offset in hours.
    ```cpp
    const int UTC_OFFSET = -3; // For Brazil, GMT-3
    ```
* **`TOTAL_STEPS_X` and `TOTAL_STEPS_Y`:** These represent the total number of steps your motors need to travel from one end-stop to the other for their respective axes. You will need to determine these values experimentally for your specific motor and mechanical setup.
    ```cpp
    #define TOTAL_STEPS_X 1200 // Example value, adjust as needed
    #define TOTAL_STEPS_Y 440  // Example value, adjust as needed
    ```
* **`MOTOR_DELAY_X`, `MOTOR_DELAY_Y`:** The delay between steps for each motor. Adjust for smoother operation.
* **`VELOCIDADE_MOTOR_X`, `VELOCIDADE_MOTOR_Y`:** PWM values (0-255) for motor speed.
* **`ANGULO_PAINEL_MIN`, `ANGULO_PAINEL_MAX`:** These define the range of angles your Y-axis motor can achieve. You'll need to measure or estimate the actual angular range of your panel's elevation movement and map it to your motor's steps.
    ```cpp
    #define ANGULO_PAINEL_MIN 105.0 // Example, adjust based on your mechanical setup
    #define ANGULO_PAINEL_MAX 180.0 // Example, adjust based on your mechanical setup
    ```

### 4. Upload Code

1.  Connect your Arduino board to your computer.
2.  Select the correct board and port in the Arduino IDE (**Tools > Board**, **Tools > Port**).
3.  Upload the sketch to your Arduino.

---

## How it Works

### Timekeeping

The system uses an RTC module to maintain accurate time. In the `setup()` function, it checks if the RTC is running. If not, it attempts to set the time using the compile time (`F(__DATE__), F(__TIME__)`). It's recommended to **manually set the RTC time once** for initial accuracy after uploading the code, for example, by uncommenting `rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));` and re-uploading, then commenting it out again.

### Solar Position Calculation

In the `loop()` function:

1.  The current time is retrieved from the RTC.
2.  The local time is converted to UTC using the `UTC_OFFSET`.
3.  The `calculateSolarPosition()` function from the `SolarPosition.h` library determines the sun's `elevation` (angle above the horizon) and `azimuth` (horizontal direction from North).

### Motor Control and Tracking

1.  **Elevation Check:** If the `elevation` is greater than 2 degrees (meaning the sun is above the horizon), the system calculates the `posicaoDesejadaX` (desired X-axis steps) and `posicaoDesejadaY` (desired Y-axis steps) using the `anguloParaPassosX()` and `anguloParaPassosY()` functions.
    * `anguloParaPassosX()` maps the sun's azimuth (0-360 degrees) to the motor's step range (0 to `TOTAL_STEPS_X`). It's currently configured to track azimuths primarily in the 0-90 and 270-360 degree ranges, effectively focusing on the morning and evening sun.
    * `anguloParaPassosY()` maps the sun's elevation (0-90 degrees) to the panel's inclination angle (between `ANGULO_PAINEL_MIN` and `ANGULO_PAINEL_MAX`), and then to the motor's step range (0 to `TOTAL_STEPS_Y`).
2.  **Movement:** The `moverParaXY()` function moves both motors simultaneously until they reach their respective `posicaoDesejadaX` and `posicaoDesejadaY`. This function ensures both axes move in parallel to reach the target.
3.  **Night Mode/Parking:** If the `elevation` is 2 degrees or less, the system considers it nighttime or low-light conditions. It then calls `forcarFimDeCursoX()` and `forcarFimDeCursoY()` to return the panel to its calibrated home (parked) position (0,0) and sets `painelEstacionado` to `true`.

### Calibration

The `forcarFimDeCursoX()` and `forcarFimDeCursoY()` functions are crucial for calibration. They drive the motors to their mechanical limits (end-stops), effectively resetting their internal step counters (`posicaoAtualX` and `posicaoAtualY`) to zero. This ensures accurate positioning regardless of power cycles or previous movements. Calibration is performed once during `setup()` and again when the sun sets (or elevation is low).

### Power Management

* **`digitalWrite(en, HIGH)` / `digitalWrite(en, LOW)`:** Pin A0 (`en`) is used to enable or disable the power to the motor drivers. `HIGH` enables them for movement, `LOW` disables them to save power when not in use.
* **`ligarTimersPWM()` / `desligarTimersPWM()`:** These functions enable/disable the Arduino's internal PWM timers. Turning them off when not needed (e.g., when motors are stopped) further reduces power consumption.
* **`LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF)`:** The `LowPower` library is used to put the Arduino into a deep sleep mode.
    * When the sun is up (`soneca` is true initially, then set to false after movement), the Arduino sleeps for 15 cycles of 8 seconds each (total 120 seconds or 2 minutes) before waking up to re-evaluate the sun's position and potentially move the panel.
    * When the sun is down (`soneca` is false after parking), the Arduino sleeps for 500 cycles of 8 seconds each (total 4000 seconds or approximately 1 hour and 6 minutes), significantly reducing power consumption overnight.

---

## Usage

1.  Power up your Arduino and motor drivers.
2.  Observe the serial monitor (set to 115200 baud) for debug information and system status.
3.  The panel will automatically calibrate, then track the sun throughout the day.
4.  At night (or when elevation is low), it will return to its home position and enter a longer sleep cycle.

---

## Troubleshooting

* **Motors not moving:**
    * Check wiring, especially power connections to motor drivers.
    * Verify `VELOCIDADE_MOTOR_X` and `VELOCIDADE_MOTOR_Y` are non-zero.
    * Ensure the `en` pin is correctly wired and set to `HIGH` during movement.
    * Confirm `TOTAL_STEPS_X` and `TOTAL_STEPS_Y` are not zero.
* **Incorrect tracking:**
    * Double-check `latitude`, `longitude`, and `UTC_OFFSET`.
    * Verify `TOTAL_STEPS_X`, `TOTAL_STEPS_Y`, `ANGULO_PAINEL_MIN`, and `ANGULO_PAINEL_MAX` are accurately configured for your mechanical setup.
    * Ensure the RTC time is correct.
* **Excessive power consumption:**
    * Confirm `LowPower` library is correctly installed and functioning.
    * Ensure `prepararPinosParaSleep()` is being called to set pins to a low state before sleep.
    * Check for any external components drawing power when the system is supposed to be sleeping.
* **"Couldn't find RTC" error:**
    * Verify RTC wiring (SDA, SCL, VCC, GND).
    * Ensure the `RTClib` library is installed.

---

## Future Improvements

* Implement limit switches for more robust end-stop detection instead of relying solely on step counting.
* Add a light sensor (LDR) array for fine-tuning the tracking and providing an alternative tracking method.
* Implement a manual control mode via serial commands.
* Add an LCD display to show current solar position, panel position, and system status.
* Optimize motor control for smoother and more energy-efficient movement (e.g., acceleration/deceleration profiles).
* Consider using a higher-precision RTC for long-term accuracy.
