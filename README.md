# Remote-Drone-Control-System
ESP32 drone controller manages servos, motors, and lights via a web interface. Python Flask app with OpenCV detects people in USB camera video and streams it in real-time.
# ESP32 & Linux Web Control System for a VTOL UAV Stand

This repository documents a comprehensive remote control system for a Vertical Take-Off and Landing (VTOL) UAV stand. The system provides robust control over all actuators and aviation lights via a local Wi-Fi web interface, powered by an **ESP32**.

It is augmented by a secondary **ARM Linux computer (Rockchip RK3188)** that performs real-time **Computer Vision** tasks (HOG pedestrian detection) and streams the processed FPV feed back to the main control UI.

This project was developed at the National University of Water and Environmental Engineering (Rivne, Ukraine).

## Live Demo
<div style="text-align: center;">
  <a href="https://www.youtube.com/watch?v=qL3FL8OUee4">
    <img src="https://img.youtube.com/vi/qL3FL8OUee4/0.jpg" alt="ESP32 Exhibition Stand Demo" style="max-width: 100%; height: auto;">
  </a>
</div>

## About This Project: A Note on the Development

The development process was a pragmatic blend of hands-on hardware engineering and rapid software integration.

Modern tools, including AI code generation, were leveraged to write the boilerplate for the C++ and Python applications. The primary engineering challenge was not in writing *every* line of code, but in:

1.  **Architecture:** Designing the two-device (ESP32 + Linux) system to separate real-time tasks from heavy processing.
2.  **Integration:** Making multiple, disparate technologies (C++, Python, Flask, OpenCV, HTML/JS, `systemd`, and the specific hardware) all work together reliably.
3.  **Hardware:** Sourcing, wiring, and physically building the stand, motor drivers, and servo gimbals.
4.  **Debugging:** Iteratively fixing and adapting the generated code to work on the real-world hardware.

**Note on AI vs. Computer Vision:** This project uses classic **Computer Vision** (OpenCV with a HOG descriptor) for pedestrian detection, *not* a deep learning "AI" model. The term "AI-assisted" refers to the *development process* (using generative AI for coding), not the *product's functionality*.

-----

## Core Concept: The Two-Device Architecture

This system's stability comes from intelligently splitting the workload between two specialized components:

1.  **The Controller (ESP32):** Acts as the **Real-Time Control Hub**. Its sole job is to handle tasks that require high reliability and low latency:

      * Creating the Wi-Fi Access Point & DNS Server.
      * Hosting the Web UI (HTML/CSS/JS).
      * Handling API requests from the user.
      * Generating stable PWM signals for 6 servos and the L298N motor drivers.

2.  **The Vision Server (Rockchip RK3188):** Acts as the **"Eyes" and "Brain"**. This more powerful ARM Linux computer handles the computationally "heavy" tasks:

      * Capturing the USB camera feed.
      * Running the OpenCV HOG algorithm on every frame.
      * Drawing bounding boxes on detected objects.
      * Re-encoding and streaming the processed MJPEG feed.

This separation ensures that a spike in the Computer Vision processing load **will never** cause a stutter or delay in the user's flight control commands.

-----

## Key Features

### (A) The Controller (ESP32)

  * **Self-Contained Network:** Creates its own "DroneController" Wi-Fi Access Point.
  * **Captive Portal:** Automatically redirects any connected device to the control interface, eliminating the need to type an IP address.
  * **Rich Web Interface:** Serves a single-page application with:
      * Flight Joystick (Ailerons/Elevator).
      * Camera Pan/Tilt Sliders.
      * Yaw (Rudder) Slider.
      * Traction Motor Speed/Direction Slider.
      * Flaps Control (Modal Popup).
  * **Secure Control System:**
      * **Exclusive Control:** Only one user can "Get Control" at a time.
      * **Inactivity Timeout:** Automatically releases control and returns all servos to neutral after 30 seconds of user inactivity.
      * **Rate Limiting:** Protects the ESP32 from being flooded by HTTP requests.
  * **Full Hardware Control:**
      * Smooth, non-blocking control of 6 distinct servos.
      * Transistor-driven control of all aviation lights (NAV, BEACON, STROBE).
      * PWM speed and direction control for vertical lift motors via L298N drivers.
  * **"Quiet Mode":** Servos are automatically `detached()` when in neutral to stop humming/buzzing and save power. They `attach()` instantly when control is taken.

### (B) The FPV Vision Server (Rockchip RK3188)

  * **Computer Vision FPV:** Streams a low-latency video feed with **real-time pedestrian detection** (using OpenCV's HOG descriptor).
  * **Fully Autonomous Operation:**
    1.  **Auto-Connects to WiFi:** The Linux OS (Armbian) is configured to automatically join the ESP32's "DroneController" network on boot.
    2.  **Static IP:** The server is assigned a fixed IP (`192.168.4.250`) so the web interface can always find it.
    3.  **Auto-Start Daemon:** The Python/Flask server runs as a `systemd` service, launching automatically in the background as soon as the device boots.
  * **Efficient Streaming:** Uses a Flask server to stream the processed video as an MJPEG feed, which is embedded directly into the ESP32's web interface.

-----

## Hardware Breakdown

| Component | Role | Specific Model Used |
| :--- | :--- | :--- |
| **Main Controller** | Real-time control, Web Server, AP | **ESP32-WROOM-32** |
| **Vision Server** | CV Processing, Video Stream | **Rockchip RK3188** (2GB RAM) ARM Computer |
| **Camera Gimbal** | FPV Camera Pan/Tilt | 2x **SG90** Micro Servo |
| **Tail Ailerons** | Control Surfaces | 2x **MG90S** Metal Gear Servo |
| **Wing Ailerons** | Control Surfaces | 2x **MG996R** High Torque Servo |

  * **Motor Drivers:** 2x **L298N** Dual H-Bridge Drivers (for vertical lift motors).
  * **Thrust Motor Driver:** 1x **L298N** (for main propulsion).
  * **Aviation Lights:** Red, Green, and White LEDs driven by **TIP141 Transistors**.
  * **Camera:** Standard USB Webcam.
  * **Power:** 5V-12V power source (e.g., LiPo + BEC) for motors and servos.

-----

## System Architecture & UI

### System Diagram

This diagram shows the flow of information between the user, the ESP32, and the Linux server.

\<p align="center"\>
\<b\>[ --- PLACEHOLDER FOR YOUR "Fig. 2 Structural diagram" --- ]\</b\>
<br>
\<em\>(Insert your structural diagram image here)\</em\>
\</p\>

### Web Interface

The unified web interface provides all controls and the processed FPV feed in a single screen.

\<p align="center"\>
\<b\>[ --- PLACEHOLDER FOR YOUR "Fig. 1 User interface" --- ]\</b\>
<br>
\<em\>(Insert a screenshot of your web UI here)\</em\>
\</p\>

-----

## Controller Pinout (ESP32)

| Function | ESP32 Pin |
| :--- | :--- |
| **Aileron - Front Left** (MG996R) | 33 |
| **Aileron - Front Right** (MG996R) | 17 |
| **Aileron - Rear Left** (MG90S) | 32 |
| **Aileron - Rear Right** (MG90S) | 16 |
| **Camera - Pan** (SG90) | 27 |
| **Camera - Tilt** (SG90) | 14 |
| **Wing Motor 1** (L298N) | 13 |
| **Wing Motor 2** (L298N) | 25 |
| **Traction Motor EN (PWM)** | 12 |
| **Traction Motor IN1** | 5 |
| **Traction Motor IN2** | 19 |
| **NAV Light Left (Red)** | 15 |
| **NAV Light Right (Green)** | 4 |
| **NAV Light Rear (White)** | 18 |
| **BEACON Light (White)** | 23 |
| **STROBE Light (White)** | 21 |

-----

## Installation & Setup

This is a two-part setup. You must configure both the Controller and the FPV Server.

### Part 1: The Controller (ESP32)

This part is designed for **maximum simplicity**. All code (C++, HTML, CSS, and JavaScript) is contained in **one single `.ino` file** (`DroneController_ESP32.ino`) for easy, one-click flashing. No SPIFFS or data uploading is required.

1.  **Clone the Repository:**
    ```bash
    git clone https://github.com/your-username/your-repo-name.git
    ```
2.  **Install Arduino IDE** and the [ESP32 Board definitions](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html).
3.  **Install Libraries:**
    Open the Arduino IDE and go to `Tools > Manage Libraries...`. Install:
      * `ESP32Servo` by Kevin Harrington
      * (All other libraries like `WiFi`, `WebServer`, and `DNSServer` are built-in)
4.  **Open & Flash:**
      * Open the `DroneController_ESP32.ino` file in the Arduino IDE.
      * Select your ESP32 board and COM port.
      * Click the **"Upload"** button.

The controller is now ready.

### Part 2: The FPV Server (Rockchip RK3188)

This part assumes you have flashed your ARM device with a compatible Linux OS (e.g., **Armbian**).

1.  **Configure Network (The "Auto-Connect" Magic):**
    Your Linux system must be configured to **automatically connect to the ESP32's WiFi** and **set a static IP**.

      * Edit your network configuration file (e.g., `/etc/network/interfaces` or via `nmtui`).
      * Set it to connect to `SSID: DroneController` with `PSK: flyhigh123`.
      * Assign it the following static IP configuration:
          * **IP Address:** `192.168.4.250`
          * **Netmask:** `255.255.255.0`
          * **Gateway:** `192.168.4.1` (The ESP32's IP)

2.  **Install Dependencies:**

      * Connect your device to the internet (temporarily) to install software.
      * Navigate to the `fpv_server` directory from the cloned repository.
      * Install Python and the required libraries:
        ```bash
        sudo apt-get update
        sudo apt-get install python3 python3-pip
        pip3 install -r requirements.txt
        ```

3.  **Configure Auto-Start (The "Daemon" Magic):**
    We will use `systemd` to make the Python script run automatically on boot.

      * Create a new service file:
        ```bash
        sudo nano /etc/systemd/system/fpv.service
        ```
      * Paste the following configuration. **Remember to change `/path/to/`** to the *actual* full path of your `fpv_server` directory.
        ```ini
        [Unit]
        Description=FPV Video Server for Drone Controller
        After=network-online.target
        Wants=network-online.target

        [Service]
        ExecStart=/usr/bin/python3 /path/to/fpv_server/video_server.py
        WorkingDirectory=/path/to/fpv_server
        StandardOutput=inherit
        StandardError=inherit
        Restart=always
        User=root (or your login user)

        [Install]
        WantedBy=multi-user.target
        ```
      * Enable and start the service:
        ```bash
        sudo systemctl daemon-reload
        sudo systemctl enable fpv.service
        sudo systemctl start fpv.service
        ```
      * The FPV server will now start automatically every time the device boots up.

-----
Safety and Reliability Features
The system was engineered with multiple layers of safety to prevent hardware damage and unpredictable behavior. These features operate automatically at the microcontroller level.

Exclusive Control: Only one user (IP address) can gain control at a time. This prevents conflicting commands if multiple users are connected to the web interface.

Automatic Motor Timeout: All motors (both vertical lift and main thrust) are programmed to automatically shut down after 30 seconds of continuous operation (motorTimeout). This acts as a dead-man's switch to prevent the motors from running indefinitely if the connection is lost or the UI freezes.

User Inactivity Timeout: If the system detects no user activity (no clicks or joystick moves) for 30 seconds (controlTimeout), it automatically releases control, stops all motors, and returns all servos to their neutral "quiet" state.

HTTP Request Limiter: To protect the ESP32's web server from being overwhelmed by a buggy or malicious script, a rate limiter (checkRequestLimit()) is built-in, capping the number of commands accepted per second.

Input Sanitization: All incoming values from the web interface (joystick coordinates, slider percentages) are strictly sanitized using the constrain() function. This ensures that no out-of-range value can be written to the servos or motor drivers, protecting them from physical damage.

To ensure an installation and robust mounting for key components, several custom parts were designed and fabricated using a 3D printer. This includes specialized  housings for the vertical lift motors, providing optimal positioning for the stand.

## How to Use

1.  **Power On:** Turn on the ESP32 controller and the Rockchip Linux device.
2.  **Wait:** Give both devices \~30 seconds to boot up. The FPV server will automatically connect to the ESP32's network and start the video service.
3.  **Connect:** On your phone or laptop, connect to the WiFi network named **"DroneController"** (password: `flyhigh123`).
4.  **Login:** Your device's Captive Portal should automatically open a login page. If not, open a browser and go to `192.168.4.1`.
5.  **Start:** Click the **"Почати" (Start)** button.
6.  **Control:** You will be redirected to the main control panel. The FPV video feed (with HOG detection boxes) should appear automatically.
7.  **IMPORTANT:** You must click **"ПОЧАТИ КЕРУВАТИ" (GET CONTROL)** to activate the joysticks and sliders.
8.  You are now in full control.

-----

## References

[1] Tsukanov O.S., Reut D.T. СИСТЕМА ДИСТАНЦІЙНОГО КЕРУВАННЯ ДРОНОМ ЧЕРЕЗ ВЕБ-СЕРВЕР НА ESP32 [Remote Drone Control System via ESP32 Web Server]. *Zbirnyk tez dopovidei Vseukrainskoi naukovo-praktychnoi konferentsii zdobuvachiv vyshchoi osvity ta molodykh vchenykh "VODA. ZEMLYA. ENERGETYKA"*, Rivne, 15 May 2025. Rivne: NUWEE, 2025. [in Ukrainian]

[2] Pallets, “Flask,” GitHub repository, [https://github.com/pallets/flask](https://github.com/pallets/flask)

[3] OpenCV, "HOG Descriptor and Object Detection," Documentation, [Online]. Available: [https://docs.opencv.org/4.x/d5/d33/structcv\_1\_1HOGDescriptor.html](https://docs.opencv.org/4.x/d5/d33/structcv_1_1HOGDescriptor.html)

## License

This project is licensed under the MIT License. See the `LICENSE` file for details.
