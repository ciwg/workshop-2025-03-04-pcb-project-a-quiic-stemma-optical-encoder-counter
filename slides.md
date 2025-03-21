class: center, middle

# High-Speed Optical Encoder Counter 

**Compatible with the Qwiic/STEMMA ecosystem from SparkFun and Adafruit**

Rebecca Snyder, Steve Traugott

*CSWG Workshop March 2025*

![:img 3D View, 40%](3dview.png)

---

# Outline

- What is a quadrature optical encoder?
- Why are they needed?
- High speed issues
- Few open-source encoder counter boards right now
- What is i2c?
- What is Qwiic/STEMMA?
- Many available Qwiic/STEMMA microcontroller boards
- Many available Qwiic/STEMMA peripherals
- No available Qwiic/STEMMA high-speed encoder counters

---

# What is a quadrature optical encoder?

- Sensor for tracking rotational position and direction
- Uses two output channels (A and B) offset by 90 degrees
- Optical disk with alternating opaque/transparent sections
- LED light source and photodetector generate signals

![:img Encoder Animation, 40%](https://upload.wikimedia.org/wikipedia/commons/1/1e/Incremental_directional_encoder.gif)
![:img Quadrature encoder signals, 40%](quadrature-signals.jpg)

- [Incremental encoder on Wikipedia](https://en.wikipedia.org/wiki/Incremental_encoder)
- [What is a Quadrature Encoder?](https://www.usdigital.com/blog/what-is-quadrature/)
- [Encoder Guide](https://www.phidgets.com/docs/Encoder_Guide)
- [Quadrature encoder signals](https://www.dynapar.com/technology/encoder_basics/quadrature_encoder/)

---
# How does an optical encoder work?

- Direction of rotation determined by phase relationship
- Counting pulses on both channels gives position and direction

![:img Optical encoder CW, 40%](r900-mvcw-encoder-base-fig3-2.gif)
![:img Optical encoder CC, 40%](r900-mvcc-encoder-base-fig3-2.gif)

- [Tutorial from AKM](https://www.akm.com/global/en/products/rotation-angle-sensor/tutorial/type-mechanism-2/)

---

# Where are optical encoders used?

## Low-speed applications
- User input for rotary controls

![:img Mouse Mechanism, 30%](Mouse-mechanism-cutaway.png)
---
# Where are optical encoders used?

## High-speed applications
- Precise position, speed, and direction sensing for motors
    - motor + encoder = servo
    - closed-loop motor control

![:img Closed-loop motor control, 50%](closed-loop.png)

- [Real-Time Feedback Response](https://www.analog.com/en/lp/001/real-time-feedback-response.html)

---
# Closed-loop motor control

![:img DC Motor Open Loop Vs Closed Loop Demonstrator, 50%](dc-motor-open-loop-vs-closed-loop-demonstrator.png)

- [DC Motor Open Loop Vs Closed Loop Demonstrator](https://www.instructables.com/DC-Motor-Open-Loop-Vs-Closed-Loop-Demonstrator/)

---

# High speed issues

- Signal frequency increases with speed and resolution
- If using two GPIO pins to watch the A and B signals, microcontroller may miss pulses
    - Microcontroller still handling previous pulse when next pulse arrives
- Example: 
    - 2000 pulses/rev at 2,000 RPM * 2 detectors ≈ 133 kHz, or 7.5 µs per pulse
    - If microcontroller takes longer than 7.5 µs to process a pulse, it will miss the next one

![:img High-resolution discs, 70%](disks-resolution.jpg)

- [High-speed quadrature encoder issue](https://forum.pjrc.com/index.php?threads%2Fhigh-speed-quadrature-encoder-issue-teensy-4-0.74408%2F)
- [Optical encoder skips pulses at high speeds](https://forum.arduino.cc/t/optical-encoder-skips-pulses-at-high-speeds/1275203)
---
# Solution:  Use dedicated hardware

- Encoder counter circuit with hardware quadrature decoding
- Counts pulses on A and B signals
- Figures out direction of rotation from phase relationship
- Uses pulse count and direction to maintain a position counter
- Microcontroller reads position counter by some method -- I2C, SPI, serial, etc.

![:img Quadrature decoder, 70%](QuadratureDecoder.png)

[Quadrature decoder](https://en.wikipedia.org/wiki/Incremental_encoder#Quadrature_decoder)
---
# The missing piece

- Open-source encoder counter boards are rare
    - This causes the Maker and open hardware community to use
      open-loop control for motors
    - e.g. most 3D printers are open-loop
- Even bigger gap for boards compatible with Qwiic/STEMMA ecosystem
  from SparkFun and Adafruit
    - Qwiic/STEMMA is a standardized connector for I2C devices
- No Qwiic/STEMMA encoder counter boards known to exist as of this
  writing
    - This makes closed-loop prototyping harder than it should be

![:img Layer shifts, 40%](layer_shifts.jpg)

[Layer shifts](https://wiki.bambulab.com/en/knowledge-sharing/layer-shifts)

---

# What is I2C?

- Inter-Integrated Circuit communication protocol
- Two-wire interface: SDA (data) and SCL (clock)
- Allows multiple devices on the same bus
- Simple and widely supported
    - many microcontrollers have I2C hardware
    - your laptop uses I2C internally to talk to the trackpad, keyboard, etc.

![:img I2C controller target, 50%](I2C_controller-target.png)

- [I2C protocol](https://en.wikipedia.org/wiki/I%C2%B2C)

---

# What is Qwiic/STEMMA?

- Connectors for I2C devices standardized by SparkFun and Adafruit
- Qwiic (SparkFun) and STEMMA QT (Adafruit) are compatible with each
  other
- 4-pin JST SH connector (3.3V, GND, SDA, SCL)
- Speeds up prototyping by eliminating soldering and breadboarding

![:img Qwiic, 40%](qwiic.jpg)
![:img STEMMA QT, 40%](stemma-qt.jpg)

- [SparkFun Qwiic](https://www.sparkfun.com/qwiic)
- [Adafruit STEMMA QT](https://learn.adafruit.com/introducing-adafruit-stemma-qt/what-is-stemma-qt)

---

# Many, many available Qwiic/STEMMA microcontroller boards

SparkFun, Adafruit, and others now make many boards with Qwiic/STEMMA connectors:

- Microcontrollers
  - Arduino-compatible
  - Adafruit Feather and QT Py series
  - Teensy with STEMMA QT adapter
  - Raspberry Pi with STEMMA QT hat

- Peripherals, Sensors, and Displays
  - CO2
  - temperature, humidity, pressure
  - motion (accelerometers, gyroscopes)
  - light

- [SparkFun Qwiic boards](https://www.sparkfun.com/catalogsearch/result/?q=qwiic)
- [Adafruit STEMMA QT boards](https://www.adafruit.com/search?q=stemma+qt)

---

# Qwiic/STEMMA encoder counter

- LSI Logic has a 32-bit counter chip with quadrature decoding and I2C interface (LS7866)
- So we designed a board around it
- Uses Qwiic/STEMMA connectors for easy integration
- Like any Qwiic/STEMMA device, it can be daisy-chained
    - Multiple encoders can be used on the same I2C bus
    - Same bus can be shared with other devices like sensors, displays, etc.

![:img LS7866, 10%](ls7866-TSSOP14.png)
![:img LS7866, 35%](ls7866-bus.jpg)

[LS7866 datasheet](https://lsicsi.com/wp-content/uploads/2023/12/LS7866.pdf)

---

# The story so far

- Ordered Parts - Done!
- Steve shipped kit to Rebecca - Done!
- Both assembled a breadboard prototype - Done!
- Worked together on example code for Particle Photon - Done!
- Tested example code with breadboard prototype - Works!
- Designed v0.1 PCB - Done!
- Ordered v0.1 bare PCB and solder paste stencil - Done!
- Soldered v0.1 PCB - Done!
- Tested v0.1 PCB - Works! 
- Designed v0.2 and v0.3 PCBs - Done!
- Ordered v0.3 bare PCB and solder paste stencil - Done!

---

### Next: 

- v0.3 solder, test, iterate
- write libraries for Particle Photon, Adafruit Feather, Teensy, etc.
    - both C++ and CircuitPython
- commission reflow oven @cdint
- small batch production
- offer on e.g. Tindie or Crowd Supply
- offer on cdint.com (after site redesign)
- offer on Amazon?

--- 
# Breadboard prototype

--- 
# Example code

---
# v0.1 PCB

- schematic
- layout
- 3D view
- OSH Park 
- OSH Stencils 

---
# reflow videos

- [Capacitor reflow soldering](https://www.youtube.com/watch?v=wkkoVMnc4eQ)
- [Chip reflow soldering](youtube.com/watch?v=GyPrWQnuNqw)
- [Stencil](youtube.com/watch?v=uXvXwzQf1gU)

---
# v0.3 PCB

- schematic
- layout
- 3D view


