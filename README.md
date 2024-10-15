# Assitbot

## Arduino Sketch

1. Upload the sketch to the Arduino board.
2. Wiring L298N Motor Driver:
        - Arduino Pins -> Motor Driver Pins
        - D10: L Fwd IN2
        - D6 : L Rev IN1
        - D9 : R Fwd IN4
        - D5 : R Rev IN3
3. Wiring Encoders:
        - Arduino Pins -> Motor Encoder Pins
        - D2: Left A
        - D3: Left B
        - A4: Right A
        - A5: Right B
4. Test: Open the serial monitor `minicom -b 57600 -o -D /dev/ttyACM0`
5. Send the following commands to the Arduino board:
        - `e` - Motor responds with current encoder counts for each motor
        - `r` - Reset encoder values
        - `o <PWM1> <PWM2>` - Set the raw PWM speed of each motor (-255 to 255)
        - `m <Spd1> <Spd2>` - Set the closed-loop speed of each motor in *counts per loop* (Default loop rate is 30, so `(counts per sec)/30`)
6. Test running at 1 rev/sec: `m <spd> <spd>` where `<spd>` is (encoder_resolution/30)

### Setting control cycle target

spd = Speed(rev/sec) * N/R  
spd = Speed(rad/sec) * N/(R * 2 * pi)  

Where:
        - N = Encoder resolution (Counts per full revolution)
        - R = Loop rate (30 Hz)