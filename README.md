# Smart Parking System in Verilog

This project implements a Smart Parking System using Verilog HDL, designed for simulation and potential FPGA implementation. The system uses an ultrasonic sensor to detect vehicles, IR sensors to monitor parking slot availability, and a servo motor to control a gate. It also includes LED indicators to show parking availability (green for available, red for full). The design is synchronous, with a state machine managing the system's operation, making it suitable for educational purposes and real-world parking management applications.

## 📁 Project Structure
```bash
├── SmartParkingSystem.v    # Main module for the Smart Parking System
```


## 🔧 Features

- **Ultrasonic Vehicle Detection**: Uses an ultrasonic sensor (via `echo` and `trigger`) to detect vehicles within a 25 cm threshold.
- **Parking Slot Monitoring**: Monitors four parking slots using IR sensors (`ir_sensors[3:0]`).
- **Servo-Controlled Gate**: Controls a gate using PWM signals (`servo_pwm`) to open/close based on slot availability.
- **LED Indicators**:
  - `green_led`: Lights up when parking slots are available.
  - `red_led`: Lights up when all slots are occupied.
- **State Machine Design**: Implements a finite state machine (FSM) with states: `IDLE`, `CHECK_SLOTS`, `OPEN_GATE`, `HOLD_GATE_OPEN`, and `CLOSE_GATE`.
- **Simulation Ready**: Designed for simulation in tools like Xilinx Vivado or ModelSim.

## 🧠 How It Works

The `SmartParkingSystem.v` module integrates the following components:

1. **Ultrasonic Sensor Logic**:
   - Generates a 10 µs trigger pulse (`trigger`) every 3 million clock cycles.
   - Measures the `echo` signal duration to calculate distance.
   - Detects a vehicle if the distance is less than 25 cm (`DISTANCE_THRESHOLD`).

2. **Parking Slot Monitoring**:
   - Uses `ir_sensors[3:0]` to monitor four parking slots (0 = occupied, 1 = free).
   - Updates `slot_status` to reflect the current availability.

3. **Servo PWM Control**:
   - Generates a PWM signal (`servo_pwm`) with a 20 ms period (`PWM_PERIOD` = 1,000,000 cycles at 50 MHz).
   - Duty cycle varies between `PWM_MIN` (50,000 cycles, ~1 ms) for closed gate and `PWM_MAX` (100,000 cycles, ~2 ms) for open gate.

4. **State Machine**:
   - **IDLE**: Monitors vehicle presence and slot availability. Sets LEDs based on `slot_status`.
   - **CHECK_SLOTS**: Checks if slots are available when a vehicle is detected.
   - **OPEN_GATE**: Opens the gate by setting the PWM duty cycle to `PWM_MAX`.
   - **HOLD_GATE_OPEN**: Holds the gate open for 250 million clock cycles (~5 seconds at 50 MHz).
   - **CLOSE_GATE**: Closes the gate by setting the PWM duty cycle to `PWM_MIN` and returns to `IDLE`.

## 📥 Inputs

- `clk`: System clock (50 MHz, configurable via `CLK_FREQ`).
- `reset_n`: Active-low synchronous reset.
- `echo`: Echo signal from the ultrasonic sensor (high when reflecting sound wave is received).
- `ir_sensors[3:0]`: Four IR sensor inputs (0 = slot occupied, 1 = slot free).

## 📤 Outputs

- `trigger`: Trigger pulse for the ultrasonic sensor (10 µs pulse).
- `servo_pwm`: PWM signal to control the servo motor for the gate.
- `red_led`: Indicates all slots are occupied (high when `slot_status == 4'b1111`).
- `green_led`: Indicates slots are available (high when `slot_status != 4'b1111`).

## 🚀 Simulation and Testing

You can simulate the project using tools like Xilinx Vivado, ModelSim, or Icarus Verilog.

### Steps to Simulate

1. **Set Up the Project**:
   - Create a new project in your Verilog simulator (e.g., Vivado or ModelSim).
   - Add `SmartParkingSystem.v` as the source file.

2. **Create a Testbench**:
   - Create a testbench file (e.g., `SmartParkingSystem_tb.v`) to simulate the system. Below is an example:

```verilog
module SmartParkingSystem_tb;
    reg clk;
    reg reset_n;
    reg echo;
    reg [3:0] ir_sensors;
    wire trigger;
    wire servo_pwm;
    wire red_led;
    wire green_led;

    // Instantiate the Smart Parking System
    SmartParkingSystem uut (
        .clk(clk),
        .reset_n(reset_n),
        .echo(echo),
        .ir_sensors(ir_sensors),
        .trigger(trigger),
        .servo_pwm(servo_pwm),
        .red_led(red_led),
        .green_led(green_led)
    );

    // Clock generation (50 MHz)
    always #10 clk = ~clk; // 20 ns period

    initial begin
        // Initialize signals
        clk = 0;
        reset_n = 0;
        echo = 0;
        ir_sensors = 4'b1111; // All slots initially occupied

        // Reset the system
        #100 reset_n = 1;

        // Test case 1: No vehicle, all slots occupied
        #10_000_000;
        ir_sensors = 4'b0111; // One slot free
        #10_000_000;

        // Test case 2: Vehicle detected, slot available
        echo = 1;
        #1160 echo = 0; // Simulate echo for 20 cm distance (1160 cycles = 20 cm * 58)
        #10_000_000;

        // Test case 3: Reset while gate is open
        #100_000_000;
        reset_n = 0;
        #100 reset_n = 1;

        #10_000_000;
        $finish;
    end

    // Monitor signals
    initial begin
        $monitor("Time=%0t, State=%b, Vehicle=%b, Slots=%b, Red=%b, Green=%b, Servo=%b",
                 $time, uut.state, uut.vehicle_detected, ir_sensors, red_led, green_led, servo_pwm);
    end
endmodule
```
## Run the Simulation

- Add the testbench to your project.
- Simulate the design and observe the waveform or console output to verify:
  - `trigger` pulses every 3 million cycles.
  - `vehicle_detected` goes high when `echo` duration indicates a distance < 25 cm.
  - `red_led` and `green_led` reflect slot availability.
  - `servo_pwm` duty cycle changes to open/close the gate.

## 🔍 Applications

- **Educational Demo**: Ideal for learning Verilog, state machines, and sensor integration in digital systems.
- **Smart Parking Solutions**: Can be used as a prototype for automated parking systems in small lots or garages.
- **Embedded Systems**: Serves as a foundation for IoT-based parking management with further enhancements.

## 📌 Future Enhancements

- **Vehicle Counting**: Add a counter to track the number of vehicles entering/exiting.
- **Display Integration**: Interface with a 7-segment display or LCD to show available slots.
- **FPGA Implementation**: Adapt the design for FPGA deployment with real hardware interfacing.
- **Advanced Sensors**: Incorporate additional sensors (e.g., cameras) for more accurate vehicle detection.
- **Power Optimization**: Implement clock gating to reduce power consumption in idle states.

## 📝 Author

**Kathir S**  
B.E. Electronics and Communication Engineering  
Anna University, Coimbatore

## 📜 License

This project is for educational purposes. You may use or modify it freely with attribution.

**Instructions to Access**:

- Click the link above to open the Google Drive folder.
- Ensure you are logged into a Google account with access (contact the owner if access is restricted).
- Download or view files directly in the browser.
- To contribute, request edit access from the folder owner or drag and drop files into the shared folder if permissions allow.

**Note**: The Google Drive folder is set to "Anyone with the link" for view access. For sensitive projects, adjust permissions to "Restricted" or specific email access for security.

## Troubleshooting

- **Ultrasonic Detection Issues**: Ensure the `echo` signal duration in the testbench matches the expected distance (distance in cm * 58 cycles). For example, 20 cm requires 1160 cycles.
- **Servo PWM Not Working**: Verify the `pwm_duty_cycle` transitions between `PWM_MIN` and `PWM_MAX` in the waveform. Check the state machine transitions.
- **LEDs Not Reflecting Status**: Confirm the `ir_sensors` input correctly reflects slot occupancy (0 for occupied, 1 for free). Check the state machine logic for `red_led` and `green_led`.
- **Simulation Hangs**: Ensure the clock frequency (`CLK_FREQ`) matches your testbench clock period. Adjust timing parameters if using a different clock frequency.

## Acknowledgments

- Xilinx Vivado for providing a platform for Verilog simulation.
- Anna University, Coimbatore, for supporting this educational project.
