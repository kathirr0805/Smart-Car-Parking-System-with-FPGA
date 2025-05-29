module SmartParkingSystem(
    input clk,
    input reset_n,
    input echo,
    input [3:0] ir_sensors,
    output reg trigger,
    output reg servo_pwm,
    output reg red_led,
    output reg green_led
);

//------------------------------
// PARAMETERS
//------------------------------
localparam CLK_FREQ = 50_000_000;
localparam TRIGGER_PULSE_US = 10;
localparam MEASURE_INTERVAL = 3_000_000;
localparam DISTANCE_THRESHOLD = 25;
localparam CM_CONVERSION = 58;

localparam PWM_PERIOD = 1_000_000;
localparam PWM_MIN = 50_000;
localparam PWM_MAX = 100_000;

localparam IDLE           = 3'b000;
localparam CHECK_SLOTS    = 3'b001;
localparam OPEN_GATE      = 3'b010;
localparam HOLD_GATE_OPEN = 3'b011;
localparam CLOSE_GATE     = 3'b100;

//------------------------------
// REG/WIRE DECLARATIONS
//------------------------------
reg [31:0] ultrasonic_counter;
reg [31:0] echo_counter;
reg [2:0] state;
reg vehicle_detected;
reg [3:0] slot_status;
reg [31:0] pwm_counter;
reg [31:0] gate_timer;

// FIX: Single controlled signal for duty cycle
reg [31:0] pwm_duty_cycle;
reg [31:0] next_pwm_duty_cycle;

//------------------------------
// ULTRASONIC SENSOR
//------------------------------
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        ultrasonic_counter <= 0;
        trigger <= 0;
        echo_counter <= 0;
        vehicle_detected <= 0;
    end else begin
        if (ultrasonic_counter < MEASURE_INTERVAL) begin
            ultrasonic_counter <= ultrasonic_counter + 1;
            trigger <= (ultrasonic_counter < (TRIGGER_PULSE_US * (CLK_FREQ/1_000_000)));
        end else begin
            ultrasonic_counter <= 0;
        end

        if (echo) begin
            echo_counter <= echo_counter + 1;
        end else if (echo_counter > 0) begin
            vehicle_detected <= (echo_counter < (DISTANCE_THRESHOLD * CM_CONVERSION));
            echo_counter <= 0;
        end
    end
end

//------------------------------
// SERVO PWM (Single Source)
//------------------------------
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        pwm_counter <= 0;
        servo_pwm <= 0;
        pwm_duty_cycle <= PWM_MIN;
    end else begin
        // Update duty cycle from FSM
        pwm_duty_cycle <= next_pwm_duty_cycle;

        if (pwm_counter < PWM_PERIOD) begin
            pwm_counter <= pwm_counter + 1;
            servo_pwm <= (pwm_counter < pwm_duty_cycle);
        end else begin
            pwm_counter <= 0;
        end
    end
end

//------------------------------
// STATE MACHINE (Single Driver)
//------------------------------
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        state <= IDLE;
        green_led <= 0;
        red_led <= 0;
        slot_status <= 4'b1111;
        gate_timer <= 0;
        next_pwm_duty_cycle <= PWM_MIN;
    end else begin
        slot_status <= ir_sensors;

        case (state)
            IDLE: begin
                red_led <= (slot_status == 4'b1111);
                green_led <= (slot_status != 4'b1111);
                next_pwm_duty_cycle <= PWM_MIN;

                if (vehicle_detected) begin
                    state <= CHECK_SLOTS;
                end
            end

            CHECK_SLOTS: begin
                if (slot_status != 4'b1111) begin
                    state <= OPEN_GATE;
                    green_led <= 1;
                    red_led <= 0;
                end else begin
                    state <= IDLE;
                    red_led <= 1;
                    green_led <= 0;
                end
            end

            OPEN_GATE: begin
                next_pwm_duty_cycle <= PWM_MAX;
                state <= HOLD_GATE_OPEN;
                gate_timer <= 0;
            end

            HOLD_GATE_OPEN: begin
                if (gate_timer < 250_000_000) begin
                    gate_timer <= gate_timer + 1;
                end else begin
                    state <= CLOSE_GATE;
                end
            end

            CLOSE_GATE: begin
                next_pwm_duty_cycle <= PWM_MIN;
                state <= IDLE;
                green_led <= 0;
                red_led <= (slot_status == 4'b1111);
            end

            default: state <= IDLE;
        endcase
    end
end

endmodule
