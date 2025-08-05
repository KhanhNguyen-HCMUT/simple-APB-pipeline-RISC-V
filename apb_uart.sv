// =============================================================================
// COMPLETE UART SYSTEM WITH APB INTERFACE - SINGLE FILE
// All dependencies included for easy integration
// =============================================================================

// =============================================================================
// 1. BAUD RATE GENERATOR - TRANSMITTER
// =============================================================================
module BaudGenTx(
    input logic         reset_n,           // Active low reset
    input logic         clock,             // The system's main clock
    input logic [1:0]	baud_rate,         // Baud rate agreed upon by the Tx and Rx units

    output logic        baud_clk           // Clocking output for the other modules
);

	// Internal declarations
	logic [27:0] clock_ticks;
	logic [27:0] final_value;

	// Encoding for the Baud Rates states
	localparam [2:0]
        BAUD24  = 2'b00,
        BAUD48  = 2'b01,
        BAUD96  = 2'b10,
        BAUD192 = 2'b11;

	// BaudRate 4-to-1 Mux
	always_comb begin
		case (baud_rate)
			// All these ratio ticks are calculated for 50MHz Clock,
         // The values shall change with the change of the clock frequency.
         BAUD24:  final_value = 14'd10417;  // Ratio ticks for the 2400 BaudRate
         BAUD48:  final_value = 14'd5208;   // Ratio ticks for the 4800 BaudRate
         BAUD96:  final_value = 14'd2604;   // Ratio ticks for the 9600 BaudRate
         BAUD192: final_value = 14'd1302;   // Ratio ticks for the 19200 BaudRate
         default: final_value = 14'd9999;      // The system's original clock
		endcase
	end

	// Timer logic
   always_ff @(posedge clock or negedge reset_n) begin
		if (!reset_n) begin
			clock_ticks <= 28'd0;
         baud_clk    <= 1'b0;
      end else if (clock_ticks == final_value) begin
			// Ticks whenever it reaches its final value,
         // Then resets and starts all over again.
			clock_ticks <= 28'd0;
         baud_clk    <= ~baud_clk;
      end else begin
         clock_ticks <= clock_ticks + 1'd1;
         baud_clk    <= baud_clk;  // Hold current state
      end
    end

endmodule

// =============================================================================
// 2. BAUD RATE GENERATOR - RECEIVER
// =============================================================================
module BaudGenRx(
    input logic         reset_n,     // Active low reset
    input logic         clock,       // The system's main clock
    input logic  [1:0]  baud_rate,   // Baud rate agreed upon by the Tx and Rx units

    output logic        baud_clk     // Clocking output for the other modules
);

    // Internal declarations
    logic [9:0] final_value;  // Holds the number of ticks for each baud rate
    logic [9:0] clock_ticks;  // Counts until it equals final_value (timer principle)

    // Encoding the different Baud Rates
 localparam [2:0]
        BAUD24  = 2'b00,
        BAUD48  = 2'b01,
        BAUD96  = 2'b10,
        BAUD192 = 2'b11;
    // Baud rate 4-to-1 Mux
    always_comb begin
        case (baud_rate)
            BAUD24:  final_value = 10'd651;  // 16 * 2400 baud rate
            BAUD48:  final_value = 10'd326;  // 16 * 4800 baud rate
            BAUD96:  final_value = 10'd163;  // 16 * 9600 baud rate
            BAUD192: final_value = 10'd81;   // 16 * 19200 baud rate
            default: final_value = 10'd163;  // Default to 9600 baud rate
        endcase
    end

    // Timer logic
    always_ff @(negedge reset_n or posedge clock) begin
        if (!reset_n) begin
            clock_ticks <= 10'd0;
            baud_clk    <= 1'b0;
        end else begin
            // Ticks whenever reaches its final value
            if (clock_ticks == final_value) begin
                baud_clk    <= ~baud_clk;
                clock_ticks <= 10'd0;
            end else begin
                clock_ticks <= clock_ticks + 1'd1;
                baud_clk    <= baud_clk;  // Hold current baud_clk state
            end
        end
    end

endmodule

// =============================================================================
// 3. PARITY GENERATION
// =============================================================================
module Parity(
    input logic        reset_n,     // Active low reset
    input logic [7:0]  data_in,     // The data input from the InReg unit
    input logic [1:0]  parity_type, // Parity type agreed upon by the Tx and Rx units

    output logic       parity_bit   // The parity bit output for the frame
);

    // Encoding for the parity types
	localparam [2:0]
        NOPARITY00 = 2'b00,
        ODD        = 2'b01,
        EVEN       = 2'b10,
        NOPARITY11 = 2'b11;

    // Parity logic with asynchronous active low reset
    always_comb begin
        if (!reset_n) begin
            // No parity bit
            parity_bit = 1'b1;
        end else begin
            case (parity_type)
                NOPARITY00, NOPARITY11: parity_bit = 1'b1; // No parity bit
                ODD:                   parity_bit = (^data_in) ? 1'b0 : 1'b1; // Odd parity
                EVEN:                  parity_bit = (^data_in) ? 1'b1 : 1'b0; // Even parity
                default:               parity_bit = 1'b1; // No parity
            endcase
        end
    end

endmodule

// =============================================================================
// 4. PARALLEL-IN SERIAL-OUT (PISO) - TRANSMITTER CORE
// =============================================================================
module PISO(
    input  logic         reset_n,            //  Active low reset.
    input  logic         send,               //  An enable to start sending data.
    input  logic         baud_clk,           //  Clocking signal from the BaudGen unit.
    input  logic         parity_bit,         //  The parity bit from the Parity unit.
	 input  logic [1:0]   parity_type,        //  Parity type input.
    input  logic [ 7:0]   data_in,            //  The data input.

    output logic         data_tx,            //  Serial transmitter's data out
    output logic         active_flag,        //  High when Tx is transmitting, low when idle.
	 output logic [ 3:0]   stop_count, bit_count,
    output logic [10:0]  frame_r,
    output logic [10:0]  frame_man,
    output logic         next_state,
    output logic         count_full,
    output logic         done_flag           //  High when transmission is done, low when active.
);

    // Frame generation
    always_ff @(posedge baud_clk or negedge reset_n) begin
        if (!reset_n)
            frame_r <= {11{1'b1}};
        else if (next_state)
            frame_r <= frame_r;
        else
            frame_r <= {1'b1, parity_bit, data_in, 1'b0};
    end

    // Counter logic
    always_ff @(posedge baud_clk or negedge reset_n) begin
        if (!reset_n)
            stop_count <= 4'd0;
		  else if ((!next_state) || count_full)
				stop_count <= 4'd0;
        else
            stop_count <= stop_count + 4'd1;
    end

    assign count_full = (stop_count == 4'd11);
    assign bit_count = stop_count; // For compatibility

    // Transmission logic FSM
    always_ff @(posedge baud_clk or negedge reset_n) begin
        if (!reset_n)
            next_state <= 1'b0;
        else begin
            case (next_state)
                1'b0: begin
                    if (send) begin
                        next_state <= 1'b1;
                    end else
                        next_state <= 1'b0;
                end
                1'b1: begin
                    if (count_full) begin
                        next_state <= 1'b0;
                    end else
                        next_state <= 1'b1;
                end
            endcase
        end
    end

	always@ (*) begin
        if (next_state && (stop_count != 4'd0)) begin
            data_tx      = frame_man[stop_count - 1];
            active_flag  = 1'b1;
            done_flag    = 1'b0;
        end
        else begin
            data_tx      = 1'b1;
            frame_man    = frame_r;
            active_flag  = 1'b0;
            done_flag    = 1'b1;
        end
    end

endmodule

// =============================================================================
// 5. SERIAL-IN PARALLEL-OUT (SIPO) - RECEIVER CORE
// =============================================================================
module SIPO(
    input  logic				reset_n,        //  Active low reset.
    input  logic				data_tx,        //  Serial Data recieved from the transmitter.
    input  logic				baud_clk,       //  The clocking input comes from the sampling unit.
	 input  logic rx_enable,

    output logic  			active_flag,    //  outputs logic 1 when data is in progress.
    output logic          	recieved_flag,  //  outputs a signal enables the deframe unit. 
    output logic  [10:0]	data_parll      //  outputs the 11-bit parallel frame.
);
	//  Internal
	logic [3:0]  frame_counter;
	logic [3:0]  stop_count;
	logic [1:0]  next_state;

	//  Encoding the states of the reciever
	//  Every State captures the corresponding bit from the frame
	localparam IDLE   = 2'b00,
				  CENTER = 2'b01,
              FRAME  = 2'b10,
              HOLD   = 2'b11;

	//  FSM with Asynchronous Reset logic
	always_ff @(posedge baud_clk or negedge reset_n) begin
		if (~reset_n) begin
			next_state        <= IDLE;
		end else begin
			case (next_state)
			//  Idle case waits untill start bit
				IDLE : begin
					data_parll    <= {11{1'b1}};
					stop_count    <= 4'd0;
					frame_counter <= 4'd0;
					recieved_flag <= 1'b0;
					active_flag   <= 1'b0;
					//  waits till sensing the start bit which is low
					if(~data_tx) begin
						next_state  <= CENTER;
						active_flag <= 1'b1;
					end else begin
						next_state  <= IDLE;
						active_flag <= 1'b0;
					end
				end

				//  shifts the sampling to the Center of the recieved bit
				//  due to the protocol, thus the bit is stable.
				CENTER : begin
					if(stop_count == 7) begin
						//  Captures the start bit
						data_parll[0]  <= data_tx;
						stop_count     <= 4'd0;
						next_state     <= FRAME;
					end else begin
						stop_count  <= stop_count + 4'b1;
						next_state  <= CENTER;
					end
				end

				//  shifts the remaining 10-bits of the frame,
				//  then returns to the idle case.
				FRAME : begin
					if(frame_counter == 4'd10) begin
						frame_counter <= 4'd0;
						recieved_flag <= 1'b1;
						next_state    <= HOLD;
						active_flag   <= 1'b0;
					end else if(stop_count == 4'd15) begin
						data_parll[frame_counter + 4'd1]    <= data_tx;
						frame_counter                       <= frame_counter + 4'b1;
						stop_count                          <= 4'd0; 
						next_state                          <= FRAME;
					end else begin
						stop_count <= stop_count + 4'b1;
						next_state <= FRAME;
					end
				end

				//  Holds the data recieved for a 16 baud cycles
				HOLD : begin
					if(stop_count == 4'd15) begin
						data_parll    <= data_parll;
						frame_counter <= 4'd0;
						stop_count    <= 4'd0; 
						recieved_flag <= 1'b0;
						next_state    <= IDLE;
					end else begin
						stop_count <= stop_count + 4'b1;
						next_state <= HOLD;
					end
				end

				//  Automatically directs to the IDLE state
				default : begin
					next_state <= IDLE;
				end
			endcase
		end
	end

endmodule

// =============================================================================
// 6. DEFRAME - EXTRACT FRAME COMPONENTS
// =============================================================================
module DeFrame(
    input  logic        reset_n,        // Active low reset
    input  logic        recieved_flag,  // Enable indicates when data is in progress
    input  logic [10:0]	data_parll,     // Data frame passed from the SIPO unit

    output logic        parity_bit,     // The parity bit separated from the data frame
    output logic        start_bit,      // The Start bit separated from the data frame
    output logic        stop_bit,       // The Stop bit separated from the data frame
    output logic        done_flag,      // Indicates that the data is received and ready for another data packet
    output logic [ 7:0] raw_data        // The 8-bit data separated from the data frame
);

    // -Deframing- Output Data & Parity Bit Logic with Asynchronous Reset
	always_comb begin
		if (!reset_n) begin
			// Idle
			raw_data     = {8{1'b1}};
         parity_bit   = 1'b1;
         start_bit    = 1'b0;
         stop_bit     = 1'b1;
         done_flag    = 1'b1;
		end else if (recieved_flag) begin
			start_bit  = data_parll[0];
         raw_data   = data_parll[8:1];
         parity_bit = data_parll[9];
         stop_bit   = data_parll[10];
         done_flag  = 1'b1;
		end else begin
			// Idle
         raw_data   = {8{1'b1}};
         parity_bit = 1'b1;
         start_bit  = 1'b0;
         stop_bit   = 1'b1;
         done_flag  = 1'b0;
		end
	end

endmodule

// =============================================================================
// 7. ERROR CHECKING
// =============================================================================
module ErrorCheck(
    input logic        reset_n,       // Active low reset
    input logic        recieved_flag, // Enable from the SIPO unit for the flags
    input logic        parity_bit,    // The parity bit from the frame for comparison
    input logic        start_bit,     // The Start bit from the frame for comparison
    input logic        stop_bit,      // The Stop bit from the frame for comparison
    input logic [1:0]  parity_type,   // Parity type agreed upon by the Tx and Rx units
    input logic [7:0]  raw_data,      // The 8-bit data separated from the data frame

    output logic [2:0] error_flag     // Bus of three bits for error flags
);

    // Internal signals
    logic error_parity;
    logic parity_flag;
    logic start_flag;
    logic stop_flag;

    // Encoding for the 4 types of parity
   localparam [2:0]
        ODD        = 2'b01,
        EVEN       = 2'b10,
        NOPARITY00 = 2'b00,
        NOPARITY11 = 2'b11;

    // Parity Check Logic
    always_comb begin
        case (parity_type)
            NOPARITY00, NOPARITY11: error_parity = 1'b1;
            ODD:                     error_parity = (^raw_data) ? 1'b0 : 1'b1;
            EVEN:                    error_parity = (^raw_data) ? 1'b1 : 1'b0;
            default:                 error_parity = 1'b1;  // No parity
        endcase
    end

    // Flag logic
    always_comb begin
        if (!reset_n) begin
            parity_flag  = 1'b0;
            start_flag   = 1'b0;
            stop_flag    = 1'b0;
        end else if (recieved_flag) begin
            parity_flag = ~(error_parity && parity_bit);
            start_flag  = (start_bit || 1'b0);
            stop_flag   = ~(stop_bit && 1'b1);
        end else begin
            parity_flag  = 1'b0;
            start_flag   = 1'b0;
            stop_flag    = 1'b0;
        end
    end

    // Output logic
    always_comb begin
        error_flag = {stop_flag, start_flag, parity_flag};
    end

endmodule

// =============================================================================
// 8. FIFO BUFFER
// =============================================================================
module FIFO_Buffer
(
	input              reset,               // Active low reset
                      clk,                // Clock
                      writeEn, 				// Write enable
                      readEn, 				// Read enable
	input      	 [7:0] dataIn, 				// Data written into FIFO
	output logic [2:0] wptr, rptr,
   output logic [7:0] dataOut, 				// Data read from FIFO
   output              		EMPTY, 				// FIFO is empty when high
                           FULL 				// FIFO is full when high
);

  logic [7:0]	fifo[8];
  logic [7:0]  check_fifo_tx;
  
  assign check_fifo_tx = fifo[wptr];

  always @ (posedge clk or negedge reset) begin
    if (!reset) begin
      wptr <= 0;
    end else begin
      if (writeEn & !FULL) begin
        fifo[wptr] <= dataIn;
        wptr <= wptr + 1;
      end
    end
  end
  always @ (posedge clk or negedge reset) begin
    if (!reset) begin
      rptr <= 0;
    end else begin
      if (readEn & !EMPTY) begin
        dataOut <= fifo[rptr];
        rptr <= rptr + 1;
      end
    end
  end

  assign FULL  = ((wptr + 3'd1) == rptr);
  assign EMPTY = wptr == rptr;
endmodule

// =============================================================================
// 9. TX UNIT - COMPLETE TRANSMITTER
// =============================================================================
module TxUnit(
    input  logic        reset_n,       // Active low reset
    input  logic        send,          // Enable to start sending data
    input  logic        clock,         // The main system's clock
    input  logic [1:0]	parity_type,   // Parity type agreed upon by the Tx and Rx units
    input  logic [1:0]	baud_rate,     // Baud rate agreed upon by the Tx and Rx units
    input  logic [7:0]	data_in,       // The data input

    output logic        data_tx,       // Serial transmitter's data out
    output logic        active_flag,   // High when Tx is transmitting, low when idle
	 output logic 			next_state,
	 output logic  		baud_clk_w,	
    output logic        done_flag,      // High when transmission is done, low when active
	 output logic [10:0] frame_man_piso
);
	//  Interconnections
	logic parity_bit_w;
	
	// Internal signals for unused PISO outputs
	logic [3:0] stop_count_int, bit_count_int;
	logic [10:0] frame_r_int;
	logic count_full_int;

	//  Baud generator unit instantiation
	BaudGenTx Unit1(
		//  Inputs
		.reset_n(reset_n),
		.clock(clock),
		.baud_rate(baud_rate),
   
		//  Output
		.baud_clk(baud_clk_w)
	);

	//Parity unit instantiation 
	Parity Unit2(
		//  Inputs
		.reset_n(reset_n),
		.data_in(data_in),
		.parity_type(parity_type),
  
		//  Output
		.parity_bit(parity_bit_w)
	);

	//  PISO shift register unit instantiation
	PISO Unit3(
		//  Inputs
		.reset_n(reset_n),
		.send(send),
		.baud_clk(baud_clk_w),
		.data_in(data_in),
		.parity_bit(parity_bit_w),
		.parity_type(parity_type),
		
		//  Outputs
		.data_tx(data_tx),
		.active_flag(active_flag),
		.next_state(next_state),
		.done_flag(done_flag),
		.frame_man(frame_man_piso),
		.stop_count(stop_count_int),
		.bit_count(bit_count_int),
		.frame_r(frame_r_int),
		.count_full(count_full_int)
	);

endmodule

// =============================================================================
// 10. RX UNIT - COMPLETE RECEIVER
// =============================================================================
module RxUnit(
    input  logic        reset_n,       // Active low reset
    input  logic        data_tx,       // Serial data received from the transmitter
    input  logic        clock,         // The system's main clock
    input  logic [ 1:0]	parity_type,   // Parity type agreed upon by the Tx and Rx units
    input  logic [ 1:0] baud_rate,     // Baud rate agreed upon by the Tx and Rx units

    output logic       	active_flag,   // Logic 1 when data is in progress
    output logic       	done_flag,     // Logic 1 when data is received
    output logic [ 2:0]	error_flag,    // Error flags: [ParityError, StartError, StopError]
    output logic [ 7:0] data_out,       // The 8-bit data separated from the frame
	 output logic       	baud_clk_R,
	 output logic [10:0] Recieved_Frame,
	 input logic rx_enable
);
	//  Intermediate wires
	logic [10:0]	data_parll_w; //  data_out parallel comes from the SIPO unit.
	logic 			recieved_flag_w;     //  works as an enable for deframe unit.
	logic 			def_par_bit_w;       //  The Parity bit from the Deframe unit to the ErrorCheck unit.
	logic 			def_strt_bit_w;      //  The Start bit from the Deframe unit to the ErrorCheck unit.
	logic 			def_stp_bit_w;       //  The Stop bit from the Deframe unit to the ErrorCheck unit.

	assign Recieved_Frame = data_parll_w;
	//  clocking Unit Instance
	BaudGenRx Unit1(
		//  Inputs
		.reset_n(reset_n),
		.clock(clock),
		.baud_rate(baud_rate),

		//  Output
		.baud_clk(baud_clk_R)
	);

	//  Shift Register Unit Instance
	SIPO Unit2(
		//  Inputs
		.reset_n(reset_n),
		.data_tx(data_tx),
		.baud_clk(baud_clk_R),
		.rx_enable(rx_enable),

		//  Outputs
		.active_flag(active_flag),
		.recieved_flag(recieved_flag_w),
		.data_parll(data_parll_w)
	);

	//  DeFramer Unit Instance
	DeFrame Unit3(
		//  Inputs
		.reset_n(reset_n),
		.recieved_flag(recieved_flag_w),
		.data_parll(data_parll_w),
    
		//  Outputs
		.parity_bit(def_par_bit_w),
		.start_bit(def_strt_bit_w),
		.stop_bit(def_stp_bit_w),
		.done_flag(done_flag),
		.raw_data(data_out)
	);

	//  Error Checking Unit Instance
	ErrorCheck Unit4(
		//  Inputs
		.reset_n(reset_n),
		.recieved_flag(done_flag),
		.parity_bit(def_par_bit_w),
		.start_bit(def_strt_bit_w),
		.stop_bit(def_stp_bit_w),
		.parity_type(parity_type),
		.raw_data(data_out),

		//  Output
		.error_flag(error_flag)
	);

endmodule

// =============================================================================
// 11. DUPLEX - FULL DUPLEX UART WITH FIFO
// =============================================================================
module Duplex (
    input  logic        reset_n,        // Active low reset
    input  logic        send,           // Enable to start sending data
    input  logic        clock,          // The main system's clock
    input  logic [ 1:0]	parity_type,    // Parity type agreed upon by the Tx and Rx units
    input  logic [ 1:0]  baud_rate,      // Baud rate agreed upon by the Tx and Rx units
    input  logic [ 7:0]  data_in,        // Data input to be sent
	 input  logic  		RX,
	 input logic rx_enable,

    output logic        tx_active_flag, // Logic 1 when Tx is in progress
    output logic        tx_done_flag,   // Logic 1 when transmission is done
    output logic        rx_active_flag, // Logic 1 when Rx is in progress
    output logic        rx_done_flag,   // Logic 1 when data is received
    output logic  		TX,       // 8-bit data output from the FIFO 
	 output logic [ 7:0]  data_out, 
	 output logic [ 7:0]  fifo_tx_data_out,       // Data output from Tx FIFO to Tx unit
    output logic [ 2:0]  error_flag,      // Error flags: Parity, Start, Stop errors
	 output logic        baud_clk_R, tx_fifo_empty, tx_fifo_full, // Tx FIFO status flags
	 output logic        baud_clk_w,
	 output logic [10:0] Recieved_Frame,
	 output logic 			readEN_ctrl,
	 output logic [ 2:0] 	wptr,rptr,
	 output logic 			tx_start, tx_start_init,
	 output logic [10:0] frame_man_piso,
	 output logic [7:0] fifo_rx_data_in
);

	typedef enum logic [1:0] {
		IDLE      = 2'b00,
		WAIT_DONE = 2'b01,
		READ_ONCE = 2'b10,
		ONE_DATA = 2'b11
	} state_t;

	state_t curr_state, next_state;
  
	 // Internal wires
	logic       data_tx_w;              // Serial transmitter's data out
	logic writeEN_ctrl;
	
	// Internal signals for unused FIFO outputs
	logic [2:0] rx_fifo_wptr, rx_fifo_rptr;
  
   logic       rx_fifo_empty, rx_fifo_full, fifo_empty, tmp_baud_clk_w; // Rx FIFO status flags
	logic 		tx_fifo_read_en;
	logic 		tx_next_state; // Internal signal for TxUnit next_state output
	logic [1:0] state;
	
	assign fifo_empty = tx_fifo_empty;
	
	// Transmitter FIFO
   FIFO_Buffer tx_fifo (
		.clk(clock),
      .reset(reset_n),
      .dataIn(data_in),            // Data input from external source
		.writeEn(send), // Write enable controlled by send and FIFO full flag
		.readEn(readEN_ctrl),
      .dataOut(fifo_tx_data_out), // Data output to Tx unit
      .EMPTY(tx_fifo_empty),
      .FULL(tx_fifo_full),
		.wptr(wptr),
		.rptr(rptr)
    );

	// Receiver FIFO
   FIFO_Buffer rx_fifo (
		.clk(clock),
      .reset(reset_n),
      .dataIn(fifo_rx_data_in),    // Data input from Rx unit
		.writeEn(writeEN_ctrl),
      .readEn(rx_done_flag && tx_done_flag),     // Read when FIFO is not empty
      .dataOut(data_out),          // Data output to external consumer
      .EMPTY(rx_fifo_empty),
      .FULL(rx_fifo_full),
		.wptr(rx_fifo_wptr),
		.rptr(rx_fifo_rptr)
	);

	// Transmitter unit instance
   TxUnit Transmitter (
		.reset_n(reset_n),
		.send(tx_start || tx_start_init),    // Send data only when FIFO has data
      .clock(clock),
      .parity_type(parity_type),
      .baud_rate(baud_rate),
      .data_in(fifo_tx_data_out),     // Data input from Tx FIFO
      .data_tx(TX),            // Serial output
      .active_flag(tx_active_flag),
		.baud_clk_w(baud_clk_w),
      .done_flag(tx_done_flag),
		.next_state(tx_next_state),
		.frame_man_piso(frame_man_piso)
	);

	// Receiver unit instance
   RxUnit Receiver (
		.reset_n(reset_n),
      .clock(clock),
      .parity_type(parity_type),
      .baud_rate(baud_rate),
      .data_tx(RX),            // Serial data from Tx unit
      .data_out(fifo_rx_data_in),     // Data output to Rx FIFO
      .error_flag(error_flag),
      .active_flag(rx_active_flag),
      .done_flag(rx_done_flag),
		.baud_clk_R(baud_clk_R),
		.Recieved_Frame(Recieved_Frame),
		.rx_enable(rx_enable)
	);

	always_ff @(posedge baud_clk_w) begin
		if (!tx_fifo_empty) begin
			tx_start <= 1;
		end else
			tx_start <= 0;
	end

	logic initial_data, tmp_initial_data;
	
	// State register
	always_ff @(posedge clock or negedge reset_n) begin
		if (!reset_n) begin
			curr_state <= IDLE;
			initial_data <= 0;
		end else begin
			curr_state <= next_state;
			initial_data <= tmp_initial_data;
		end
	end

	// Next-state logic & outputs
	assign tmp_baud_clk_w = baud_clk_w;

	always_comb begin
		// Default assignments
		next_state = curr_state;
		readEN_ctrl = 0;
		writeEN_ctrl = 0;
		tmp_initial_data = initial_data;
		tx_start_init = 1'b0;
		case (curr_state)
			IDLE: begin
				
				if (~tx_fifo_empty && ~initial_data) begin
					readEN_ctrl = 1;
					tmp_initial_data = 1;
				end
			
				if (!baud_clk_w && ~tx_fifo_empty && ~initial_data) begin
					next_state = ONE_DATA;
				end
			
				if (tx_active_flag) begin
					// Wait for active=1
					next_state = WAIT_DONE;			 
				end 
			end
	
			ONE_DATA: begin 
				tx_start_init = 1'b1;
				if (baud_clk_w) begin
					tx_start_init = 1'b0;
					next_state = IDLE;
				end else 
					next_state = ONE_DATA;
			end 
			
			WAIT_DONE: begin
				// Wait for done=1
				if (tx_done_flag) begin
					next_state = READ_ONCE;
				end
			end
	
			READ_ONCE: begin
				if (!tx_active_flag) begin
					// Assert readEn for 1 cycle
					readEN_ctrl  = 1'b1;
					writeEN_ctrl = 1'b1;
				end
					next_state = IDLE;
				end
	
			default: begin
				// Safe default (should never happen)
				next_state = IDLE;
			end
		endcase
	end

endmodule

// =============================================================================
// 12. UART APB COMPLETE - TOP LEVEL MODULE
// =============================================================================
module uart_apb_complete #(
    parameter logic[31:0] BaseAddr = 32'h30000000,  // APB Base Address
    parameter int NumWords = 16                   // Register space size
)(
    // APB Interface
    input  logic         p_clk,
    input  logic         p_reset_n,
    input  logic [31:0]  p_addr,
    input  logic         p_sel,
    input  logic         p_enable,
    input  logic         p_write,
    input  logic [31:0]  p_wdata,
    output logic [31:0]  p_rdata,
    output logic         p_ready,
    
    // UART Physical Interface
    input  logic         uart_rx,
    output logic         uart_tx,
    
    // Debug/Status Outputs
    output logic         uart_tx_active,
    output logic         uart_rx_active,
    output logic         uart_tx_done,
    output logic         uart_rx_done,
    output logic [2:0]   uart_error_flags,
    output logic [7:0]   uart_rx_data,
    output logic [7:0]   uart_tx_data,
    output logic [1:0]   uart_baud_rate,
    output logic [1:0]   uart_parity_type
);

    // =========================================================================
    // Internal Signals
    // =========================================================================
    
    // Configuration Registers
    logic [31:0] enable_reg;    // [1:0] = {rx_enable, tx_enable}
    logic [31:0] control_reg;   // [3:0] = {baud_rate[1:0], parity_type[1:0]}
    logic [31:0] status_reg;    // Status readback
    logic [31:0] data_reg;      // Data register
    
    // UART Control Signals
    logic rx_enable, tx_enable;
    logic uart_send;
    logic [1:0] baud_rate;
    logic [1:0] parity_type;
    
    // UART Data Signals
    logic [7:0] tx_data_internal;
    logic [7:0] rx_data_internal;
    logic [2:0] error_flag_internal;
    
    // APB Address Decode
    localparam AddrWidth = $clog2(NumWords * 4);
    logic[31:AddrWidth] addr_tag;
    logic[AddrWidth-1:2] word_offset;
    logic[1:0] byte_offset;
    logic addr_valid, addr_in_range, addr_aligned;
    logic write_en, read_en;
    
    // =========================================================================
    // APB Address Decode
    // =========================================================================
    
    assign {addr_tag, word_offset, byte_offset} = p_addr;
    assign addr_in_range = (addr_tag == BaseAddr[31:AddrWidth]);
    assign addr_aligned = (byte_offset == '0);
    assign addr_valid = addr_in_range & addr_aligned;
    
    assign write_en = p_sel & p_enable & p_write & addr_valid;
    assign read_en = p_sel & p_enable & ~p_write & addr_valid;
    assign p_ready = p_sel & p_enable;
    
    // =========================================================================
    // Register Map Implementation
    // =========================================================================
    
    // Register Write Logic
    always_ff @(posedge p_clk or negedge p_reset_n) begin
        if (!p_reset_n) begin
            enable_reg <= 32'h0;
            control_reg <= 32'h0;
            uart_send <= 1'b0;
            tx_data_internal <= 8'h0;
        end else begin
            // Default: clear send pulse
            uart_send <= 1'b0;
            
            if (write_en) begin
                case (p_addr[7:0])
                    8'h00: begin // Enable Register
                        enable_reg <= p_wdata;
                    end
                    8'h04: begin // Control Register  
                        control_reg <= p_wdata;
                    end
                    8'h0C: begin // Data Register (TX)
                        if (tx_enable && !uart_tx_active) begin
                            tx_data_internal <= p_wdata[7:0];
                            uart_send <= 1'b1; // Single cycle pulse
                        end
                    end
                endcase
            end
            
            // Auto-clear send after one cycle
            if (uart_send && uart_tx_done) begin
                uart_send <= 1'b0;
            end
        end
    end
    
    // Extract control signals
    assign rx_enable = enable_reg[1];
    assign tx_enable = enable_reg[0];
    assign baud_rate = control_reg[3:2];
    assign parity_type = control_reg[1:0];
    
    // Status Register Construction
    always_comb begin
        status_reg = {
            24'h0,                    // [31:8] Reserved
            uart_tx_active,           // [7] TX Active
            uart_rx_active,           // [6] RX Active  
            error_flag_internal[2],   // [5] Stop bit error
            error_flag_internal[1],   // [4] Start bit error
            error_flag_internal[0],   // [3] Parity error
            1'b0,                     // [2] Reserved
            uart_tx_done,             // [1] TX Done
            uart_rx_done              // [0] RX Done
        };
    end
    
    // Register Read Logic
    always_comb begin
        p_rdata = 32'h0;
        if (read_en) begin
            case (p_addr[7:0])
                8'h00: p_rdata = enable_reg;                    // Enable
                8'h04: p_rdata = control_reg;                   // Control  
                8'h08: p_rdata = status_reg;                    // Status
                8'h0C: p_rdata = {24'h0, rx_data_internal};     // RX Data
                default: p_rdata = 32'h0;
            endcase
        end
    end
    
    // =========================================================================
    // UART Core Instance - Full Duplex
    // =========================================================================
    
    Duplex uart_core (
        .reset_n(p_reset_n),
        .send(uart_send),
        .clock(p_clk),
        .parity_type(parity_type),
        .baud_rate(baud_rate),
        .data_in(tx_data_internal),
        .RX(uart_rx),
        .rx_enable(rx_enable),
        
        // Outputs
        .tx_active_flag(uart_tx_active),
        .tx_done_flag(uart_tx_done),
        .rx_active_flag(uart_rx_active),
        .rx_done_flag(uart_rx_done),
        .TX(uart_tx),
        .data_out(rx_data_internal),
        .error_flag(error_flag_internal),
        
        // Debug outputs (can be left unconnected if not needed)
        .baud_clk_R(),
        .tx_fifo_empty(),
        .tx_fifo_full(),
        .baud_clk_w(),
        .Recieved_Frame(),
        .readEN_ctrl(),
        .wptr(),
        .rptr(),
        .tx_start(),
        .tx_start_init(),
        .frame_man_piso(),
        .fifo_tx_data_out(),
        .fifo_rx_data_in()
    );
    
    // =========================================================================
    // Debug/Status Output Assignments
    // =========================================================================
    
    assign uart_error_flags = error_flag_internal;
    assign uart_rx_data = rx_data_internal;
    assign uart_tx_data = tx_data_internal;
    assign uart_baud_rate = baud_rate;
    assign uart_parity_type = parity_type;

endmodule

// =============================================================================
// END OF COMPLETE UART SYSTEM
// =============================================================================