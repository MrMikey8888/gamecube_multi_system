#define CONSOLE 4


// these cannot be changed without alterting the code, just declare them here so they are knowen
// make sure these are all on the same port on your board
#define PORTCONDITION 12 // status of the data is in portdata TODO: work out a better pin, if memory is a problem put in same buss
#define PORTDATA1 8 // data being transfered
#define PORTDATA2 9 // data being transfered
#define PORTDATA3 10 // data being transfered
#define PORTDATA4 11 // data being transfered

void setup() {
  // put your setup code here, to run once:
  

  uint8_t setup_data[19];
  setup_data[16] = digitalPinToBitMask(CONSOLE);
  uint8_t portConsole = digitalPinToPort(CONSOLE);
  uint16_t temp = portModeRegister(portConsole);
  setup_data[0]  = temp; // modePortConsole_lower
  setup_data[1] = temp >> 8; // modePortConsole_upper


  temp = portOutputRegister(portConsole);
  setup_data[2] = temp; // outPortConsole_lower
  setup_data[3] = temp >> 8; // outPortConsole_upper

  temp = portInputRegister(portConsole);
  setup_data[4] = temp; // inPortConsole_lower
  setup_data[5] = temp >> 8; // inPortConsole_upper


  uint8_t portData = digitalPinToPort(PORTCONDITION);
  setup_data[17] = 0x10; // declared like this so that if it is changed you will have to change it manually
  setup_data[18] = 0x0F; // declared like this insted of writing it manually so if you want to change it you can find where it is used

  
  temp = portInputRegister(portData);
  setup_data[6] = temp; // inPortComm_lower
  setup_data[7] = temp >> 8; // inPortComm_upper
  
  // set information pins to input
  *portModeRegister(portData) &= ~setup_data[17];
  *portModeRegister(portData) &= ~setup_data[18];

  // enable internel pullup resistor
  *portOutputRegister(portData) |= setup_data[17];
  *portOutputRegister(portData) |= setup_data[18];

  uint8_t command[3] = {0x00, 0x00, 0x00};
  temp = &command;
  setup_data[8] = temp; // command_lower
  setup_data[9] = temp >> 8; // command_upper

  uint8_t contoller_init[3] = {0x09, 0x00, 0x03};
  temp = &contoller_init;
  setup_data[10] = temp; // contoller_init_lower
  setup_data[11] = temp >> 8; // contoller_init_upper

  // we set the upper 2 bytes to be correct here, i cba to do it in the asm block
  uint8_t initaldata[10] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x02};
  temp = &initaldata;
  setup_data[12] = temp; // initaldata_lower
  setup_data[13] = temp >> 8; // initaldata_upper

  uint8_t buffer[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  temp = &buffer;
  setup_data[14] = temp; // buffer_lower
  setup_data[15] = temp >> 8; // buffer_upper

  // make into a list
  // load registers when entering the asm


  // we dont want to make any vairables here since we going to write everything in asm 
  // we only compile this way since i dont know how to get the memory adress of the ports in asm
  asm volatile (
    "; main program\n"
    // to initalise we want to set the gamecube port to input 
  
  
    // initalise all the data to be accesses quickly
    "ld r0, X+\n" // mode port
    "ld r1, X+\n" 
    "ld r2, X+\n" // out console
    "ld r3, X+\n"
    "ld r4, X+\n" // in console
    "ld r5, X+\n"
    "ld r6, X+\n" // im comm port
    "ld r7, X+\n"
    "ld r8, X+\n" // command read
    "ld r9, X+\n" 
    "ld r10, X+\n" // initalisation for contoller
    "ld r11, X+\n"
    "ld r12, X+\n" // inital data
    "ld r13, X+\n"
    "ld r14, X+\n" // buffer pointer
    "ld r15, X+\n"
    "ld r16, X+\n" // bitmask console
    "ld r17, X+\n" // bitmask status
    "ld r18, X+\n" // bitmask data

    "mov r26,r6\n"
    "mov r27,r7\n"
    "mov r30, r12\n"
    "mov r31, r13\n"
    "ldi r19, 0x08\n"
    "rjmp .L%=read_stream_ensure_read\n"

  // read and send loop: no inputs
    ".L%=read_console_loop:\n"
    // first thing to do is to read data

    
    "ldi r19, 0x03\n" // max command length is 3
    "mov r26,r4\n" // (1) x
    "mov r27,r5\n" // (1) set the register inport console
    "mov r30, r8\n" // z 
    "mov r31, r9\n" // load the register for command

    "ldi r24,0x00\n" // yet to get a byte
    "ldi r23,0x08\n" // 8 bits to read
    
    ".L%=read_init_loop:\n" // no timeout since if we dont get a console comand then nothing else to do
    "ld r25, X\n" // (2) reads pin then saves to input value 
    "and r25, r16\n" // (1) compares the input value to the bitmask
    "breq .L%=read_found_low\n" // (1/2) jumps if the value is 0 from the and (the result was low)
    "rjmp .L%=read_init_loop\n" // (2)



    ".L%=read_wait_for_low:\n"
    "ld r25, x\n" // (2) reads pin then saves to input value 
    "and r25, r16\n" // (1) compares the input value to the bitmask
    "breq .L%=read_found_low\n" // (1/2) jumps if the value is 0 from the and (the result was low)
    "dec r22\n" // (1) 
    "brne .L%=read_wait_for_low\n" // (1/2) loop if the counter isn't 0
    // timeout if the loop did not find low
    "rjmp .L%=read_program_exit\n" // (2) jump to end

    ".L%=read_found_low:\n"
    // want to read between the 12 and the 20th cpu cycle of the 2us.
    "nop\nnop\nnop\nnop\nnop\n" //(5)
    "nop\nnop\nnop\nnop\nnop\n" //(5) (10)
    "nop\nnop\nnop\nnop\nnop\n" //(5) (15)
    "nop\nnop\nnop\nnop\nnop\n" //(5) (20)
    "nop\n" //(1) (21)
    "lsl r21\n" // (1) moves data recived left so the new data can fit in 
    "ldi r22,0x10\n" // (1) (23) set the timeout to 16 loops


    // logic to check what the bit is. 
    ".L%=read_read_bit:\n"
    "ld r25, X\n" // (2) reads pin then saves to input value (happens before the 2 cycles)
    "and r25, r16\n" // (1) compares the input value to the bitmask
    "breq .L%=read_decrement_bit\n" // (1/2) jumps if the value is 0 from the and (the result was low)
    "sbr r21,0x01\n" // (1) sets the first bit in the data to be a 1
    ".L%=read_decrement_bit:\n"
    "dec r23\n" // (1) decrement 1 from our bit counter
    "brne .L%=read_wait_for_high\n" // (1/2) branch if we've not received the full byte

    "inc r24\n" // (1) increase byte count
    "ldi r23,0x08\n" // (1) set bitcount to 8 bits
    "st Z+,r21\n" // (2) save %[data] back to memory and increment byte pointer
    "cp r19,r24\n" // (1) %[len] == %[receivedBytes] ?
    "breq .L%=read_stop_bit\n" // (1/2) jump to end of the data

    ".L%=read_wait_for_high:\n"
    "ld r25, X\n" // (2) read the pin
    "and r25, r16\n" // (1) compare pinstate with bitmask
    "brne .L%=read_wait_for_low\n" // (1/2) if the line is high then jump
    "dec r22\n" // (1) decrease timeout by 1
    "brne .L%=read_wait_for_high\n" // (1/2) loop if the counter isn't 0
    //"ldi %[receivedBytes],0x50\n"
    ".L%=read_stop_bit:\n"
    // want to wait for the last bit, to do this we get here with at most 21 bits in this byte
    // need to do logic for command type

    // we have to use more then 37 instructions to modify the data report
    // start by loading the buffer into the 
    "mov r30, r8\n" // (1) z 
    "mov r31, r9\n" // (1) load the register for command
    "ld r25, Z+\n" // (2) dump
    "ld r19, Z\n" // (2) load the data format we need to make into r19

    // this is technially not needed for some modifications but i need to use some cpu cycles
    "mov r26, r14\n" // (1) X
    "mov r27, r15\n" // (1) load the register for command
    "mov r30, r14\n" // (1) Z 
    "mov r31, r15\n" // (1) load the register for command

    // we need to modify at least [4] (in the x) and [5] in the Z
    "ld r25, Z+\n" // (2) Z = Z[1]
    "ld r25, Z+\n" // (2) Z = Z[2]
    "ld r25, Z+\n" // (2) Z = Z[3]
    "ld r25, Z+\n" // (2) Z = Z[4]
    "ld r25, Z+\n" // (2) Z = Z[5]

    "ld r25, X+\n" // (2) X = X[1]
    "ld r25, X+\n" // (2) X = X[2]
    "ld r25, X+\n" // (2) X = X[3]
    "ld r25, X+\n" // (2) X = X[4]


    "cpi r19,0x03\n" // (1) comand == 3 ?
    "breq .L%=read_no_change\n" // (1/2) jump if true

    "cpi r19,0x01\n" // (1) comand == 1 ?
    "breq .L%=read_format_one\n" // (1/2) jump if true


    // set the x regiuster to be the buffer
    "mov r30, r14\n" // (1) z 
    "mov r31, r15\n" // (1) load the register for command

    "cp %[len],%[receivedBytes]\n" // (1) %[len] == %[receivedBytes] ?
    "breq .L%=_stop_bit\n" // (1/2) jump if true


    "mov r26,r6\n" // (1) x
    "mov r27,r7\n" // (1) set the register modeport

    



    ".L%=read_no_change:\n" // we have 31/37 cpu cycles done
    "ldi r19, 0x01\n" // (1) indicate that we need to get new data
    // set the data buffer
    "mov r30,r14\n" // (1) Z
    "mov r31,r15\n" // (1) set as the data buffer
    "rjmp .L%=send_data_setup\n"

    ".L%=read_format_one:\n" // we have 31/37 cpu cycles done
    "ld r25, X\n" // (2) r25 = data[4]
    "ld r24, Z+\n" // (2) r24 = data[5], Z = data[6]
    "lsr r25\n" // (1) shift right by 1
    "lsr r25\n" // (1) shift right by 1 [2]
    "lsr r25\n" // (1) shift right by 1 [3]
    "lsr r25\n" // (1) shift right by 1 [4]
    "andi r24, 0xF0\n" // (1) limit data[5] to be only the top 4 bits
    "or r25, r24\n"

    // set in X, increment X
    // copy Z to 25, increment Z
    // set X to 25, increment x
    // copy Z to 25, increment Z
    // set X to 25, increment x
    // set x to 0
    // set Z and r19


    "ldi r19, 0x01\n" // (1) indicate that we need to get new data
    // set the data buffer
    "mov r30,r14\n" // (1) Z
    "mov r31,r15\n" // (1) set as the data buffer
    "rjmp .L%=send_data_setup\n"

  

    // need to load X as the inport, Z as the buffer to send

    // should be high or turning high rn, 
    // we return the number of bits filled 
    ".L%=read_program_exit:\n"
    // if len = 1 then logic

    
  


  ".L%=send_data_setup:\n"
  // total of 16 instructions, get ready for the read
  // r20 is low, 21 is high, x is inport console
    // need to set port as an outport
    "mov r26,r0\n" // (1) x
    "mov r27,r1\n" // (1) set the register modeport
    "ld r25, X\n" // (2) load portmode into r25
    "or r25, r16\n" // (1) determin the new portmode
    "st X, r25\n" // (2) store the bitmask

    // need to store low in 20
    "mov r26,r6\n" // (1) x
    "mov r27,r7\n" // (1) set as the outport console
    "ld r20, X\n" // (2) load portmode into r25
    "com r20\n" // (1) bitwise invers of bitmask
    "and r20, r16\n" // (1) r20 is the port low 

    // need to store high in r21
    "ld r21, X\n" // (2) load outport console into r21
    "or r21, r16\n" // (1) r21 is the port high
  // end code hide

    ".L%=send_data:\n" // start the function for readint he stream TODO: implementation
  // inputs: r19 read_new, r20: low, r20: high, X = outport, z = buffer. port set to input

    
    // set the port mode to input
  // hide TODO: clobbers


    ".L%=read_data:\n"
  // input: 
      // we need to set the port to input (9)
    "mov r26,r0\n" // (1) x
    "mov r27,r1\n" // (1) set the register modeport
    "mov r16,r25\n" // (1) set the register 25 to be the bitmask for console
    "com r25\n" // (1) bitwise invers of bitmask
    "ld r24, X\n" // (2) load x into r24
    "and r25, r24\n" // (1) determin the new portmode
    "st X, r25\n" // (2) store the bitmask

    // set the X to be the inport
    "mov r26, r4\n" // (1) X 
    "mov r27, r5\n" // (1) load the register for inport console



  // clobbers



    ".L%=read_stream_ensure_read:\n" // start the function for readint he stream
  // read stream data, needs r19 as length, X as the inport, Z as the buffer
    // **logic to sunc up with the start of the data, not using timeouts**
    // used when there is not time sensitive code running

    ".L%=read_stream_init_wait_low_loop_e:\n"
    "ld r25,X\n" // (2) reads pin then saves to input value 
    "and r25, r17\n" // (1) compares the input value to the bitmask
    "breq .L%=read_stream_init_wait_high_e\n" // (1/2) jumps if the value is 0 from the and (the result was low)
    "rjmp .L%=read_stream_init_wait_low_loop_e\n"    


    ".L%=read_stream_init_wait_high_e:\n"
    ".L%=read_stream_init_wait_high_loop_e:\n"
    "ld r25,X\n" // (2) reads pin then saves to input value 
    "and r25, r17\n" // (1) compares the input value to the bitmask
    "brne .L%=read_stream_start_reading\n" // (1/2) jumps if the value is 0 from the and (the result was low)
    "rjmp .L%=read_stream_init_wait_high_loop_e\n"
  // clobbers



    ".L%=read_stream:\n" // start the function for readint he stream
  // Inputs:
    "ldi r24,0x64\n" // (1)

    // **logic to sunc up with the start of the data**

    ".L%=read_stream_init_wait_low_loop:\n"
    "ld r25,X\n" // (2) reads pin then saves to input value 
    "and r25, r17\n" // (1) compares the input value to the bitmask
    "breq .L%=read_stream_init_wait_high\n" // (1/2) jumps if the value is 0 from the and (the result was low)
    "dec r24\n" // (1)
    "brne .L%=read_stream_init_wait_low_loop\n" // (1/2) jumps if not 0
    "rjmp .L%=read_stream_exit\n"
    
    ".L%=read_stream_init_wait_high:\n"
    "ldi r24,0x10\n" // (1)
    ".L%=read_stream_init_wait_high_loop:\n"
    "ld r25,X\n" // (2) reads pin then saves to input value 
    "and r25, r17\n" // (1) compares the input value to the bitmask
    "brne .L%=read_stream_start_reading\n" // (1/2) jumps if the value is 0 from the and (the result was low)
    "dec r24\n" // (1)
    "brne .L%=read_stream_init_wait_high_loop\n" // (1/2) jumps if not 0
    "rjmp .L%=read_stream_exit\n"


    // **Logic to read the data*


    ".L%=read_stream_wait_prebyte:\n"
    "ldi r24,0x10\n" /// (1)
    // logic to wait for line to go low
    ".L%=read_stream_wait_high_prebyte:\n"
    "ld r25, X\n" // (2) reads pin then saves to input value 
    "andi r25, 0x01\n" // (1) compares the input value to the bitmask (all the bits get set to low then high when between data)
    "brne .L%=read_stream_wait_low_prebyte\n" // (1/2) jumps if the value is not 0 from the and (the result was high)
    "dec r24\n" // (1) decrements timeout 
    "brne .L%=read_stream_wait_high_prebyte\n" // (1/2) loop if the timeout is not 0
    "rjmp .L%=read_stream_exit\n" // (2) 
    
    // logic to wait for line to go high 
    ".L%=read_stream_wait_low_prebyte:\n"
    "ld r25, X\n" // (2) reads pin then saves to input value 
    "andi r25, 0x01\n" // (1) compares the input value to the bitmask (all the bits get set to low then high when between data)
    "breq .L%=read_stream_bits_1\n" // (1/2) jumps if the value is 0 from the and (the result was low)
    "dec r24\n" // (1) decrements timeout 
    "brne .L%=read_stream_wait_low_prebyte\n" // (1/2) loop if the timeout is not 0
    "rjmp .L%=read_stream_exit\n" // (2)

    
    ".L%=read_stream_bits_1:\n"
    // TODO: add nops

    "nop\nnop\nnop\nnop\nnop\n" //(5) 
    "nop\nnop\nnop\nnop\nnop\n" //(5) (10)
    "nop\nnop\nnop\nnop\nnop\n" //(2) (15)
    "nop\n" // (1) (16)


    ".L%=read_stream_start_reading:\n"
    // worst case we are 7 cycles into the data when we read it and it took 5 cycles to get here 
    // since the data is 16 us long and we are between 5 and 12 instructios into it, this is the time
    // to read it
    "ld r23, X\n" // (2)

    ".L%=read_stream_wait_midbyte:\n"
    "ldi r24,0x10\n" // (1)
    // need to wait for line to go 0
    ".L%=read_stream_wait_high_loop_midbyte:\n"
    "ld r25, X\n" // (2) reads pin then saves to input value 
    "andi r25, 0x01\n" // (1) compares the input value to the bitmask
    "brne .L%=read_stream_wait_low_loop_midbyte\n" // (1/2) jumps if the value is not 0 from the and (the result was high)
    "dec r24\n" // (1) decrements timeout 
    "brne .L%=read_stream_wait_high_loop_midbyte\n" // (1/2) loop if the timeout is not 0
    "rjmp .L%=read_stream_exit\n" // (2)


    ".L%=read_stream_wait_low_loop_midbyte:\n"
    "ld r25, X\n" // (2) reads pin then saves to input value 
    "andi r25, 0x01\n" // (1) compares the input value to the bitmask
    "breq .L%=read_stream_bits_2\n" // (1/2) jumps if the value is 0 from the and (the result was low)
    "dec r24\n" // (1) decrements timeout 
    "brne .L%=read_stream_wait_low_loop_midbyte\n" // (1/2) loop if the timeout is not 0
    "rjmp .L%=read_stream_exit\n" // (2)
    
    ".L%=read_stream_bits_2:\n"
    // we are between 5 and 12 nops into the high bit, it is 16long and we want to read 
    // between the 5th and 12th cycle of the data bit, this means that we need 16 cycles
    "lsl r23\n" // (1)
    "lsl r23\n" // (1) (2)
    "lsl r23\n" // (1) (3)
    "lsl r23\n" // (1) (4)
    "nop\nnop\nnop\nnop\nnop\n" //(5) (9)
    "nop\nnop\nnop\nnop\nnop\n" //(5) (14)
    "nop\nnop\n" // (4) (16)


    "ld r22, X\n" // (2)
    "and r22, r18\n" // (1)
    "or r23, r22\n" // (1)
    "st Z+,r23\n" // (2) save %[data] back to memory and increment byte pointer
    "dec r19\n" // (1) 
    "breq .L%=read_stream_exit\n" // (1/2) loop if the counter isn't 0
    "rjmp .L%=read_stream_wait_prebyte\n" // (2)


    ".L%=read_stream_exit:\n"
    "rjmp .L%=read_console_loop\n"
  // clobbers r22 r23, r24. r25.


    // outputs
    : 
    //: [bitMaskConsole] "d" (bitMaskConsole), // bitmask for the console
    //[bitMaskStatus] "d" (bitMaskStatus),
    //[bitMaskData] "d" (bitMaskData),
    : [setup_data] "X" (setup_data)
    //[modePortConsole_lower] "l" (modePortConsole_lower), // 0
    //[modePortConsole_upper] "l" (modePortConsole_upper), // 1
    //[outPortConsole_lower] "l" (outPortConsole_lower), // 2
    //[outPortConsole_upper] "l" (outPortConsole_upper), // 3
    //[inPortConsole_lower] "l" (inPortConsole_lower), // 4
    //[inPortConsole_upper] "l" (inPortConsole_upper), // 5
    //[inPortComm_lower] "l" (inPortComm_lower), // 6
    //[inPortComm_upper] "l" (inPortComm_upper), // 7
    //[command_lower] "l" (command_lower), // 8
    //[command_upper] "l" (command_upper), // 9
    //[contoller_init_lower] "l" (contoller_init_lower), // 10 
    //[contoller_init_upper] "l" (contoller_init_upper), // 11
    //[initaldata_lower] "l" (initaldata_lower), // 12
    //[initaldata_upper] "l" (initaldata_upper), // 13
    //[buffer_lower] "l" (buffer_lower), // 14
    //[buffer_upper] "l" (buffer_lower) // 15
    : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r10", "r11", "r12", "r13", "r14", "r15", "r16", "r17", "r18", "r19", "r24", "r25", "r30", "r31"
  );
}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println("test");
}


//register uint8_t low_byte = static_cast<uint8_t>(pointerValue);
//    register uint8_t high_byte = static_cast<uint8_t>(pointerValue >> 8);