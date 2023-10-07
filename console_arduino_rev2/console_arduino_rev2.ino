// If you wish to use different ports then you need to ensure that the portdata 1 -> 4 are all on the same buss and are the lower 4 bits
// if you ish to use the upper 4 bits then you have a bit of editing to do, i have labled eveythng i have done to the best of my ability so
// i hope this is understandabe
//
// I will include some basic sudocode at the end of the file to provide a quick overview of the assembly, i was having problems with timings and
// the code that i did not write in assmebly taking to long (i tested reading the data and seeing how closly i can send two get requests and timed the delay)
// matmamatically the assembly wont xceed the delay needed so either the compiler was slow (i hope not) or the compiler geting readdy fro inline assembly takes ages
// (ages being a coupl us), hence i figgured thata i can just do it myself

// i do not use the y register in the asembly since compiler did not like me passing it in
// (i assume that is bc it is a link to the neviroment vairables that are moved to ram while the inline asembly runs)

// i know that writing code on a more powerfuk hardwhere may be cheaper when you scale above a point, i was thinking of redoing this on a pi when i get one
// it should not be too hard to translate and depending on th enumber of gpio pins on it i may be able to hook up 16 consles to one pi.
// if this is something you are intrested in let me know and i may start on it sooner, i have some other projects i want to work on like analising the gamecube link cable
// and working out how the data is transfered from that to the pokemon box ruby and sapphire. If this works i may be able to use a similar program on the
// pi to make the startup of multiple saves faster. Eh thats beyon the scope of the project but please let me know if this is somrthing you want

// another thing i wan to look into if i get the pi working is i may setup a program to swap what console your input is going to, if i do this i can see this having uses for
// more then pokemon


// if you think any of the work i have done infringes on anyone elses ip then let me know and ill rectify the situation. I do not suport piracy and this is designed to only work with offical contollers
// but i may make some ports from the offical contoller to a compyter soon to play pirated games on the pc, idk tho

// thoughts if i can work out how to put images on a gamecube if i can setup a pc to play the emulator on the gamecube??
// smth i need to look into :)


// - gba video playing
// - gba output buttons
// pi multi system
// pi input swap

// can i make a basic interface in the pi that allows button mapping and hdmi out through use of a keyboard?
// this will be useful for loading scripts and allowing different profiles to be used for different games

// want to try make this work on a pi pico
// less registers but better io so we can have 16 cubes running to one pico

#define CONSOLE 4



// these cannot be changed without alterting the code, just declare them here so they are knowen
#define PORTCONDITION 12  // status of the data is in portdata TODO: work out a better pin, if memory is a problem put in same buss
#define PORTDATA1 8       // data being transfered
#define PORTDATA2 9       // data being transfered
#define PORTDATA3 10      // data being transfered
#define PORTDATA4 11      // data being transfered

void setup() {
  // put your setup code here, to run once:

  uint8_t setup_data[19];
  setup_data[16] = digitalPinToBitMask(CONSOLE);
  uint8_t portConsole = digitalPinToPort(CONSOLE);
  uint16_t temp = portModeRegister(portConsole);
  setup_data[0] = temp;       // modePortConsole_lower
  setup_data[1] = temp >> 8;  // modePortConsole_upper


  temp = portOutputRegister(portConsole);
  setup_data[2] = temp;       // outPortConsole_lower
  setup_data[3] = temp >> 8;  // outPortConsole_upper

  temp = portInputRegister(portConsole);
  setup_data[4] = temp;       // inPortConsole_lower
  setup_data[5] = temp >> 8;  // inPortConsole_upper


  uint8_t portData = digitalPinToPort(PORTCONDITION);
  setup_data[17] = 0x10;  // declared like this so that if it is changed you will have to change it manually
  setup_data[18] = 0x0F;  // declared like this insted of writing it manually so if you want to change it you can find where it is used


  temp = portInputRegister(portData);
  setup_data[6] = temp;       // inPortComm_lower
  setup_data[7] = temp >> 8;  // inPortComm_upper

  // set information pins to input
  *portModeRegister(portData) &= ~setup_data[17];
  *portModeRegister(portData) &= ~setup_data[18];

  // enable internel pullup resistor
  *portOutputRegister(portData) |= setup_data[17];
  *portOutputRegister(portData) |= setup_data[18];

  uint8_t command[3] = { 0x00, 0x00, 0x00 };
  temp = &command;
  setup_data[8] = temp;       // command_lower
  setup_data[9] = temp >> 8;  // command_upper

  uint8_t contoller_init[3] = { 0x09, 0x00, 0x03 };
  temp = &contoller_init;
  setup_data[10] = temp;       // contoller_init_lower
  setup_data[11] = temp >> 8;  // contoller_init_upper

  // we set the upper 2 bytes to be correct here, i cba to do it in the asm block
  uint8_t initaldata[10] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x02 };
  temp = &initaldata;
  setup_data[12] = temp;       // initaldata_lower
  setup_data[13] = temp >> 8;  // initaldata_upper

  uint8_t buffer[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  temp = &buffer;
  setup_data[14] = temp;       // buffer_lower
  setup_data[15] = temp >> 8;  // buffer_upper

  // make into a list
  // load registers when entering the asm

  // we dont want to make any vairables here since we going to write everything in asm
  // we only compile this way since i dont know how to get the memory adress of the ports in asm
  asm volatile(  // initalise the portode to input and the port status to high
    "; main program\n"
    // to initalise we want to set the gamecube port to input


    // initalise all the data to be accesses quickly
    "ld r0, X+\n"  // mode port
    "ld r1, X+\n"
    "ld r2, X+\n"  // out console
    "ld r3, X+\n"
    "ld r4, X+\n"  // in console
    "ld r5, X+\n"
    "ld r6, X+\n"  // im comm port
    "ld r7, X+\n"
    "ld r8, X+\n"  // command read
    "ld r9, X+\n"
    "ld r10, X+\n"  // initalisation for contoller
    "ld r11, X+\n"
    "ld r12, X+\n"  // inital data
    "ld r13, X+\n"
    "ld r14, X+\n"  // buffer pointer
    "ld r15, X+\n"
    "ld r16, X+\n"  // bitmask console
    "ld r17, X+\n"  // bitmask status
    "ld r18, X+\n"  // bitmask data

    // set the portmode for the input
    "mov r30,r0\n"    // (1) Z
    "mov r31,r1\n"    // (1) set the register modeport
    "ld r25, Z\n"     // (2) load portmode into r25
    "mov r16, r24\n"  // (1)
    "com r24\n"       // (1)
    "and r25, r24\n"  // (1) determin the new portmode
    "st Z, r25\n"     // (2) store the portmode for the input

    // sets the pullup resistor
    "mov r26,r6\n"   // (1) x
    "mov r27,r7\n"   // (1) set as the outport console
    "ld r21, X\n"    // (2) load outport console into r21
    "or r21, r16\n"  // (1) r21 is the port high
    "st X, r21\n"     // (2) store the portmode for the input


    "mov r26,r6\n"                        // X
    "mov r27,r7\n"                        // In port com
    "mov r30, r12\n"                      // Z
    "mov r31, r13\n"                      // inital data
    "ldi r19, 0x08\n"                     // read 8 bytes
    "rjmp .L%=read_stream_ensure_read\n"  // ensure that we read the data

    // read and send loop: no inputs
    ".L%=read_console_loop:\n"
    // inputs nothing
      // r19 is max len
      // r21 is data
      // r22 is the loop timeout
      // r23 is bitcount
      // r24 is bytecount
      // r25 is temp reg


      // set up the data we need to access
      "ldi r19, 0x03\n"  // max command length is 3
      "mov r26,r4\n"     // (1) x
      "mov r27,r5\n"     // (1) set the register inport console
      "mov r30, r8\n"    // (1) z
      "mov r31, r9\n"    // (1) load the register for command

      "ldi r24,0x00\n"  // yet to get a byte
      "ldi r23,0x08\n"  // 8 bits to read

      // start the loop
      ".L%=read_init_loop:\n"      // no timeout since if we dont get a console comand then nothing else to do
      "ld r25, X\n"                // (2) reads pin then saves to input value
      "and r25, r16\n"             // (1) compares the input value to the bitmask
      "breq .L%=read_found_low\n"  // (1/2) jumps if the value is 0 from the and (the result was low)
      "rjmp .L%=read_init_loop\n"  // (2)

      // in loop (uses timeouts)
      ".L%=read_wait_for_low:\n"
      "ld r25, X\n"                   // (2) reads pin then saves to input value
      "and r25, r16\n"                // (1) compares the input value to the bitmask
      "breq .L%=read_found_low\n"     // (1/2) jumps if the value is 0 from the and (the result was low)
      "dec r22\n"                     // (1)
      "brne .L%=read_wait_for_low\n"  // (1/2) loop if the counter isn't 0
      // timeout if the loop did not find low
      "rjmp .L%=read_program_exit\n"  // (2) jump to end

      ".L%=read_found_low:\n"
      // want to read between the 12 and the 20th cpu cycle of the 2us.
      "nop\nnop\nnop\nnop\nnop\n"  //(5)
      "nop\nnop\nnop\nnop\nnop\n"  //(5) (10)
      "nop\nnop\nnop\nnop\nnop\n"  //(5) (15)
      "nop\nnop\nnop\nnop\nnop\n"  //(5) (20)
      "nop\n"                      //(1) (21)
      "lsl r21\n"                  // (1) moves data recived left so the new data can fit in
      "ldi r22,0x10\n"             // (1) (23) set the timeout to 16 loops, 16 * 8 = 128 cycles, or 8us


      // logic to check what the bit is.
      ".L%=read_read_bit:\n"           // not used
      "ld r25, X\n"                    // (2) reads pin then saves to input value (happens before the 2 cycles)
      "and r25, r16\n"                 // (1) compares the input value to the bitmask
      "breq .L%=read_decrement_bit\n"  // (1/2) jumps if the value is 0 from the and (the result was low)
      "sbr r21,0x01\n"                 // (1) sets the first bit in the data to be a 1
      ".L%=read_decrement_bit:\n"
      "dec r23\n"                      // (1) decrement 1 from our bit counter
      "brne .L%=read_wait_for_high\n"  // (1/2) branch if we've not received the full byte

      "inc r24\n"                 // (1) increase byte count
      "ldi r23,0x08\n"            // (1) set bitcount to 8 bits
      "st Z+, r21\n"              // (2) save %[data] back to memory and increment byte pointer
      "cp r19, r24\n"             // (1) %[len] == %[receivedBytes] ?
      "breq .L%=read_stop_bit\n"  // (1/2) jump to end of the data

      ".L%=read_wait_for_high:\n"
      "ld r25, X\n"                    // (2) read the pin
      "and r25, r16\n"                 // (1) compare pinstate with bitmask
      "brne .L%=read_wait_for_low\n"   // (1/2) if the line is high then jump
      "dec r22\n"                      // (1) decrease timeout by 1
      "brne .L%=read_wait_for_high\n"  // (1/2) loop if the counter isn't 0
      // maybe jump to the start
      "rjmp .L%=read_init_loop\n"
    // clobbers 19, 20, 21, 22, 23, 24, 25, X, Z


    ".L%=read_stop_bit:\n"
    // we get here if we recived 3 bytes
    // inputs:
      // want to wait for the last bit, to do this we get here with at most 21 cycles left in this byte then 48 in the stop byte

      // we have to use more then 69 instructions to modify the data report, after that the gc will open the port to start reading
      // may be smart to use 16 more instructions to gove 1us for the gc to open the port

      // start by loading the buffer into the
      "mov r30, r8\n"  // (1) z
      "mov r31, r9\n"  // (1) load the register for command
      "ld r25, Z+\n"   // (2) dump
      "ld r19, Z\n"    // (2) load the data format we need to make into r19

      // this is technially not needed for some modifications but i need to use some cpu cycles
      "mov r30, r14\n"  // (1) Z
      "mov r31, r15\n"  // (1) load the register for command

      // tho this is not always needed it is here anyway to wast time for the one case that it is not needed
      // we need to modify at least [4] (in the x) and [5] in the Z
      "ld r25, Z+\n"  // (2) Z = Z[1]
      "ld r25, Z+\n"  // (2) Z = Z[2]
      "ld r25, Z+\n"  // (2) Z = Z[3]
      "ld r25, Z+\n"  // (2) Z = Z[4]

      // we have r20 - r25 free (r19 is comand[1])
      // r20 = buff[4] (temp), r21 = buff[5], r22 = buff[6], r23 = buff[7], r24 = buff 4 + 5 combo, r25 = buff 6 + 7 combo
      "ld r20, Z+\n"  // (2) r20 = buff[4], incrment z pointer
      "ld r21, Z+\n"  // (2) r21 = buff[5], incrment z pointer
      "ld r22, Z+\n"  // (2) r22 = buff[6], incrment z pointer
      "ld r23, Z\n"   // (2) r23 = buff[7]
      // we now have z registers as registers to dump in
      "lsr r20\n"         // (1) shift right by 1
      "lsr r20\n"         // (1) shift right by 1 [2]
      "lsr r20\n"         // (1) shift right by 1 [3]
      "lsr r20\n"         // (1) shift right by 1 [4]
      "mov r24, r21\n"    // (1)
      "andi r24, 0xF0\n"  // (1) limit data[5] to be only the top 4 bits
      "or r24, r20\n"     // (1) top for bytes of buffer[5] then top 4 bytes of buffer[4] (todo: check if need to swap)
      "mov r25, r22\n"    // (1)
      "lsr r25\n"         // (1) shift right by 1
      "lsr r25\n"         // (1) shift right by 1 [2]
      "lsr r25\n"         // (1) shift right by 1 [3]
      "lsr r25\n"         // (1) shift right by 1 [4]
      "mov r20, r23\n"    // (1)
      "andi r20, 0xF0\n"  // (1) limit data[5] to be only the top 4 bits
      "or r25, r20\n"     // (1) top for bytes of buffer[5] then top 4 bytes of buffer[4] (todo: check if need to swap)
      "ldi r21, 0x00\n"   // (1) set r25 as a 0 register

      "mov r30, r14\n"  // (1) Z
      "mov r31, r15\n"  // (1) load the register for command
      "ld r25, Z+\n"    // (2) Z = Z[1]
      "ld r25, Z+\n"    // (2) Z = Z[2]
      "ld r25, Z+\n"    // (2) Z = Z[3]
      "ld r25, Z+\n"    // (2) Z = Z[4]


      // we have done 50/69 instructions before we can star sending
      // we have 4 cycles at the end

      "cpi r19,0x03\n"             // (1) comand == 3 ? // TODO: how to check if a value is this
      "breq .L%=read_no_change\n"  // (1/2) jump if true (52)

      "cpi r19,0x01\n"              // (1) comand == 1 ?
      "breq .L%=read_format_one\n"  // (1/2) jump if true (54)

      "cpi r19,0x02\n"              // (1) comand == 2 ?
      "breq .L%=read_format_two\n"  // (1/2) jump if true (56)

      "cpi r19,0x02\n"               // (1) comand == 4 ?
      "breq .L%=read_format_four\n"  // (1/2) jump if true (58)

      "andi r19, 0xF8\n"              // (1) comand && 0xF8 == 0x00 ?
      "breq .L%=read_format_other\n"  // (1/2) jump if true (60)

      "rjmp .L%=read_console_loop\n"  // (2) jump to start and wait since the data we got is wrong to the best of my knoweledge


      ".L%=read_no_change:\n"       // we have 53/69 cpu cycles done
      "ldi r19, 0x01\n"             // (1) indicate that we need to get new data
      "ldi r22, 0x08\n"             // (1) we are going to send 8 bytes of data
      "mov r30,r14\n"               // (1) Z
      "mov r31,r15\n"               // (1) set as the data buffer
      "rjmp .L%=send_data_setup\n"  // (2) 59/69 cycles


      ".L%=read_format_one:\n"      // we have 55/69 cpu cycles done
      "st Z+, r24\n"                // (2) x = buffer[4] = r24
      "st Z+, r22\n"                // (2) x = buffer[5] = r22
      "st Z+, r23\n"                // (2) x = buffer[6] = r23
      "st Z+, r21\n"                // (2) x = buffer[7] = 0x00
      "ldi r19, 0x01\n"             // (1) indicate that we need to get new data
      "ldi r22, 0x08\n"             // (1) we are going to send 8 bytes of data
      "mov r30,r14\n"               // (1) Z
      "mov r31,r15\n"               // (1) set as the data buffer
      "rjmp .L%=send_data_setup\n"  // 69/69


      ".L%=read_format_two:\n"      // we have 57/69 cpu cycles done
      "st Z+, r24\n"                // (2) x = buffer[4] = r24
      "st Z+, r25\n"                // (2) x = buffer[5] = r22
      "st Z+, r21\n"                // (2) x = buffer[6] = 0x00
      "st Z+, r21\n"                // (2) x = buffer[7] = 0x00
      "ldi r19, 0x01\n"             // (1) indicate that we need to get new data
      "ldi r22, 0x08\n"             // (1) we are going to send 8 bytes of data
      "mov r30,r14\n"               // (1) Z
      "mov r31,r15\n"               // (1) set as the data buffer
      "rjmp .L%=send_data_setup\n"  // 71/69


      ".L%=read_format_four:\n"  // we have 35/37 cpu cycles done
      "ld r20, Z+\n"             // (2) increment Z buffer (r20 is dummy)
      "ld r20, Z+\n"             // (2) increment z buffer (r20 is dummy)
      "st Z+, r21\n"             // (2) Z = buffer[6] = 0x00
      "st Z+, r21\n"             // (2) Z = buffer[7] = 0x00
      "ldi r19, 0x01\n"          // (1) indicate that we need to get new data
      "ldi r22, 0x08\n"          // (1) we are going to send 8 bytes of data
      "mov r30,r14\n"            // (1) Z
      "mov r31,r15\n"            // (1) set as the data buffer
      "rjmp .L%=send_data_setup\n"

      ".L%=read_format_other:\n"  // we have 35/37 cpu cycles done
      "ld r20, Z+\n"              // (2) increment Z buffer (r20 is dummy)
      "st Z+, r21\n"              // (2) Z = buffer[5] = r21
      "st Z+, r25\n"              // (2) Z = buffer[6] = r25
      "st Z+, r21\n"              // (2) Z = buffer[7] = 0x00
      "ldi r19, 0x01\n"           // (1) indicate that we need to get new data
      "ldi r22, 0x08\n"           // (1) we are going to send 8 bytes of data
      "mov r30,r14\n"             // (1) Z
      "mov r31,r15\n"             // (1) set as the data buffer
      "rjmp .L%=send_data_setup\n"
    // clobbers: r19, r20, r21, r22, r23, r24, r25, X outputs: Z output buffer


    ".L%=read_program_exit:\n"  // TODO: time out -
    // we get here after the input times out meaning either junk or 1 byte comand is recvived
    // Inputs: r24 bytecount
      // if len = 1 then logic
      "cpi r24,0x01\n"                 // (1) comand == 1 ?
      "brne .L%=read_jump_to_start\n"  // (1/2) jump if true

      "mov r30, r8\n"  // (1) Z
      "mov r31, r9\n"  // (1) load the register for command
      "ld r19, Z+\n"   // (1) r19 = Z[1]

      "cpi r19,0x00\n"              // (1) comand == 0 ?
      "breq .L%=send_setup_init\n"  // (1/2) jump if true

      "cpi r19,0xFF\n"              // (1) comand == FF ?
      "breq .L%=send_setup_init\n"  // (1/2) jump if true

      "cpi r19,0x41\n"               // (1) comand == 41 ?
      "breq .L%=send_inital_data\n"  // (1/2) jump if true

      "cpi r19,0x41\n"               // (1) comand == 42 ?
      "breq .L%=send_inital_data\n"  // (1/2) jump if true

      ".L%=read_jump_to_start:\n"
      "rjmp .L%=read_console_loop\n"  // (2) jump to start of the


      ".L%=send_setup_init:\n"  // we have x cpu cycles done
      "ldi r19, 0x00\n"         // (1) indicate that we dont need new data
      "ldi r22, 0x03\n"         // (1) we are going to send 3 bytes of data
      "mov r30,r10\n"           // (1) Z
      "mov r31,r11\n"           // (1) set as the initalisation data buffer
      "rjmp .L%=send_data_setup\n"

      ".L%=send_inital_data:\n"  // we have x cpu cycles done
      "ldi r19, 0x00\n"          // (1) indicate that we dont need new data
      "ldi r22, 0x0A\n"          // (1) we are going to send 3 bytes of data
      "mov r30,r12\n"            // (1) Z
      "mov r31,r13\n"            // (1) set as the initalisation data buffer
      "rjmp .L%=send_data_setup\n"
    // clobbers: r19, r20, r21, r22, r23, r24, r25, X outputs: Z output buffer or jumps to start of loop


    ".L%=send_data_setup:\n"
    // total of 16 instructions, get ready for the read,
      // r19 needs to be get new data condtions, r22 needs to be len
      // r20 is low, 21 is high, x is inport console
      // need to set port as an outport
      "mov r26,r0\n"   // (1) x
      "mov r27,r1\n"   // (1) set the register modeport
      "ld r25, X\n"    // (2) load portmode into r25
      "or r25, r16\n"  // (1) determin the new portmode
      "st X, r25\n"    // (2) store the bitmask

      // need to store low in 20
      "mov r26,r6\n"    // (1) x
      "mov r27,r7\n"    // (1) set as the outport console
      "ld r20, X\n"     // (2) load portmode into r20
      "com r20\n"       // (1) bitwise invers of bitmask
      "and r20, r16\n"  // (1) r20 is the port low

      // need to store high in r21
      "ld r21, X\n"    // (2) load outport console into r21
      "or r21, r16\n"  // (1) r21 is the port high
    // end code hide

    ".L%=send_data:\n"
    // inputs: r19 read_new, r20: low, r21: high, r22 = len, X = outport, z = buffer. port set to input
      // we have r23, 24, 25 to use
      // let r23 be the bitcount, 24 be the byte

      ".L%=send_data_byte_loop:\n"

      "ld r24, Z+\n"    // (2) load the next byte and increment byte pointer
      "ldi r23,0x08\n"  // (1) set bitcount to 8 bits

      ".L%=send_data_bit_loop:\n"
      "st X,r20\n"                 // (2) pull the line low
      "nop\nnop\nnop\nnop\nnop\n"  //(5) (7)
      "nop\nnop\nnop\nnop\nnop\n"  //(5) (12)
      "nop\n"                      //(1) (13)
      "lsl r24\n"                  // (1) (14) shift left. MSB goes into carry bit of status reg
      // brcc branches if the carry bit (output from previous line) is a 0
      "brcc .L%=send_data_zero_bit\n"    // (1/2) branch if carry is cleared
      "nop\n"                            // (1) (16)
      "st X,r21\n"                       // (2) set the line high again
      "rjmp .L%=send_data_finish_bit\n"  // (2) jumps to end of the bit

      ".L%=send_data_zero_bit:\n"
      // leave low, need to wait 4 cycles to sync up
      "nop\nnop\nnop\nnop\n"  //(4)


      ".L%=send_data_finish_bit:\n"
      // wait 28 cycles for 2us
      "nop\nnop\nnop\nnop\nnop\n"  //(5)
      "nop\nnop\nnop\nnop\nnop\n"  //(5) (10)
      "nop\nnop\nnop\nnop\nnop\n"  //(5) (15)
      "nop\nnop\nnop\nnop\nnop\n"  //(5) (20)
      "nop\nnop\nnop\nnop\nnop\n"  //(5) (25)
      "nop\nnop\nnop\n"            //(3) (28)

      "st X,r21\n"  // (2) set the line high

      // logic about ending the bit
      "dec r23\n"                            // (1) subtract 1 from our bit counter
      "breq .L%=send_data_load_next_byte\n"  // (1/2) branch if we've sent all the bits of this byte

      // 16 cycles for the high, - 2 for the st, -2 for the logic - 2 for the jump
      "nop\nnop\nnop\nnop\nnop\n"      //(5)
      "nop\nnop\nnop\nnop\nnop\n"      //(5) (10)
      "rjmp .L%=send_data_bit_loop\n"  // (2)

      ".L%=send_data_load_next_byte:\n"
      // we are 5 cycles into the high bit
      "dec r22\n"                       // (1) len--
      "breq .L%=send_data_loop_exit\n"  // (1/2) if the byte counter is 0, exit
      // needs to go high after 1us or 16 cycles
      // 16 - 7 (above) - 2 (the jump itself) - 3 (after jump) = 4
      "nop\nnop\nnop\nnop\n"            //(4)
      "rjmp .L%=send_data_byte_loop\n"  // (2)

      ".L%=send_data_loop_exit:\n"
      // send the stop bit, which is a 1 (1us low 3us high)
      // the line goes low in: 16 - 8 (above since line went high) = 8 cycles

      // work out the new portmode to set after the it goes high
      // the z register is not in use anymore
      "mov r30,r0\n"    // (1) Z
      "mov r31,r1\n"    // (1) set the register modeport
      "ld r25, Z\n"     // (2) load portmode into r25
      "mov r16, r24\n"  // (1)
      "com r24\n"       // (1)
      "and r25, r24\n"  // (1) determin the new portmode
      "nop\n"           // (1) 8/8 nops
      "st X,r20\n"      // (2) pull the line low
      // stay low for 1us: 16 - 2 (below st) = 14

      //
      "nop\nnop\nnop\nnop\nnop\n"              //(5) (5)
      "nop\nnop\nnop\nnop\nnop\n"              //(5) (10)
      "cpi r19, 0x01\n"                        // (1) is r19 == 0x01
      "breq .L%=send_data_prep_read_stream\n"  // (1/2) jump if true (reads new data for the command)
      "nop\nnop\n"                             //(2) (14)
      "st X,r21\n"                             // (2) set the line high again   //we do here to optimise instructions
      "st Z,r25\n"                             // (2) set the portmode to input //
      "rjmp .L%=read_console_loop\n"

      ".L%=send_data_prep_read_stream:\n"
      "ldi r19, 0x08\n"  // (1) set the len of the data to read to be 1
      "st X,r21\n"       // (2) set the line high again   //we do here to optimise instructions
      "st Z,r25\n"       // (2) set the portmode to input //
      "mov r26, 6\n"     // (1) X
      "mov r27, 7\n"     // (1) set x as the in port for comm
      "mov r30, r14\n"   // (1) Z
      "mov r31, r15\n"   // (1) set the z as the buffer poiter
      "rjmp .L%=read_stream\n"
    // sets r19 = len, X = inport comm, Z = buffer
    // Clobbers: r23, r24, r25


    ".L%=read_stream_ensure_read:\n"
    // read stream data, needs r19 as length, X as the inport, Z as the buffer
      // **logic to sunc up with the start of the data, not using timeouts**
      // used when there is not time sensitive code running

      ".L%=read_stream_init_wait_low_loop_e:\n"
      "ld r25,X\n"                               // (2) reads pin then saves to input value
      "and r25, r17\n"                           // (1) compares the input value to the bitmask
      "breq .L%=read_stream_init_wait_high_loop_e\n"  // (1/2) jumps if the value is 0 from the and (the result was low)
      "rjmp .L%=read_stream_init_wait_low_loop_e\n"

      //".L%=read_stream_init_wait_high_e:\n"
      ".L%=read_stream_init_wait_high_loop_e:\n"
      "ld r25,X\n"                            // (2) reads pin then saves to input value
      "and r25, r17\n"                        // (1) compares the input value to the bitmask
      "brne .L%=read_stream_start_reading\n"  // (1/2) jumps if the value is 0 from the and (the result was low)
      "rjmp .L%=read_stream_init_wait_high_loop_e\n"
    // clobbers:

    ".L%=read_stream:\n"  // start the function for readint he stream
    // Inputs: r19 = len, X = inport, Z = buffer
      "ldi r24,0x64\n"      // (1)
  
      // **logic to sunc up with the start of the data**
  
      ".L%=read_stream_init_wait_low_loop:\n"
      "ld r25,X\n"                                 // (2) reads pin then saves to input value
      "and r25, r17\n"                             // (1) compares the input value to the bitmask
      "breq .L%=read_stream_init_wait_high\n"      // (1/2) jumps if the value is 0 from the and (the result was low)
      "dec r24\n"                                  // (1)
      "brne .L%=read_stream_init_wait_low_loop\n"  // (1/2) jumps if not 0
      "rjmp .L%=read_stream_exit\n"
  
      ".L%=read_stream_init_wait_high:\n"
      "ldi r24,0x10\n"  // (1)
      ".L%=read_stream_init_wait_high_loop:\n"
      "ld r25,X\n"                                  // (2) reads pin then saves to input value
      "and r25, r17\n"                              // (1) compares the input value to the bitmask
      "brne .L%=read_stream_start_reading\n"        // (1/2) jumps if the value is 0 from the and (the result was low)
      "dec r24\n"                                   // (1)
      "brne .L%=read_stream_init_wait_high_loop\n"  // (1/2) jumps if not 0
      "rjmp .L%=read_stream_exit\n"
  
  
      // **Logic to read the data*
  
  
      ".L%=read_stream_wait_prebyte:\n"
      "ldi r24, 0x10\n"  /// (1)
      // logic to wait for line to go low
      ".L%=read_stream_wait_high_prebyte:\n"
      "ld r25, X\n"                               // (2) reads pin then saves to input value
      "andi r25, 0x01\n"                          // (1) compares the input value to the bitmask (all the bits get set to low then high when between data)
      "brne .L%=read_stream_wait_low_prebyte\n"   // (1/2) jumps if the value is not 0 from the and (the result was high)
      "dec r24\n"                                 // (1) decrements timeout
      "brne .L%=read_stream_wait_high_prebyte\n"  // (1/2) loop if the timeout is not 0
      "rjmp .L%=read_stream_exit\n"               // (2)
  
      // logic to wait for line to go high
      ".L%=read_stream_wait_low_prebyte:\n"
      "ld r25, X\n"                              // (2) reads pin then saves to input value
      "andi r25, 0x01\n"                         // (1) compares the input value to the bitmask (all the bits get set to low then high when between data)
      "breq .L%=read_stream_bits_1\n"            // (1/2) jumps if the value is 0 from the and (the result was low)
      "dec r24\n"                                // (1) decrements timeout
      "brne .L%=read_stream_wait_low_prebyte\n"  // (1/2) loop if the timeout is not 0
      "rjmp .L%=read_stream_exit\n"              // (2)
  
  
      ".L%=read_stream_bits_1:\n"
      // we are 8/16 into the data
  
      "nop\nnop\nnop\nnop\nnop\n"  //(5)
      "nop\nnop\nnop\nnop\nnop\n"  //(5) (10)
      "nop\nnop\nnop\nnop\nnop\n"  //(2) (15)
      "nop\n"                      // (1) (16)
  
  
      ".L%=read_stream_start_reading:\n"
      // worst case we are 7 cycles into the data when we read it and it took 5 cycles to get here
      // since the data is 16 us long and we are between 5 and 12 instructios into it, this is the time
      // to read it
      "ld r23, X\n"  // (2)
  
      ".L%=read_stream_wait_midbyte:\n"
      "ldi r24,0x10\n"  // (1)
      // need to wait for line to go 0
      ".L%=read_stream_wait_high_loop_midbyte:\n"
      "ld r25, X\n"                                    // (2) reads pin then saves to input value
      "andi r25, 0x01\n"                               // (1) compares the input value to the bitmask
      "brne .L%=read_stream_wait_low_loop_midbyte\n"   // (1/2) jumps if the value is not 0 from the and (the result was high)
      "dec r24\n"                                      // (1) decrements timeout
      "brne .L%=read_stream_wait_high_loop_midbyte\n"  // (1/2) loop if the timeout is not 0
      "rjmp .L%=read_stream_exit\n"                    // (2)
  

      ".L%=read_stream_wait_low_loop_midbyte:\n"
      "ld r25, X\n"                                   // (2) reads pin then saves to input value
      "andi r25, 0x01\n"                              // (1) compares the input value to the bitmask
      "breq .L%=read_stream_bits_2\n"                 // (1/2) jumps if the value is 0 from the and (the result was low)
      "dec r24\n"                                     // (1) decrements timeout
      "brne .L%=read_stream_wait_low_loop_midbyte\n"  // (1/2) loop if the timeout is not 0
      "rjmp .L%=read_stream_exit\n"                   // (2)

      ".L%=read_stream_bits_2:\n"
      // we are between 5 and 12 nops into the high bit, it is 16long and we want to read
      // between the 5th and 12th cycle of the data bit, this means that we need 16 cycles
      "lsl r23\n"                  // (1)
      "lsl r23\n"                  // (1) (2)
      "lsl r23\n"                  // (1) (3)
      "lsl r23\n"                  // (1) (4)
      "nop\nnop\nnop\nnop\nnop\n"  // (5) (9)
      "nop\nnop\nnop\nnop\nnop\n"  // (5) (14)
      "nop\nnop\n"                 // (4) (16)


      "ld r22, X\n"                          // (2)
      "and r22, r18\n"                       // (1)
      "or r23, r22\n"                        // (1)
      "st Z+, r23\n"                         // (2) save %[data] back to memory and increment byte pointer
      "dec r19\n"                            // (1)
      "breq .L%=read_stream_exit\n"          // (1/2) loop if the counter isn't 0
      "rjmp .L%=read_stream_wait_prebyte\n"  // (2)


      ".L%=read_stream_exit:\n"
      //"rjmp .L%=read_console_loop\n"
    // clobbers r22 r23, r24. r25.


    // outputs
    :
    : [setup_data] "X"(setup_data)
    : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r10", "r11", "r12", "r13", "r14", "r15", "r16", "r17", "r18", "r19", "r20", "r21", "r22", "r23", "r24", "r25", "r26", "r27", "r30", "r31"
  );
}

void loop() {
}
