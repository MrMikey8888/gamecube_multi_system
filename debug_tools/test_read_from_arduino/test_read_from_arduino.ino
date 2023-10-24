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
  Serial.begin(9600);
  Serial.println(digitalPinToBitMask(PORTCONDITION), BIN);
}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println("test");
  uint8_t setup_data[19];
  setup_data[16] = digitalPinToBitMask(CONSOLE);
  uint8_t portConsole = digitalPinToPort(CONSOLE);
  uint16_t temp = (uint16_t)portModeRegister(portConsole);
  //Serial.println("temp");
  //Serial.println(temp, HEX);
  setup_data[0] = temp;       // modePortConsole_lower
  setup_data[1] = temp >> 8;  // modePortConsole_upper


  temp = (uint16_t)portOutputRegister(portConsole);
  setup_data[2] = temp;       // outPortConsole_lower
  setup_data[3] = temp >> 8;  // outPortConsole_upper

  temp = (uint16_t)portInputRegister(portConsole);
  setup_data[4] = temp;       // inPortConsole_lower
  setup_data[5] = temp >> 8;  // inPortConsole_upper


  uint8_t portData = digitalPinToPort(PORTCONDITION);
  setup_data[17] = 0x10;  // declared like this so that if it is changed you will have to change it manually
  setup_data[18] = 0x0F;  // declared like this insted of writing it manually so if you want to change it you can find where it is used


  temp = (uint16_t)portInputRegister(portData);
  setup_data[6] = temp;       // inPortComm_lower
  setup_data[7] = temp >> 8;  // inPortComm_upper
  //Serial.println("location");
  //Serial.println(setup_data[6], HEX);
  //Serial.println(setup_data[7], HEX);
  

  // set information pins to input
  *portModeRegister(portData) &= ~setup_data[17];
  *portModeRegister(portData) &= ~setup_data[18];

  // enable internel pullup resistor
  *portOutputRegister(portData) |= setup_data[17];
  *portOutputRegister(portData) |= setup_data[18];

  uint8_t command[3] = { 0x00, 0x00, 0x00 };
  temp = (uint16_t)command;
  setup_data[8] = temp;       // command_lower
  setup_data[9] = temp >> 8;  // command_upper

  uint8_t contoller_init[3] = { 0x09, 0x00, 0x03 };
  temp = (uint16_t)contoller_init;
  setup_data[10] = temp;       // contoller_init_lower
  setup_data[11] = temp >> 8;  // contoller_init_upper

  // we set the upper 2 bytes to be correct here, i cba to do it in the asm block
  uint8_t initaldata[10] = { 0x00, 0x00, 0x00, 0xAA, 0x01, 0x02, 0x04, 0x08, 0x02, 0x02 };
  //Serial.println(initaldata[0], BIN);
  //Serial.println("start first print\n");
  temp = (uint16_t)initaldata;
  setup_data[12] = temp;       // initaldata_lower
  setup_data[13] = temp >> 8;  // initaldata_upper
  Serial.println("save locartion");
  Serial.println(temp, HEX);
  //temp = &initaldata[0];
  //Serial.println(temp, HEX);
  //temp = &initaldata[1];
  //Serial.println(temp, HEX);
  //Serial.println(setup_data[12], HEX);
  //Serial.println(setup_data[13], HEX);


  uint8_t buffer[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  temp = (uint16_t)buffer;
  setup_data[14] = temp;       // buffer_lower
  setup_data[15] = temp >> 8;  // buffer_upper

  // make into a list
  // load registers when entering the asm
  //Serial.println(initaldata[0], BIN);
  //Serial.println(initaldata[1], BIN);
  //Serial.println(initaldata[2], BIN);
  //Serial.println(initaldata[3], BIN);
  //Serial.println(initaldata[4], BIN);
  //Serial.println(initaldata[5], BIN);
  //Serial.println(initaldata[6], BIN);
  //Serial.println(initaldata[7], BIN);
  //Serial.println(initaldata[8], BIN);
  //Serial.println(initaldata[9], BIN);
  //Serial.println("ram adress of the port register");
  //temp = portInputRegister(portData);
  //Serial.println(temp, HEX);
  //temp = setup_data[6];
  //temp = temp << 8;
  //temp = temp ^ setup_data[7];
  //Serial.println(temp, HEX);
  //Serial.println(setup_data[6], HEX);
  //Serial.println(setup_data[7], HEX);
  //Serial.println(setup_data[17], BIN);
  Serial.println("start setup dump");
  Serial.println(setup_data[0], HEX);
  Serial.println(setup_data[1], HEX);
  Serial.println(setup_data[2], HEX);
  Serial.println(setup_data[3], HEX);
  Serial.println(setup_data[4], HEX);
  Serial.println(setup_data[5], HEX);
  Serial.println(setup_data[6], HEX);
  Serial.println(setup_data[7], HEX);
  Serial.println(setup_data[8], HEX);
  Serial.println(setup_data[9], HEX);
  Serial.println(setup_data[10], HEX);
  Serial.println(setup_data[11], HEX);
  Serial.println(setup_data[12], HEX);
  Serial.println(setup_data[13], HEX);
  Serial.println(setup_data[14], HEX);
  Serial.println(setup_data[15], HEX);
  Serial.println(setup_data[16], HEX);
  Serial.println(setup_data[17], HEX);
  Serial.println(setup_data[18], HEX);

  delay(100);

  //uint8_t oldSREG = SREG;
  //cli();

  // we dont want to make any vairables here since we going to write everything in asm
  // we only compile this way since i dont know how to get the memory adress of the ports in asm
  asm volatile(  // initalise the portode to input and the port status to high
    "; main program\n"
    // to initalise we want to set the gamecube port to input


    // initalise all the data to be accesses quickly
    //"ld r0, %a[setup_data]+\n"  // mode port
    //"ld r1, %a[setup_data]+\n"
    //"ld r2, %a[setup_data]+\n"  // out console
    //"ld r3, %a[setup_data]+\n"
    //"ld r4, %a[setup_data]+\n"  // in console
    //"ld r5, %a[setup_data]+\n"
    //"ld r6, %a[setup_data]+\n"  // im comm port
    //"ld r7, %a[setup_data]+\n"
    //"ld r8, %a[setup_data]+\n"  // command read
    //"ld r9, %a[setup_data]+\n"
    //"ld r10, %a[setup_data]+\n"  // initalisation for contoller
    //"ld r11, %a[setup_data]+\n"
    //"ld r12, %a[setup_data]+\n"  // inital data
    //"ld r13, %a[setup_data]+\n"
    //"ld r14, %a[setup_data]+\n"  // buffer pointer
    //"ld r15, %a[setup_data]+\n"
    //"ld r16, %a[setup_data]+\n"  // bitmask console
    //"ld r17, %a[setup_data]+\n"  // bitmask status
    //"ld r18, %a[setup_data]\n"  // bitmask data

    "mov r26, %A0\n"  // Load low byte of setup_data address into r26
    "mov r27, %B0\n"  // Load high byte of setup_data address into r27
    "ld r0,  X+\n"
    "ld r1,  X+\n"
    "ld r2,  X+\n"
    "ld r3,  X+\n"
    "ld r4,  X+\n"
    "ld r5,  X+\n"
    "ld r6,  X+\n"
    "ld r7,  X+\n"
    "ld r8,  X+\n"
    "ld r9,  X+\n"
    "ld r10, X+\n"
    "ld r11, X+\n"
    "ld r12, X+\n"
    "ld r13, X+\n"
    "ld r14, X+\n"
    "ld r15, X+\n"
    "ld r16, X+\n"
    "ld r17, X+\n"
    "ld r18, X \n"



    "mov r26,r6\n"                        // X
    "mov r27,r7\n"                        // In port com
    "mov r30, r12\n"                      // Z
    "mov r31, r13\n"                      // inital data
    //"ldi r30, 0xD9\n"
    //"ldi r31, 0x08\n"

    "ldi r19, 0x08\n"                     // read 8 bytes
    "st Z+, r0\n"
    "ldi r25, 0xAA\n"
    "st Z, r25\n"
    "rjmp .L%=read_stream_ensure_read\n"  // ensure that we read the data


    ".L%=read_stream_ensure_read:\n"
    // read stream data, needs r19 as length, X as the inport, Z as the buffer
      // **logic to sunc up with the start of the data, not using timeouts**
      // used when there is not time sensitive code running

      //".L%=read_stream_init_wait_low_loop_e:\n"
      //"ld r25,X\n"                               // (2) reads pin then saves to input value
      //"and r25, r17\n"                           // (1) compares the input value to the bitmask
      //"breq .L%=read_stream_init_wait_high_loop_e\n"  // (1/2) jumps if the value is 0 from the and (the result was low)
      //"rjmp .L%=read_stream_init_wait_low_loop_e\n"
      //
      ////".L%=read_stream_init_wait_high_e:\n"
      //".L%=read_stream_init_wait_high_loop_e:\n"
      //"ld r25,X\n"                            // (2) reads pin then saves to input value
      //"and r25, r17\n"                        // (1) compares the input value to the bitmask
      //"brne .L%=read_stream_start_reading\n"  // (1/2) jumps if the value is 0 from the and (the result was low)
      //"rjmp .L%=read_stream_init_wait_high_loop_e\n"
    // clobbers:

  //  ".L%=read_stream:\n"  // start the function for readint he stream
  //  // Inputs: r19 = len, X = inport, Z = buffer
  //    "ldi r24,0x64\n"      // (1)
  //
  //    // **logic to sunc up with the start of the data**
  //
  //    ".L%=read_stream_init_wait_low_loop:\n"
  //    "ld r25,X\n"                                 // (2) reads pin then saves to input value
  //    "and r25, r17\n"                             // (1) compares the input value to the bitmask
  //    "breq .L%=read_stream_init_wait_high\n"      // (1/2) jumps if the value is 0 from the and (the result was low)
  //    "dec r24\n"                                  // (1)
  //    "brne .L%=read_stream_init_wait_low_loop\n"  // (1/2) jumps if not 0
  //    "rjmp .L%=read_stream_exit\n"
  //
  //    ".L%=read_stream_init_wait_high:\n"
  //    "ldi r24,0x10\n"  // (1)
  //    ".L%=read_stream_init_wait_high_loop:\n"
  //    "ld r25,X\n"                                  // (2) reads pin then saves to input value
  //    "and r25, r17\n"                              // (1) compares the input value to the bitmask
  //    "brne .L%=read_stream_start_reading\n"        // (1/2) jumps if the value is 0 from the and (the result was low)
  //    "dec r24\n"                                   // (1)
  //    "brne .L%=read_stream_init_wait_high_loop\n"  // (1/2) jumps if not 0
  //    "rjmp .L%=read_stream_exit\n"
  //
  //
  //    // **Logic to read the data*
  //
  //
  //    ".L%=read_stream_wait_prebyte:\n"
  //    "ldi r24, 0x10\n"  /// (1)
  //    // logic to wait for line to go low
  //    ".L%=read_stream_wait_high_prebyte:\n"
  //    "ld r25, X\n"                               // (2) reads pin then saves to input value
  //    "andi r25, 0x01\n"                          // (1) compares the input value to the bitmask (all the bits get set to low then high when between data)
  //    "brne .L%=read_stream_wait_low_prebyte\n"   // (1/2) jumps if the value is not 0 from the and (the result was high)
  //    "dec r24\n"                                 // (1) decrements timeout
  //    "brne .L%=read_stream_wait_high_prebyte\n"  // (1/2) loop if the timeout is not 0
  //    "rjmp .L%=read_stream_exit\n"               // (2)
  //
  //    // logic to wait for line to go high
  //    ".L%=read_stream_wait_low_prebyte:\n"
  //    "ld r25, X\n"                              // (2) reads pin then saves to input value
  //    "andi r25, 0x01\n"                         // (1) compares the input value to the bitmask (all the bits get set to low then high when between data)
  //    "breq .L%=read_stream_bits_1\n"            // (1/2) jumps if the value is 0 from the and (the result was low)
  //    "dec r24\n"                                // (1) decrements timeout
  //    "brne .L%=read_stream_wait_low_prebyte\n"  // (1/2) loop if the timeout is not 0
  //    "rjmp .L%=read_stream_exit\n"              // (2)
  //
  //
  //    ".L%=read_stream_bits_1:\n"
  //    // we are 8/16 into the data
  //
  //    "nop\nnop\nnop\nnop\nnop\n"  //(5)
  //    "nop\nnop\nnop\nnop\nnop\n"  //(5) (10)
  //    "nop\nnop\nnop\nnop\nnop\n"  //(2) (15)
  //    "nop\n"                      // (1) (16)
  //
  //
      ".L%=read_stream_start_reading:\n"
  //    // worst case we are 7 cycles into the data when we read it and it took 5 cycles to get here
  //    // since the data is 16 us long and we are between 5 and 12 instructios into it, this is the time
  //    // to read it
  //    "ld r23, X\n"  // (2)
  //
  //    ".L%=read_stream_wait_midbyte:\n"
  //    "ldi r24,0x10\n"  // (1)
  //    // need to wait for line to go 0
  //    ".L%=read_stream_wait_high_loop_midbyte:\n"
  //    "ld r25, X\n"                                    // (2) reads pin then saves to input value
  //    "andi r25, 0x01\n"                               // (1) compares the input value to the bitmask
  //    "brne .L%=read_stream_wait_low_loop_midbyte\n"   // (1/2) jumps if the value is not 0 from the and (the result was high)
  //    "dec r24\n"                                      // (1) decrements timeout
  //    "brne .L%=read_stream_wait_high_loop_midbyte\n"  // (1/2) loop if the timeout is not 0
  //    "rjmp .L%=read_stream_exit\n"                    // (2)
  //
  //
  //    ".L%=read_stream_wait_low_loop_midbyte:\n"
  //    "ld r25, X\n"                                   // (2) reads pin then saves to input value
  //    "andi r25, 0x01\n"                              // (1) compares the input value to the bitmask
  //    "breq .L%=read_stream_bits_2\n"                 // (1/2) jumps if the value is 0 from the and (the result was low)
  //    "dec r24\n"                                     // (1) decrements timeout
  //    "brne .L%=read_stream_wait_low_loop_midbyte\n"  // (1/2) loop if the timeout is not 0
  //    "rjmp .L%=read_stream_exit\n"                   // (2)
  //
  //    ".L%=read_stream_bits_2:\n"
  //    // we are between 5 and 12 nops into the high bit, it is 16long and we want to read
  //    // between the 5th and 12th cycle of the data bit, this means that we need 16 cycles
  //    "lsl r23\n"                  // (1)
  //    "lsl r23\n"                  // (1) (2)
  //    "lsl r23\n"                  // (1) (3)
  //    "lsl r23\n"                  // (1) (4)
  //    "nop\nnop\nnop\nnop\nnop\n"  // (5) (9)
  //    "nop\nnop\nnop\nnop\nnop\n"  // (5) (14)
  //    "nop\nnop\n"                 // (4) (16)
  //
  //
  //    "ld r22, X\n"                          // (2)
  //    "and r22, r18\n"                       // (1)
  //    "or r23, r22\n"                        // (1)
  //    "st Z+, r23\n"                         // (2) save %[data] back to memory and increment byte pointer
  //    "dec r19\n"                            // (1)
  //    "breq .L%=read_stream_exit\n"          // (1/2) loop if the counter isn't 0
  //    "rjmp .L%=read_stream_wait_prebyte\n"  // (2)


      ".L%=read_stream_exit:\n"
      //"rjmp .L%=read_console_loop\n"
    // clobbers r22 r23, r24. r25.


    // outputs
    : 
    : "r" (setup_data)
    : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r10", "r11", "r12", "r13", "r14", "r15", "r16", "r17", "r18", "r19", "r25", "r26", "r27", "r30", "r31"
  );
  delay(10);
  Serial.println("after asm setup dump");
  Serial.println(setup_data[0], HEX);
  Serial.println(setup_data[1], HEX);
  Serial.println(setup_data[2], HEX);
  Serial.println(setup_data[3], HEX);
  Serial.println(setup_data[4], HEX);
  Serial.println(setup_data[5], HEX);
  Serial.println(setup_data[6], HEX);
  Serial.println(setup_data[7], HEX);
  Serial.println(setup_data[8], HEX);
  Serial.println(setup_data[9], HEX);
  Serial.println(setup_data[10], HEX);
  Serial.println(setup_data[11], HEX);
  Serial.println(setup_data[12], HEX);
  Serial.println(setup_data[13], HEX);
  Serial.println(setup_data[14], HEX);
  Serial.println(setup_data[15], HEX);
  Serial.println(setup_data[16], HEX);
  Serial.println(setup_data[17], HEX);
  Serial.println(setup_data[18], HEX);

  //SREG = oldSREG;
  Serial.println("test");
  Serial.println(initaldata[0], BIN);
  Serial.println(initaldata[1], BIN);
  Serial.println(initaldata[2], BIN);
  Serial.println(initaldata[3], BIN);
  Serial.println(initaldata[4], BIN);
  Serial.println(initaldata[5], BIN);
  Serial.println(initaldata[6], BIN);
  Serial.println(initaldata[7], BIN);
  Serial.println(initaldata[8], BIN);
  Serial.println(initaldata[9], BIN);
  delay(100);
  //Serial.println(initaldata[], Bin);
}
