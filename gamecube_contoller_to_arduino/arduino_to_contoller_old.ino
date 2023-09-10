/*
Taking huge insperation from NicoHood's gamecube library, (used as a resorce for learning the asm and how the input is formatted)
ik i am rewriting alot, but since this is my first project using the arduino ide i wanted to only use code 
i understood and could modify in the future. 
i will try add support for other contollers in future maybe?

feel free to add stuff to this or whatever 
*/


#define CONTOLLER 4 // to change
//these 5 pins need to be on the same port, the data ones work best if on pin 1 - 4 but can be on others,k just edit the asm
#define PORTCONDITION 12 // status of the data is in portdata TODO: work out a better pin, if memory is a problem put in same buss
#define PORTDATA1 8 // data being transfered
#define PORTDATA2 9 // data being transfered
#define PORTDATA3 10 // data being transfered
#define PORTDATA4 11 // data being transfered

typedef union{
    uint8_t byteseq[8];
    struct {
      // first data byte (bitfields are sorted in LSB order)
      uint8_t a : 1;
      uint8_t b : 1;
      uint8_t x : 1;
      uint8_t y : 1;
      uint8_t start : 1;
      uint8_t origin : 1; // Indicates if GetOrigin(0x41) was called (LOW)
      uint8_t errlatch : 1;
      uint8_t errstat : 1;

      // second data byte
      uint8_t dleft : 1;
      uint8_t dright : 1;
      uint8_t ddown : 1;
      uint8_t dup : 1;
      uint8_t z : 1;
      uint8_t r : 1;
      uint8_t l : 1;
      uint8_t high1 : 1;

      // 3rd-8th data byte
      uint8_t xAxis;
      uint8_t yAxis;
      uint8_t cxAxis;
      uint8_t cyAxis;
      uint8_t left;
      uint8_t right;
    }; 
} contoller_data;

// data to send
uint8_t getdata[3] = {0x40, 0x03, 0x00};
contoller_data input_data;

// data for the pin on the console
uint8_t bitMaskContoller;

volatile uint8_t* modePortContoller;
volatile uint8_t* outPortContoller;
volatile uint8_t* inPortContoller;

// bitmask for the info and the status
uint8_t bitMaskStatus;
uint8_t bitMaskInfo;

// outport for the comunicatiuon of other boards
volatile uint8_t* outPortComm;

void setup() {
  // put your setup code here, to run once:
  //Serial.begin(9600);
  setupports();
  //Serial.print("test:\n");
  // saves the interupts
  uint8_t oldSREG = SREG;
  // clears interupts
  cli();

  //uint8_t temp[3] = {0x00, 0x00, 0x00};
  send(temp, 0x01);
  read(input_data.byteseq, 0x03);
  //input_data.origin = 1;
  // TODO remove debug code
  //input_data.byteseq[0] = 0x00;
  //uint8_t temp[1] = {0x41};
  //send(getdata, 0x03);
  //uint8_t origin[10];
  //uint8_t status = read(origin, 0x08);
  //
  //status[0] &= 0xDF;
  //uint8_t status = send_get(input_data.byteseq, 0x01, 0x03);
  //while(status != 3) {
  //  cli();
  //  send(0x00, 0x01);
  //Serial.println(status, DEC);
  //input_data.origin = 1;
  //input_data.byteseq[0] = getdata[0];
  //input_data.byteseq[1] = getdata[1];
  //input_data.byteseq[2] = getdata[2];
  //  uint8_t status = read(input_data.byteseq, 0x03);
  //  SREG = oldSREG;
  //  delay(10);
  //}
  
  //while (read(input_data.byteseq, 0x08)!=8); for the other program
  //this program can deal with any data as long as it is less then 8 bytes long, the other side can deal with the issue when connecting it to the cube. 
  
  
  // dont stream data here 
  //stream_data(input_data.byteseq, 0x08);
  SREG = oldSREG;
  //Serial.print("len:");
  //Serial.println(status, DEC);
  //Serial.println(input_data.byteseq[0], BIN);
  //Serial.println(input_data.byteseq[1], BIN);
  //Serial.println(input_data.byteseq[2], BIN);
  //Serial.println(input_data.byteseq[3], BIN);
  //Serial.println(input_data.byteseq[4], BIN);
  //Serial.println(input_data.byteseq[5], BIN);
  //Serial.println(input_data.byteseq[6], BIN);
  //Serial.println(input_data.byteseq[7], BIN);

  //Serial.println(origin[0], BIN);
  //Serial.println(origin[1], BIN);
  //Serial.println(origin[2], BIN);
  //Serial.println(origin[3], BIN);
  //Serial.println(origin[4], BIN);
  //Serial.println(origin[5], BIN);
  //Serial.println(origin[6], BIN);
  //Serial.println(origin[7], BIN);
  //Serial.println(origin[8], BIN);
  //Serial.println(origin[9], BIN);

}


void loop() {
  // put your main code here, to run repeatedly:
  // saves the interupts
  uint8_t oldSREG = SREG;
  // clears interupts
  cli();
  //input_data.byteseq[0] = getdata[0];
  //input_data.byteseq[1] = getdata[1];
  //input_data.byteseq[2] = getdata[2];
  //uint8_t status = send_get(input_data.byteseq, 0x03, 0x08);
  send(getdata, 0x03);
  //uint8_t temp[1] = {0x69};
  //send(temp, 0x01);
  
  read(input_data.byteseq, 0x08);

  input_data.origin = 1;
  //uint8_t status = gc_n64_get(input_data.byteseq, 0x08, modePortContoller, outPortContoller, inPortContoller, bitMaskContoller);
  // lets start by getting the console status
  contoller_macro();
  
  //SREG = oldSREG;
  //delay(20);
  //cli();

  //cli();
  uint8_t timeout = 0x20;
  stream_data(input_data.byteseq, 0x08, timeout);

  




  SREG = oldSREG;
  
  //Serial.print("A:");
  //Serial.println(status, DEC);
  //Serial.println(input_data.byteseq[0], BIN);
  //Serial.println(input_data.byteseq[1], BIN);
  //Serial.println(input_data.byteseq[2], BIN);
  //Serial.println(input_data.byteseq[3], BIN);
  //Serial.println(input_data.byteseq[4], BIN);
  //Serial.println(input_data.byteseq[5], BIN);
  //Serial.println(input_data.byteseq[6], BIN);
  //Serial.println(input_data.byteseq[7], BIN);
  //
  //input_data.byteseq[0]  = 0x00;
  //input_data.byteseq[1]  = 0x00;
  //input_data.byteseq[2]  = 0x00;
  //input_data.byteseq[3]  = 0x00;
  //input_data.byteseq[4]  = 0x00;
  //input_data.byteseq[5]  = 0x00;
  //input_data.byteseq[6]  = 0x00;
  //input_data.byteseq[7]  = 0x00;
  //

}

void setupports() {
  // data for the information status pin
  bitMaskContoller = digitalPinToBitMask(CONTOLLER);
  uint8_t portContoller = digitalPinToPort(CONTOLLER);

  modePortContoller = portModeRegister(portContoller); // need to swap from in and out
  outPortContoller = portOutputRegister(portContoller); // need to set the data to send
  inPortContoller = portInputRegister(portContoller); // need to 

  // data for the information status pin
  bitMaskStatus = digitalPinToBitMask(PORTCONDITION);
  uint8_t portStatus = digitalPinToPort(PORTCONDITION);

  //volatile uint8_t* modePortStatus = portModeRegister(portStatus);
  //outPortStatus = portOutputRegister(portStatus);
  //inPortStatus = portInputRegister(portStatus);


  // data for the information pin 

  bitMaskInfo = digitalPinToBitMask(PORTDATA1);
  bitMaskInfo |= digitalPinToBitMask(PORTDATA2);
  bitMaskInfo |= digitalPinToBitMask(PORTDATA3);
  bitMaskInfo |= digitalPinToBitMask(PORTDATA4);

  uint8_t portInfo = digitalPinToPort(PORTDATA1);

  //volatile uint8_t* modePortInfo = portModeRegister(portInfo);
  //outPortInfo = portOutputRegister(portInfo);
  outPortComm = portOutputRegister(portInfo);

  // set information pins to output, (since all data are on the same rail we only need these 2)
  *portModeRegister(portStatus) |= bitMaskStatus;
  *portModeRegister(portInfo) |= bitMaskInfo;

  // set the pins to low
  *outPortComm &= ~bitMaskStatus;
  *outPortComm &= ~bitMaskInfo;
}

void contoller_macro() {
  // blank for now, will be added in future

  // want one to reset game
}


// make logic to track the data from the gamecube and then send stored data to the console
void stream_data(const uint8_t* buff, uint8_t len, uint8_t loops) {
    // temporary register values used as "clobbers"
    register uint8_t temp_data;
    register uint8_t len_left;
    register uint8_t loops_left;
    //register uint8_t send_buffer;
    register uint8_t low_byte;
    register uint8_t high_byte;

    asm volatile (
        "; Start of assembly to send data to other boards\n"

        // Instruction cycles are noted in parentheses
        // branch instructions have two values, one if the branch isn't
        // taken and one if it is

        // passed in the state of the pins 
        // passed in the location to set pins
        // passed in is the number of bytes to send

        // needs a register to use as temp storage for bit
        // needs register sending buffer 
        // needs register to count the bytes sent

        // sudocode for this 
        // place to jump to that sets all the lines low apart from the controle line (want to have )

        // **set all the lines high**
        // and the sending buffer with 0x10
        // wait needed nops

        // **set line low**
        // load the bit into a temp register
        // right shift by 4
        // set the 5th bit to be 1 (this is where the sending loop starts if uninitalised)
        // or the sending buffer with the temp register
        // wait needed nops

        // **set the line to the data**
        // or the buffer with 0x01
        // wait needed nops

        // **set all the lines high**
        // and the sending buffer with 0x10
        // wait needed nops

        // **set line low**
        // load the bit into a temp register
        // and with 0x01
        // or the sending buffer with the temp register
        // wait needed nops

        // **set the line to the data**
        // or the buffer with 0x01
        // decrement the bytes sent counter
        // jump to setting the lines low if it is not 0
        // 
        // program exit

        "ori %[send_buffer], 0x0F\n" // (1) send buffer will have the lower bits set to high and upper bits unchanged
        "mov %[loops_left], %[loops]\n" // (1) 

        "mov %[low_byte], R30\n" // (1) save the place of the adress
        "mov %[high_byte], R31\n" // (1) save place of the adress

        ".L%=loop_start:\n"
        "mov %[len_left], %[len]\n" // (1) 



        ".L%=_bit_start:\n"

        // set the pins low
        "st %a[outPort],%[send_buffer]\n" // (2) set pins to send buffer
        "andi %[send_buffer], 0xF0\n" // (1) sets the lower pins in the buffer to be 0
        // 16 cycles for the bit - 3 above 
        "nop\nnop\nnop\nnop\nnop\n" //(5) 
        "nop\nnop\nnop\nnop\nnop\n" //(5) (10)
        "nop\nnop\nnop\n" //(3) (13)

        // sets the pins high
        "st %a[outPort],%[send_buffer]\n" // (2) set pins to send buffer
        "ld %[temp_data], %a[buff]\n" // (2) load the next byte into the temp register
        "lsr %[temp_data]\n" // (1)
        "lsr %[temp_data]\n" // (1)
        "lsr %[temp_data]\n" // (1)
        "lsr %[temp_data]\n" // (1)
        "ori %[temp_data], 0x10\n" // (1) (sets the bit to indicate that data is being sent)
        "or %[send_buffer], %[temp_data]\n" // (1)
        // we are 10/16 cycles into the data
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\n" //(1) (6)

        // set the pins to the data
        "st %a[outPort],%[send_buffer]\n" // (2) set pins to send buffer
        "ori %[send_buffer], 0x0F\n" // (1) sets lower pins to high
        // we are 3/16 cycles into the data
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (10)
        "nop\nnop\nnop\n" //(3) (13) 


        // sets the pins high again
        "st %a[outPort],%[send_buffer]\n" // (2) set pins to send buffer
        "andi %[send_buffer], 0xF0\n" // (1) sets the lower pins in the buffer to be 0
        // we are 3/16 cycles into the data 
        "nop\nnop\nnop\nnop\nnop\n" //(5) 
        "nop\nnop\nnop\nnop\nnop\n" //(5) (10)
        "nop\nnop\nnop\n" //(3) (13)

        // sets the pins low again
        "st %a[outPort],%[send_buffer]\n" // (2) set pins to send buffer
        "ld %[temp_data], %a[buff]+\n" // (2) load the next byte into the temp register
        "andi %[temp_data], 0x0F\n" // (1) only keeps the lower 4 bits
        "or %[send_buffer], %[temp_data]\n" // (1) sets the lower 4 bits to the data
        // we are 6/16 cycles into the data 
        "nop\nnop\nnop\nnop\nnop\n" //(5) 
        "nop\nnop\nnop\nnop\nnop\n" //(5) (10)

        // set the pins to the data
        "st %a[outPort],%[send_buffer]\n" // (2) set pins to send buffer
        "ori %[send_buffer], 0x0F\n" // (1) sets lower pins to high
        "dec %[len_left]\n" // (1) decrements the len by 1
        "breq .L%=_exit_setup\n" // (1/2) jumps if the value is 0
        // we are 5/16 in but have 2 from the jump
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\n" //(4) (9)
        "rjmp .L%=_bit_start\n" // (2)

        ".L%=_exit_setup:\n"
        "andi %[send_buffer], 0xE0\n" // (1) set the send buffer to leep the top 3 ports and lower 5 to be 0
        // we are 7/16 in, here
        "nop\nnop\nnop\nnop\nnop\n" //(5) 
        "nop\nnop\nnop\nnop\n" //(4) (9)
        "st %a[outPort],%[send_buffer]\n" // (2) set pins in the outport to low

        // wait 1 us
        "nop\nnop\nnop\nnop\n" //(4)
        "nop\nnop\nnop\nnop\n" //(4) (8)
        "nop\nnop\n" //(4) (12)

        "mov R30, %[low_byte]\n" // (1) restore place of the buffer
        "mov R31, %[high_byte]\n" // (1) restore place of the buffer

        "dec %[loops_left]\n" // (1) decrement loops left
        "breq .L%=_exit\n" // (1/2) jumps if the value is not 0
        "rjmp .L%=loop_start\n" // (2)
        ".L%=_exit:\n"

        // ----------
        // outputs:
        : [buff] "+z" (buff), // (read and write) TODO: check if writes / can it move to inputs
        [outPort] "+e" (outPortComm), // (read and write)
        [temp_data] "=&d" (temp_data), // ()
        [len_left] "=&d" (len_left), 
        [loops_left] "=&d" (loops_left),
        [low_byte] "=&d" (low_byte),
        [high_byte] "=&d" (high_byte)
        //[send_buffer] "=&r" (send_buffer) // (output only)

        // inputs:
        : [len] "r" (len),
        [send_buffer] "r" (*outPortComm), // gets the pinstates
        [loops] "r" (loops) 
        // no clobbers
        ); // end of asm volatile
}

void send(const uint8_t* buff, uint8_t len) {
    // set pin to high
    *outPortContoller |= bitMaskContoller;
    // set pin to output
    *modePortContoller |= bitMaskContoller;

    // temporary register values used as "clobbers"
    register uint8_t bitCount;
    register uint8_t data;


    // TODO: need to make nicer and easyer to read 
    asm volatile (
        "; Start of assembly to send data to the pin\n"

        // passed in to this block are:
        // the %a[buff] register is the buffer pointer for the responce to the console
        // %[len] is the register holding the length of the buffer in bytes

        // Instruction cycles are noted in parentheses
        // branch instructions have two values, one if the branch isn't
        // taken and one if it is


        // Start sending the consoles return to the console. 

        // %[data] will be the current buffer byte loaded from memory
        // %[bitCount] will be the bit counter for the current byte. when this
        // reaches 0, we need to decrement the length counter, load
        // the next buffer byte, and loop. (if the length counter becomes
        // 0, that's the next byte condition)
        // %[len] if it reaches 0 then exit

        // This label starts sending the data to the console


        // sudocode
        // load byte into data
        // set bitcount to 8
        // start bitloop
        // set the line low
        // wait nops (12)
        // shift left the data
        // branch to data branch,
        // set the data then wait
        // decerement bit count
        // loop if nmot 0
        // decrment byte count
        // loop if not 0
        // send stop bit

        // This label starts the outer loop, which sends a single byte
        ".L%=_byte_loop:\n"
        "ld %[data], %a[buff]+\n" // (2) load the next byte and increment byte pointer
        "ldi %[bitCount],0x08\n" // (1) set bitcount to 8 bits
        // This label starts the inner loop, which sends a single bit
        ".L%=_bit_loop:\n"
        // we want low for 1us to start sending a bit
        "st %a[outPort],%[low]\n" // (2) pull the line low

        // line needs to stay low for 1us
        // 16 cycles, 2 from st above, 1 from lsl below and 2 from brcc, 
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (10)
        "nop\n" //(1) (11)

        "lsl %[data]\n" // (1) shift left. MSB goes into carry bit of status reg
        // brcc branches if the carry bit (output from previous line) is a 0
        "brcc .L%=_zero_bit\n" // (1/2) branch if carry is cleared
        "nop\n" // (1)

        "st %a[outPort],%[high]\n" // (2) set the line high again
        // Now stay high for 2us = 30 cycles
        "nop\nnop\nnop\nnop\nnop\n" //(5) 
        "nop\nnop\nnop\nnop\nnop\n" //(5) (10)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (15)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (20)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (25)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (30)
        "rjmp .L%=_finish_bit\n" // (2) jumps to end of the bit

        
        // this block is the timing for a 0 bit (3us low, 1us high)
        ".L%=_zero_bit:\n"
        // Need to go high in 2us, (32 cycles)
        "nop\nnop\nnop\nnop\nnop\n" //(5) 
        "nop\nnop\nnop\nnop\nnop\n" //(5) (10)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (15)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (20)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (25)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (30)
        "nop\nnop\n" // (2) (32)

        
        // The two branches meet up here.
        ".L%=_finish_bit:\n"

        "st %a[outPort],%[high]\n" // (2) set the line high

        // logic about ending the bit
        "dec %[bitCount]\n" // (1) subtract 1 from our bit counter
        "breq .L%=_load_next_byte\n" // (1/2) branch if we've sent all the bits of this byte

        // 16 cycles for the high, - 2 for the st, -2 for the logic - 2 for the jump
        "nop\nnop\nnop\nnop\nnop\n" //(5) 
        "nop\nnop\nnop\nnop\nnop\n" //(5) (10)
        "rjmp .L%=_bit_loop\n" // (2)


        // we are 5 cycles into the high bit
        ".L%=_load_next_byte:\n"
        "dec %[len]\n" // (1) len--
        "breq .L%=_loop_exit\n" // (1/2) if the byte counter is 0, exit
        // needs to go high after 1us or 16 cycles
        // 16 - 7 (above) - 2 (the jump itself) - 3 (after jump) = 4
        "nop\nnop\nnop\nnop\n" //(4) 
        "rjmp .L%=_byte_loop\n" // (2)


        // Loop exit
        ".L%=_loop_exit:\n"

        // final task: send the stop bit, which is a 1 (1us low 3us high)
        // the line goes low in:
        // 16 - 8 (above since line went high) = 8 cycles
        "nop\nnop\nnop\nnop\nnop\n" //(5) 
        "nop\nnop\nnop\n" //(3) (8)

        "st %a[outPort],%[low]\n" // (2) pull the line low
        // stay low for 1us
        // 16 - 2 (below st) = 14
        "nop\nnop\nnop\nnop\nnop\n" //(5) 
        "nop\nnop\nnop\nnop\nnop\n" //(5) (10)
        "nop\nnop\nnop\nnop\n" //(4) (14)

        "st %a[outPort],%[high]\n" // (2) set the line high again
        // just stay high. no need to wait 3us before returning


        // exit asm
        //".L%=_program_exit:\n"

        // ----------
        // outputs:
        : [buff] "+e" (buff), // (read and write)
        [outPort] "+e" (outPortContoller), // (read and write)
        [bitCount] "=&d" (bitCount), // (output only, ldi needs the upper registers)
        [data] "=&d" (data) // (output only)

        // inputs:
        : [len] "r" (len),
        [high] "r" (*outPortContoller | bitMaskContoller), // precalculate new pin states
        //[inPortConsole] "e" (inPortContoller), // where to read from
        [low] "r" (*outPortContoller & ~bitMaskContoller) // this works because we turn interrupts off
        // no clobbers
        ); // end of asm volatile
}

uint8_t read(const uint8_t* buff, uint8_t len_send) {
    // set pin to input
    *modePortContoller &= ~bitMaskContoller;
    // enable internel pullup resistor
    *outPortContoller |= bitMaskContoller;

    // temporary register values used as "clobbers"
    register uint8_t bitCount;
    register uint8_t receivedBytes; // bytes found

    register uint8_t data; // temp for bytes recived

    register uint8_t waitloop1;

    register uint8_t lastbit; 
    register uint8_t inputVal; // temp to store the pinstates when reading. 
    register uint8_t len;


    asm volatile (
        "; Start of assembly to read input\n"

        // passed in to this block are:
        // the %a[buff] register is the buffer pointer for the responce to the console
        // %[len] is the register holding the length of the buffer in bytes

        //  buff - pointer to place to store data
        //  bitCount - bits left to read in current byte
        //  data - current byte
        //  waitloop1 - outer loop for waiting for comand
        //  waitloop2 - inner loop for waiting for comand
        //  inputVal - temp to store value of pin
        //  receivedBytes - number of bytes recived
        //
        // can merge recived bytes into the len
        // can merge the bitcount into one of the loops



        // ASM overview
        // start big loop, (255 loops max)
        // start smaller loop(255 loops max)
        // 
        //  LOOP INFORMATION
        // get status of the pin that we talk on
        // compare to bitmast
        // break to foundlow if a 0 from the and
        // decrement number of times that loop will operate
        // jump to start of loop if not 0
        // 
        // loop twice more
        // jump to start of big loop
        //
        // search for low loop
        // read pin and work out if it is low
        // have a preset max number of loops that will be done before error
        // 
        //
        // found low
        // preform a number of nops so we read data from the middle of the 2us of data being sent
        // depeding on what was found break
        //
        // if high 
        // shift left
        // then we set the bit in the data buffer to high 
        // check the status of the number of bits found
        // jump to the wait for low loop
        //
        // if low
        // shift left
        // check number of bits found
        // enter wait for high loop
        //
        // wait for high loop
        // read pin, break when high
        //
        // exit error
        // exit

        // set the outer loop to loop 256 times
        "ldi %[waitloop1],0x00\n" // (1)
        "ldi %[receivedBytes],0x00\n" // (1) yet to get a byte
        "ldi %[bitCount],0x08\n" // (1) set bitcount to 8 bits
        "mov %[len],%[len_send]\n" // (1) set bitcount to 8 bits

        ".L%=_externel_loop:\n"
        // start the internal loop to loop 256 times
        "ldi %[data],0x00\n" // (1)

        ".L%=_internel_loop1:\n"
        "ld %[inputVal], %a[inPortContoller]\n" // (2) reads pin then saves to input value 
        "and %[inputVal], %[bitMaskContoller]\n" // (1) compares the input value to the bitmask
        "breq .L%=_found_low\n" // (1/2) jumps if the value is 0 from the and (the result was low)
        "dec %[data]\n" // (1) decrease loop iterations by 1
        "brne .L%=_internel_loop1\n" // (1/2) loop if the counter isn't 0


        // repeat all the code second time to increase instructions 
        "ldi %[inputVal],0x00\n" // (1)

        ".L%=_internel_loop2:\n"
        "ld %[inputVal], %a[inPortContoller]\n" // (2) reads pin then saves to input value
        "and %[inputVal], %[bitMaskContoller]\n" // (1) compares the input value to the bitmask
        "breq .L%=_found_low\n" // (1/2) jumps if the value is 0 from the and (the result was low)
        "dec %[data]\n" // (1) decrease loop iterations by 1
        "brne .L%=_internel_loop2\n" // (1/2) loop if the counter isn't 0

        // 3rd loop so we are sure that we wait long enough
        "ldi %[data],0x00\n" // (1)

        ".L%=_internel_loop3:\n"
        "ld %[inputVal], %a[inPortContoller]\n" // (2) reads pin then saves to input value 
        "and %[inputVal], %[bitMaskContoller]\n" // (1) compares the input value to the bitmask
        "breq .L%=_found_low\n" // (1/2) jumps if the value is 0 from the and (the result was low)
        "dec %[data]\n" // (1) decrease loop iterations by 1
        "brne .L%=_internel_loop3\n" // (1/2) loop if the counter isn't 0

        // checking here reduces the worst case senario that the input remains undetected for to 8 (from 10)
        "ld %[inputVal], %a[inPortContoller]\n" // (2) reads pin then saves to input value 
        "and %[inputVal], %[bitMaskContoller]\n" // (1) compares the input value to the bitmask
        "breq .L%=_found_low\n" // (1/2) jumps if the value is 0 from the and (the result was low)

        // decrement the externel loop
        "dec %[waitloop1]\n" // (1) decrease outer counter by 1
        "brne .L%=_externel_loop\n" // (1/2) loop if the counter isn't 0
        

        // loop has timed out since console has not queried for the port yet. 
        // outer loop loops 255 times and all 3 of the inner loops loop 255 times with 7 instructions for each loop
        // this totals 85.345ms
        "rjmp .L%=_program_exit\n" // (2) timeout, jump to the end


        // ** Reading the pin now**





        // checks the pin every 7 cycles
        ".L%=_wait_for_low:\n"
        "ldi %[waitloop1],0x10\n" // (1) max 16 loops so that will provide time to know that there is smth wrong
        ".L%=_wait_for_low_loop:\n" 
        "ld %[inputVal], %a[inPortContoller]\n" // (2) reads pin then saves to input value 
        "and %[inputVal], %[bitMaskContoller]\n" // (1) compares the input value to the bitmask
        "breq .L%=_found_low\n" // (1/2) jumps if the value is 0 from the and (the result was low)
        "dec %[waitloop1]\n" // (1) 
        "brne .L%=_wait_for_low_loop\n" // (1/2) loop if the counter isn't 0
        // timeout if the loop did not find low
        "rjmp .L%=_program_exit\n" // (2) jump to end
        
        // logic for identifying the data 
        // worst case it has been 8 instructions since last pin check and it takes 5 instrctions to jump here
        // this means that between 3 instrctions and 11 instructions left in the start of this bit
        ".L%=_found_low:\n"
        // to check in the most optimal way we want to check in the middle of the 32 bits of data, 
        // this will be between the 12 and the 20th bit. 
        // if the 3 left goes to the 20 then we need 23 nops, meaning that the one with 11 left checks on the 12th cycle as e wanted. 
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (10)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (15)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (20)
        "nop\nnop\nnop\n" //(3) (23)

        // logic to check what the bit is. 
        ".L%=_read_bit:\n"
        "ld %[inputVal], %a[inPortContoller]\n" // (2) reads pin then saves to input value (happens before the 2 cycles)

        "lsl %[data]\n" // (1) moves data recived left so the new data can fit in 
        "and %[inputVal], %[bitMaskContoller]\n" // (1) compares the input value to the bitmask
        "breq .L%=_decrement_bit\n" // (1/2) jumps if the value is 0 from the and (the result was low)
        "sbr %[data],0x01\n" // (1) sets the first bit in the data to be a 1
        ".L%=_decrement_bit:\n"
        "dec %[bitCount]\n" // (1) decrement 1 from our bit counter
        "brne .L%=_wait_for_high\n" // (1/2) branch if we've not received the full byte
        
        "inc %[receivedBytes]\n" // (1) increase byte count
        "ldi %[bitCount],0x08\n" // (1) set bitcount to 8 bits
        "st %a[buff]+,%[data]\n" // (2) save %[data] back to memory and increment byte pointer
        "cp %[len],%[receivedBytes]\n" // (1) %[len] == %[receivedBytes] ?
        "breq .L%=_stop_bit\n" // (1/2) jump to w
        
        ".L%=_wait_for_high:\n"
        "ldi %[waitloop1],%[timeout_length]\n" // (1) set the timeout to 16 loops, if it has not gone high then there is error
        ".L%=_wait_for_high_loop:\n" // 7 cycles if loop fails
        "ld %[inputVal], %a[inPortContoller]\n" // (2) read the pin
        "and %[inputVal], %[bitMaskContoller]\n" // (1) compare pinstate with bitmask
        "brne .L%=_wait_for_low\n" // (1/2) if the line is high then jump
        "dec %[waitloop1]\n" // (1) decrease timeout by 1
        "brne .L%=_wait_for_high_loop\n" // (1/2) loop if the counter isn't 0

        ".L%=_stop_bit:\n"
        // can be removed since we dont talk imediatly after read

      //  // want to wait for the last bit, to do this we get here with at most 21 bits in this byte , 
      //  "nop\nnop\nnop\nnop\nnop\n" //(5)
      //  "nop\nnop\nnop\nnop\nnop\n" //(5) (10)
      //  "nop\nnop\nnop\nnop\nnop\n" //(5) (15)
      //  "nop\nnop\nnop\nnop\nnop\n" //(5) (20)
      //  "nop\n" //(3) (21)
      //  // now we want to wait the 16 cycles for the last low to end
      //  "nop\nnop\nnop\nnop\nnop\n" //(5)
      //  "nop\nnop\nnop\nnop\nnop\n" //(5) (10)
      //  "nop\nnop\nnop\nnop\nnop\n" //(5) (15)
      //  "nop\n" //(3) (16)
      //  // should be high or turning high rn, 

        
        // we return the number of bits filled 
        ".L%=_program_exit:\n"

        // ----------
        // outputs:
        : [buff] "+e" (buff), // (read and write) // where to save the bytes to
        [bitCount] "=&d" (bitCount), // (output only, ldi needs the upper registers)
        [data] "=&d" (data), // (output only) // temp for storing bits recived when constructing bytes, ldi needs the upper registers)
        [waitloop1] "=&d" (waitloop1), // (output only, ldi needs the upper registers)
        [inputVal] "=&d" (inputVal), // temp storager for pin value 
        [receivedBytes] "=&d" (receivedBytes), // (ldi needs the upper registers)
        [len] "=&d" (len)

        // inputs:
        : [len_send] "r" (len_send), // max bytes to get
        [inPortContoller] "e" (inPortContoller), // where to read from
        [bitMaskContoller] "r" (bitMaskContoller), // for identifying the port to check state
        [timeout_length] "M" (0x10)


        // no clobbers
        ); // end of asm volatile
        //input_data.origin = 1;
        return receivedBytes;
}


void wait10() {
  asm volatile (
    "; we disabel interupts so this a a delay of 10us\n"
    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    // we have 16 nops or 1 us now

    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    // we have 32 nops or 2 us now

    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    // we have 3 us now

    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    // we have 4 us now

    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    // we have 5 us now

    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    // we have 6 us now

    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    // we have 7 us now

    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    // we have 8 us now

    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    // we have 9 us now

    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    // we have 10 us now
  );
}

void wait1() {
  asm volatile (
    "; we disabel interupts so this a a delay of 10us\n"
    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    // we have 16 nops or 1 us now

  );
}
