/** pico-robo-wheels
 * Copyright (C) {David L. Anderson} - All Rights Reserved
 * 
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders.  If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
 */



#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdio.h>
#include "pico/stdlib.h"


#include "pico-robo-wheels.pio.h"


// Core 0 is managed by Freertos.  


// core 0 starts Core 1 slave running before starting  RTOS Kernel.

// Core 1 is used as a slave and I NOT managed by FreeRTOS
// Core 1 runs the Quadrature encoders and manages position
// Core 1 runs the PWM for the 2 motors L+R
#include "pico/multicore.h"

#include "hardware/pwm.h" // Hardware PWM
#include "hardware/timer.h" // Timer hardware
#include "hardware/flash.h"
#include "hardware/irq.h"
#include "hardware/pio.h"


#include "pico-robo-wheels.pio.h"

// define Motor pwm pins to output pwm and direction for each motor
const uint LeftPwmPin =1;  // dio pin 1 
const uint LeftDirPin =2;
const uint RightPwmPin =8;  // dio pin 2
const uint RightDirPin =9;

// define Encoder AB input pins
const int LeftEncA =3; //Pio0
const int LeftEncB =4;
const int RightEncA =5; //Pio1
const int RightEncB =6;

// Position is 64 bits Signed !!
volatile long long LeftPosition ;
volatile long long RightPosition ; 
volatile u_int32_t Core1counter=99;
uint PIOoffset ;

QueueHandle_t charQueue; // charachters are buffered here

QueueHandle_t cmdQueue; // when char queue forms a command this triggers parsing

uint8_t FlashIdPtr[8];  // 8 byte flash id buffer, unique per Pico Flash chip, used fpt liscensing



// core1 running Bare metal
//------------------------------------------------------------------

static void Left_pio_irq_handler (void) {
         // test Which IRQ was raised by state machine on PIO0 sm1
         // Left encoder is serviced by SM0 With SM flags 
        if (pio0_hw->irq & 0)
        {
            LeftPosition -=1;
        }
        // test if irq 1 was raised
        if (pio0_hw->irq & 2)
        {
            LeftPosition +=1;
        }
        // clear both interrupts by setting to 1
        pio0_hw->irq = 3;
}

static void Right_pio_irq_handler (void) {
     // test Which IRQ was raised by state machine on PIO sm0
        if (pio0_hw->irq & 1)
        {
            RightPosition -=1;
        }
        // test if irq 1 was raised
        if (pio0_hw->irq & 3)
        {
            RightPosition +=1;
        }
        // clear both interrupts by setting to 1
        pio1_hw->irq = 3;

}


// setup pio0  to do the left motor encoder
void SetupPioLeft( )  {
    LeftPosition=0;

   // pio 0 is used
        PIO pio = pio0;
        // state machine 0
        uint8_t sm = 0;
        // configure the used pins as input with pull up
        pio_gpio_init(pio, LeftEncA);
        gpio_set_pulls(LeftEncA, true, false);
        pio_gpio_init(pio, LeftEncB);
        gpio_set_pulls(LeftEncB, true, false);
        // load the pio program into the pio memory
         PIOoffset = pio_add_program(pio, &pico_robo_wheels_program);
        // make a sm config
        pio_sm_config c = pico_robo_wheels_program_get_default_config(PIOoffset);
        // set the 'in' pins
        sm_config_set_in_pins(&c, LeftEncA);
        // set shift to left: bits shifted by 'in' enter at the least
        // significant bit (LSB), no autopush
        sm_config_set_in_shift(&c, false, false, 0);
        // set the IRQ handler
        irq_set_exclusive_handler(PIO0_IRQ_0, Left_pio_irq_handler);
        // enable the IRQ
        irq_set_enabled(PIO0_IRQ_0, true);
        // SM0 and SM2 are connected to up/down interrupt for Left Motor
        // on SM0 triggering Core1 IRQ0 with SMbits 0 and 2
        pio0_hw->inte0 = PIO_IRQ0_INTE_SM0_BITS | PIO_IRQ0_INTE_SM2_BITS;
        // init the sm.
        // Note: the program starts after the jump table -> initial_pc = 16
        pio_sm_init(pio, sm, 16, &c);
        // enable the sm
        pio_sm_set_enabled(pio, sm, true);
    }


// setup pio0  to do the Right motor encoder
void SetupPioRight( )  {
    RightPosition=0;
   // pio 0 is used
        PIO pio = pio0;
        // state machine 1
        uint8_t sm = 1;
        // configure the used pins as input with pull up
       
        pio_gpio_init(pio, RightEncA);
        gpio_set_pulls(RightEncA, true, false);
        pio_gpio_init(pio, RightEncB);
        gpio_set_pulls(RightEncB, true, false);
        
        // load the pio program into the pio memory
        // uint offset = pio_add_program(pio, &pico_robo_wheels_program);
        // make a sm config
        
        pio_sm_config c = pico_robo_wheels_program_get_default_config(PIOoffset);
        // set the 'in' pins
       
        sm_config_set_in_pins(&c, RightEncA);
        // set shift to left: bits shifted by 'in' enter at the least
        // significant bit (LSB), no autopush
        sm_config_set_in_shift(&c, false, false, 0);
          
        // set the IRQ handler
        irq_set_exclusive_handler(PIO0_IRQ_1, Right_pio_irq_handler);
        // enable the IRQ
        irq_set_enabled(PIO0_IRQ_1, true);
        // SM1 and SM3 are connected to up/down interrupt for Right Motor
        // on SM1 triggering Core1 IRQ1 with SMbits 1 and 3
        pio0_hw->inte0 = PIO_IRQ1_INTE_SM1_BITS | PIO_IRQ1_INTE_SM3_BITS;
        // init the sm.
        // Note: the program starts after the jump table -> initial_pc = 16
        pio_sm_init(pio, sm, 16, &c);
        // enable the sm
        pio_sm_set_enabled(pio, sm, true);
        
    }





// core 1 code here --- not managed by RTOS !!!
void core1_entry(void) {
   //toggle GPIO #1 to show core 0 is running

// run core 1 for
// PWM Generators for Motor control
// direction stop start control
// keep position of each wheel
// using Quadrature Encoders for motor position ( PIO inpput buffers and counting code)
// Velocty calulation for each motor
// virtual transmission locking Left and right speeds together
// Acceleration/decelleration control
Core1counter =98;

SetupPioLeft();
SetupPioRight();

Core1counter =1;

//flash_get_unique_id(FlashIdPtr); // used as sofware key

// setup PWM Pin I/O
gpio_set_dir(LeftPwmPin,true);
gpio_set_dir(LeftDirPin,true);
gpio_set_dir(RightPwmPin,true);
gpio_set_dir(RightDirPin,true);

Core1counter =2;

 puts("Setup pwm\n");
     // Tell GPIO 0 it is allocated to the PWM
    gpio_set_function(RightPwmPin, GPIO_FUNC_PWM);
    gpio_set_function(LeftPwmPin, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to pwm GPIO then configure Pwm
    // Fpwm= Fsys/period  = 125Mhz/ 15624  = 8000.6 hz
    // period =15624 = (Top +1) * (Phase corr+1) *( Divint + DivFrac/16)
    //
    uint slice_num = pwm_gpio_to_slice_num(RightPwmPin);
    pwm_set_clkdiv_int_frac(slice_num,1,0);  // Divint, divfrac  for 8Khz PWM rate
    slice_num = pwm_gpio_to_slice_num(LeftPwmPin);
    pwm_set_clkdiv_int_frac(slice_num,1,0);
    // set Top value
        pwm_set_wrap(slice_num, 15624/2); // 
    // Set channel A B to 50% duty cycle
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 15624/2); //50% duty cycle
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 15624/2); //50% duty cycle
    pwm_set_phase_correct(slice_num,true);


    // Set the PWM running
    pwm_set_enabled(slice_num, true);
    puts("pwm enabled\n");

Core1counter =3;
printf("Core 1 Started\n");
while (1) {
// set pwm I/O to defult
// Pwm =0%  stopped dir =FWD
Core1counter+=1;
sleep_ms(1000);
printf("Core 1 Running %d\n",Core1counter);
// Start Encoder PIO and sampling of  position
// set PosL and PosR to 0

// wait for Motor enable 

}
}


//Core 0 --- Running under Free RTOS 
//-----------------------------------------------------------
void led_task()
{   int i=0;
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    while (true) {
        gpio_put(LED_PIN, 1);
        vTaskDelay(50);
        gpio_put(LED_PIN, 0);
        vTaskDelay(50);
        //puts("Robowheels V0.1 \n");
        //printf("count %d\n",i++);
    }
}

// Get usbserial input and put it on the input Queue 
void USBSerial_Inputtask()
{   
   //gets(inputbuffer ); // this blocks until a CR is entered'

// push all received chars from usb input to the charqQueue 
//  when charqueue has a cr then we process command in command parsere which empties the queue
int chint;
char ch;
    while (true) {
        // change this later to interrupt based, for now we poll  ....
        chint= getchar_timeout_us(0);
        if (chint != PICO_ERROR_TIMEOUT) {
            ch=(char)chint;
            //printf("A-CH usbread %d \n",ch);
            int pflag=pdFALSE;
            while(!pflag){
            //printf("A-CH Charque add %d \n",ch);
              pflag=  xQueueSendToBack(charQueue,& ch,0); // if fails retry char 
             // printf("A-CharQueue length %d \n", uxQueueMessagesWaiting(charQueue));

                 vTaskDelay(1); //delay 1ms
            }
             pflag=pdFALSE;

            //printf("USBchar val %d Sent\n",(int) ch);
            if (ch==0xd)  {
                // printf("A- add cmdqueue %d \n",ch);
              pflag = xQueueSendToBack(cmdQueue,& ch,0); // if it fails retry.
             // printf("A- pflag %d \n", pflag);
             //  printf("A-cmdQueue length %d \n", uxQueueMessagesWaiting(cmdQueue));
                vTaskDelay(1); //delay 1ms
             //  printf("A-cmdQueue length %d \n", uxQueueMessagesWaiting(cmdQueue));
             //  printf("A-CharQueue length %d \n", uxQueueMessagesWaiting(charQueue));
            }


        } 
       vTaskDelay(10); //delay 1ms 

    }
}

int cmdPosa () {

}

// look in the char que pulling out chars and parsing commands
// a CR in the char que is end of command line.  When this routine finishes
//  the char que hase been read until after the CR
// Then it parses and runs the cmd
void parsecmd() {

// pull chars out and parse the command
// simple parser for now.
// one char commands 
// P?  Postion L,R wheel position
char ch;
char cmdbuff[33];
char * buffp;
int i =0;
//printf("parse cmd\n");
//printf("B-CharQueue length %d \n", uxQueueMessagesWaiting(charQueue));
ch=0;

while ((ch != 13) && (i < 32)) {
  if(  xQueueReceive( charQueue, &( ch ), ( TickType_t ) 0 ) == pdPASS) {
   // printf ("ParseCmd char %d\n",ch);
    cmdbuff[i++] = ch;
   // printf("C-CharQueue length %d \n", uxQueueMessagesWaiting(charQueue));
  }
}

cmdbuff[i]=0; //add temintating null
//printf("cmd buff read");
//printf("Length %d chars",i);
//printf("cmdbuff=%s\n",cmdbuff);
//printf("CharQueue length %d \n", uxQueueMessagesWaiting(charQueue));
// POS
if (cmdbuff[0]=='P' && cmdbuff[1]=='?' ) {
  printf("Position cmd P? received",cmdbuff);
  printf("L=%lld,R=%lld\n",LeftPosition,RightPosition);
}
//VELocity
if (cmdbuff[0]=='V' && cmdbuff[1]=='?' ) {
  printf("Velocity cmd V? received %s",cmdbuff);
}

//Status
if (cmdbuff[0]=='S'  && cmdbuff[1]=='?' ) {
  printf("Status cmd S? received: %s\n",cmdbuff);
  printf("Software key %H", FlashIdPtr[0]);
printf("%H", FlashIdPtr[1]);
printf("%H", FlashIdPtr[2]);
printf("%H", FlashIdPtr[3]);
printf("%H", FlashIdPtr[4]);
printf("%H", FlashIdPtr[5]);
printf("%H", FlashIdPtr[6]);
printf("%H\n", FlashIdPtr[7]);
printf("core1count=%d",Core1counter);
}


}


// wait for a cmd to appear in the cmd task
// then parse the chars unti the first linefeed in the input queue.
void InputCmd_task()
{ int cmdint;
char cmd;
char ch ;
    while (true) {
           //printf("Z- In cmdQueue \n");
        // change this later to interrupt based, for now we poll  ....
        //xQueueCRReceiveFromISR(charQueue,& ch,0); // if this fails char is lost.
          //printf("Z-cmdQueue length %d \n", uxQueueMessagesWaiting(cmdQueue));
         cmdint=xQueueReceive(cmdQueue,& cmd, ( TickType_t )0); // try to get cmd
              //printf("Z-cmdQueue length %d \n", uxQueueMessagesWaiting(cmdQueue));
       if (cmdint==pdTRUE){
        if (cmd == 0x0d) 
        { //printf("Z-cmd %d RCVD\n",cmd);
            parsecmd(); //parse and execute cmd
           // printf("Z-cmd parsed\n");
        }
       }
        
          
   // dump rcvd char for now


      // if( charQueue != 0 )
   // {
        // Peek a message on the created queue.  Block for 10 ticks if a
        // message is not immediately available.
        //if( xQueuePeek( charQueue, &( ch ), ( TickType_t ) 1 ) )
        //{
            // ch now points to the last char in the queue
            //  but all items still remains on the queue.
           // if(ch==0x0d)  // last char in queue is a cr , empty and parse the queue

       // }
   // }

    // ... Rest of task code.
    vTaskDelay(10); //delay 1ms
 }

    }




int main()
{
    stdio_init_all();
// create queue to hold 128 chars

 multicore_launch_core1(core1_entry);


charQueue = xQueueCreate( 128, sizeof( char ) ); //128 char buffer
cmdQueue = xQueueCreate( 100, sizeof( char ) ); //10 cmd avalible  buffer, one CR addred to this per cmd line in input buffer, signals parser
    xTaskCreate(led_task, "LED_Task", 256, NULL, 1, NULL);
    xTaskCreate(USBSerial_Inputtask, "USBSerial_Inputtask", 256, NULL, 3, NULL);
    xTaskCreate(InputCmd_task, "InputCmd_task", 256, NULL, 2, NULL);
    vTaskStartScheduler();
    puts("Start scheduler\n");
    puts("Robowheels V0.1 \n");

    while(1){};
}