#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "pico/stdlib.h"


void led_task()
{   
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    while (true) {
        gpio_put(LED_PIN, 1);
        vTaskDelay(100);
        gpio_put(LED_PIN, 0);
      
        vTaskDelay(500);
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
             //printf("A-CharQueue length %d \n", uxQueueMessagesWaiting(charQueue));

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

// Show POSition command
if (cmdbuff[0]=='P' && cmdbuff[1]=='?' ) {
  printf("Position cmd P? received",cmdbuff);
  printf("L=%lld,R=%lld\n",LeftPosition,RightPosition);
}
//show VELocity
if (cmdbuff[0]=='V' && cmdbuff[1]=='?' ) {
  printf("Show Velocity cmd V? received %s",cmdbuff);
  printf("left velocity %f m/s\n ",(float)(LeftVelocity)/1000E6);
printf("right velocity %F m/s \n",RightVelocity);
}
//Position Reset command  set position to 0
if (cmdbuff[0]=='R' && cmdbuff[1]=='S' ) {
  printf("Position reset RS received",cmdbuff);
  LeftPosition=0;
  RightPosition=0;
  printf("L=%lld,R=%lld\n",LeftPosition,RightPosition);
}

//Forward Motors
if (cmdbuff[0]=='F' && cmdbuff[1]=='W' ) {
  printf("Command FW received",cmdbuff);
  gpio_put(LeftMtrPinA,1);
  gpio_put(LeftMtrPinB,0);

  gpio_put(RightMtrPinA,1);
  gpio_put(RightMtrPinB,0);
 
  printf("L=%lld,R=%lld\n",LeftPosition,RightPosition);
}
// Right turn motors
if (cmdbuff[0]=='R' && cmdbuff[1]=='T' ) {
  printf("Command FW received",cmdbuff);
  gpio_put(LeftMtrPinA,1);
  gpio_put(LeftMtrPinB,0);

  gpio_put(RightMtrPinA,0);
  gpio_put(RightMtrPinB,1);
 
  printf("L=%lld,R=%lld\n",LeftPosition,RightPosition);
}
// Left turn motors
if (cmdbuff[0]=='L' && cmdbuff[1]=='T' ) {
  printf("Command FW received",cmdbuff);
  gpio_put(LeftMtrPinA,0);
  gpio_put(LeftMtrPinB,1);

  gpio_put(RightMtrPinA,1);
  gpio_put(RightMtrPinB,0);
 
  printf("L=%lld,R=%lld\n",LeftPosition,RightPosition);
}
// reverse motors
if (cmdbuff[0]=='R' && cmdbuff[1]=='V' ) {
  printf("Command FW received",cmdbuff);
  gpio_put(LeftMtrPinA,0);
  gpio_put(LeftMtrPinB,1);

  gpio_put(RightMtrPinA,0);
  gpio_put(RightMtrPinB,1);
 
  printf("L=%lld,R=%lld\n",LeftPosition,RightPosition);
}
// 100A driver .. Green heatsink
//A1,A2=0,0 is the brake;
//A1,A2=1,0 is FWD rotation;
//A1,A2=0,1 is Rev Direction;
//A1,A2=1,1 is not allowed ?? perhaps coast?
//PA is PWM wave input (motor speed regulation);
//G is the common pin with the control board (route B is the same control);

// 12A driver Orange heatsink
//A1,B1 =0,0
//A1,B1 =1,0
//A1,B1 =0,1
//A1,B1 =1,1


// Stop motor using Brake SB
if (cmdbuff[0]=='S' && cmdbuff[1]=='B' ) {
  printf("Command SB received",cmdbuff);
  gpio_put(LeftMtrPinA,0);
  gpio_put(LeftMtrPinB,0);

  gpio_put(RightMtrPinA,0);
  gpio_put(RightMtrPinB,0);
 
  printf("L=%lld,R=%lld\n",LeftPosition,RightPosition);
}

// Set Both motors PWM ( NOT Velocity controlled)
// only use when VC=0 (velocity control =0 means off)
// PW,Value
if (cmdbuff[0]=='P' && cmdbuff[1]=='W' && cmdbuff[2]==',' ) {
  //printf("PWmset command received PW,decimal value received=%d\n",cmdbuff);
  long sval;
  char *end;
  sval=strtol(cmdbuff+3,&end,10);
  printf("val=%ld\n",sval);

printf("Cmd buff length=%d\n",strlen(cmdbuff));
// set pwm block to velocity value
    uint Rightslice_num = pwm_gpio_to_slice_num(RightPwmPin);
    uint Leftslice_num = pwm_gpio_to_slice_num(LeftPwmPin);
    uint PWMchanval=(float) sval*0.01  * PWMTOP;
    printf("PWMChanVal=%d\n",PWMchanval); 
    pwm_set_chan_level(Rightslice_num, PWM_CHAN_A, PWMchanval ); //% duty cycle to chan level
    pwm_set_chan_level(Leftslice_num, PWM_CHAN_B, PWMchanval); //% duty cycle to chan level
  
  //printf("L=%d,R=%d\n",LeftPWMVal,RightPWMVal); // measured velocity
}


//Status
if (cmdbuff[0]=='S'  && cmdbuff[1]=='?' ) {
  printf("ROBO-PICO-WHEELS RP2350");
  printf("Status cmd S? received: %s\n",cmdbuff);
  printf("version= %s\n",PGMVersion);
  printf("Software key= %02X", FlashIdPtr->id[0]);
printf("%02X", FlashIdPtr->id[1]);
printf("%02X", FlashIdPtr->id[2]);
printf("%02X", FlashIdPtr->id[3]);
printf("%02X", FlashIdPtr->id[4]);
printf("%02X", FlashIdPtr->id[5]);
printf("%02X", FlashIdPtr->id[6]);
printf("%02X\n", FlashIdPtr->id[7]);
printf("core1count=%d\n",get_Core1Counter());
printf("left velocity %d\n",LeftVelocity);
printf("right velocity %d\n",RightVelocity);
printf("left Position %d\n",get_LeftPosition());
printf("right Position %d\n",get_RightPosition());

printf("DistEnccountRightuM= %f um/enc_cnt\n",(float) (DistEncoderRightuMDec)/1000.0);
printf("DistEnccountLeftuM= %f um/enc_cnt\n",(float) (DistEncoderLeftuMDec)/1000.0);
printf("comp=%d\n",comp);
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


// OLED functions
// core 0 RTOS can only talk to OLED
void OLED_setup_gpios(void) {
    i2c_init(i2c1, 400000);
    gpio_set_function(19, GPIO_FUNC_I2C); //OLED SCL1  
    gpio_set_function(18, GPIO_FUNC_I2C); //OLED SDA1
    gpio_pull_up(19);
    gpio_pull_up(20);
}

void OLED_animation_Task() {
    const char *words[]= {"SSD1306", "DISPLAY", "DRIVER"};
const uint8_t *fonts[4]= {acme_font, bubblesstandard_font, crackers_font, BMSPA_font};
const char *Speedwords[]= { "SPEED m/s","MtrL=32.58", "Mtrl=32.59"};
const char *versionwords[]= { "PicoRoboWheels","   daveland  ", "Mtrl=32.59"};
    const uint8_t num_chars_per_disp[]={7,7,7,5};

#define SLEEPTIME 25


    ssd1306_t disp;
    disp.external_vcc=false;
    ssd1306_init(&disp, 128, 64, 0x3C, i2c1);
    ssd1306_clear(&disp);

    printf("ANIMATION!\n");

    char buf[8];

    for(;;) {

// version on oled
        ssd1306_clear(&disp);
        ssd1306_draw_string(&disp, 8, 0, 2, "---pico---");
        ssd1306_draw_string(&disp, 8, 20, 2, "RoboWheels");
        ssd1306_draw_string(&disp, 8, 40, 2, PGMVersion);
        ssd1306_draw_string(&disp, 8, 60, 2, versionwords[1]);
            ssd1306_show(&disp);
            //sleep_ms(800);
            vTaskDelay(8800); //delay
            ssd1306_clear(&disp);

        for(int y=0; y<31; ++y) {
            ssd1306_draw_line(&disp, 0, y, 127, y);
            ssd1306_show(&disp);
            //sleep_ms(SLEEPTIME);
            vTaskDelay(25); //delay
            ssd1306_clear(&disp);
        }

        for(int y=0, i=1; y>=0; y+=i) {
            ssd1306_draw_line(&disp, 0, 31-y, 127, 31+y);
            ssd1306_draw_line(&disp, 0, 31+y, 127, 31-y);
            ssd1306_show(&disp);
            //sleep_ms(SLEEPTIME);
            vTaskDelay(25); //delay
            ssd1306_clear(&disp);
            if(y==32) i=-1;
        }

        for(int i=0; i<sizeof(words)/sizeof(char *); ++i) {
            ssd1306_draw_string(&disp, 8, 24, 2, words[i]);
            ssd1306_show(&disp);
            //sleep_ms(800);
            vTaskDelay(800); //delay
            ssd1306_clear(&disp);
        }

        for(int y=31; y<63; ++y) {
            ssd1306_draw_line(&disp, 0, y, 127, y);
            ssd1306_show(&disp);
            //sleep_ms(SLEEPTIME);
            vTaskDelay(25); //delay
            ssd1306_clear(&disp);
        }

        // for(size_t font_i=0; font_i<sizeof(fonts)/sizeof(fonts[0]); ++font_i) {
        //     uint8_t c=32;
        //     while(c<=126) {
        //         uint8_t i=0;
        //         for(; i<num_chars_per_disp[font_i]; ++i) {
        //             if(c>126)
        //                 break;
        //             buf[i]=c++;
        //         }
        //         buf[i]=0;

        //         ssd1306_draw_string_with_font(&disp, 8, 24, 2, fonts[font_i], buf);
        //         ssd1306_show(&disp);
        //         //sleep_ms(800);
        //         vTaskDelay(800); //delay
        //         ssd1306_clear(&disp);
        //     }
        // }
        
        ssd1306_clear(&disp);
         ssd1306_bmp_show_image(&disp, image_data, image_size);
         ssd1306_show(&disp);
         //sleep_ms(2000);
         vTaskDelay(2000); //delay

/// my message
        ssd1306_clear(&disp);
        ssd1306_draw_string(&disp, 8, 0, 2, Speedwords[0]);
    for(int i=1; i<sizeof(Speedwords)/sizeof(char *); ++i) {
            ssd1306_draw_string(&disp, 8, 4+(16*i), 2, Speedwords[i]);
    }
            ssd1306_show(&disp);
            //sleep_ms(800);
            vTaskDelay(4800); //delay
            ssd1306_clear(&disp);
        
    }
}

int main()
{
    stdio_init_all();

    xTaskCreate(led_task, "LED_Task", 256, NULL, 1, NULL);
    vTaskStartScheduler();

    while(1){};
}