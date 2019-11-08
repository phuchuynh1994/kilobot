#include <kilolib.h>

  
#define THRESH_LO 500
#define THRESH_HI 800

message_t message;
int message_sent = 0;


 // Constants for motion handling function.
 #define STOP 0
 #define FORWARD 1
 #define LEFT 2
 #define RIGHT 3
 #define DIF 10


 int current_motion = STOP;
 uint16_t current_light = 0;


 // Function to handle motion.
 void set_motion(int new_motion)
 {
     // Only take an action if the motion is being changed.
     if (current_motion != new_motion)
     {
         current_motion = new_motion;

         if (current_motion == STOP)
         {
             set_motors(0, 0);
         }
         else if (current_motion == FORWARD)
         {
             spinup_motors();
             set_motors(kilo_straight_left-DIF, kilo_straight_right-DIF);
         }
         else if (current_motion == LEFT)
         {
             spinup_motors();
             set_motors(kilo_turn_left-DIF, 0);
         }
         else if (current_motion == RIGHT)
         {
             spinup_motors();
             set_motors(0, kilo_turn_right-DIF);
         }
     }
 }


 // Function to sample light.
 void sample_light()
 {
     // The ambient light sensor gives noisy readings. To mitigate this,
     // we take the average of 300 samples in quick succession.

     int number_of_samples = 0;
     uint32_t sum = 0;


     while (number_of_samples < 300)
     {
         int sample = get_ambientlight();

         // -1 indicates a failed sample, which should be discarded.
         if (sample != -1)
         {
             sum = sum + sample;
             number_of_samples = number_of_samples + 1;
         }
     }

     // Compute the average.
     current_light = sum / number_of_samples;

   //  printf("Wert Lichtsensor: %u\n", current_light );
  //   delay(10);

 }


 void setup()
 {
     message.type=NORMAL;
     message.data[0]=kilo_uid;
     message.crc=message_crc(&message);
     // This ensures that the robot starts moving.
     set_motion(LEFT);
 }


 void loop()
 {
     sample_light();

     if (current_light < THRESH_LO)
     {
         set_motion(LEFT);
     }
     else if (current_light > THRESH_HI)
     {
         set_motion(RIGHT);
     }
    
     if(message_sent==1)
     {
         message_sent=0;
         if (kilo_uid==4)
         {
            set_color(RGB(1,0,0));
            delay(100);
            set_color(RGB(0,0,0));
         }
         else 
         {
             set_color(RGB(1,0,1));
             delay(100);
            set_color(RGB(0,0,0));
         }
         
     }
 }
 message_t *message_tx()
 {
     return &message;
 }
void message_tx_success()
{
    message_sent=1;
}

 int main()
 {
     kilo_init();
     kilo_message_tx=message_tx;
     kilo_message_tx_success=message_tx_success;
     kilo_start(setup, loop);

     return 0;
 }




