
/**
 * Hunter Adams (vha3@cornell.edu)
 * 
 * This demonstration animates two balls bouncing about the screen.
 * Through a serial interface, the user can change the ball color.
 *
 * HARDWARE CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 330 ohm resistor ---> VGA Red
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - RP2040 GND ---> VGA GND
 *
 * RESOURCES USED
 *  - PIO state machines 0, 1, and 2 on PIO instance 0
 *  - DMA channels 0, 1
 *  - 153.6 kBytes of RAM (for pixel color data)
 *
 */

// Include the VGA grahics library
#include "vga_graphics.h"
// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// Include Pico libraries
#include "pico/stdlib.h"
#include "pico/divider.h"
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
// Include protothreads
#include "pt_cornell_rp2040_v1.h"

//=== the fixed point macros ========================================
// typedef signed int fix15 ;
// #define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
// #define float2fix15(a) ((fix15)((a)*32768.0)) // 2^15
// #define fix2float15(a) ((float)(a)/32768.0)
// #define absfix15(a) abs(a) 
// #define int2fix15(a) ((fix15)(a << 15))
// #define fix2int15(a) ((int)(a >> 15))
// #define char2fix15(a) (fix15)(((fix15)(a)) << 15)
// #define divfix(a,b) (fix15)(div_s64s64( (((signed long long)(a)) << 15), ((signed long long)(b))))
// #define max(a,b) ((a>b)?a:b)
// #define min(a,b) ((a<b)?a:b)

// === the fixed point macros ========================================
typedef signed short fix5 ;
#define multfix5(a,b) ((fix5)((((signed int)(a))*((signed int)(b)))>>5))
#define float2fix5(a) ((fix5)((a)*32.0)) // 2^5
#define fix2float5(a) ((float)(a)/32.0)
#define absfix5(a) abs(a) 
#define int2fix5(a) ((fix5)(a << 5))
#define fix2int5(a) ((int)(a >> 5))
#define char2fix5(a) (fix5)(((fix5)(a)) << 5)
#define divfix5(a,b) (fix5)(div_s32s32( (((signed int)(a)) << 5), ((signed int)(b))))  // may have some problem
#define max(a,b) ((a>b)?a:b)
#define min(a,b) ((a<b)?a:b)


// uS per frame
#define FRAME_RATE 33000

// the color of the boid
char color = WHITE ;

// number of boids
#define NUM_BOIDS 12000
#define turnfactor float2fix5(0.2)
#define CD float2fix5(0.47)
#define G30 float2fix5(1)
#define RC float2fix5(1)
// #define visualRange int2fix5(40)
// #define protectedRange int2fix5(8)
// #define centeringfacotor float2fix5(0.0005)
// #define matchingfactor float2fix5(0.1)
// #define avoidfactor float2fix5(0.05)
// #define maxspeed int2fix5(6)
// #define minspeed int2fix5(3)
// #define maxbias float2fix5(0.2)
// #define bias_increment float2fix5(0.0004)
// #define biasval_1 float2fix5(0.001)
// #define biasval_2 float2fix5(0.002)

int old_arena_left = 0;
int old_arena_right = 640;
int old_arena_bottom = 480;
int old_arena_top = 0;
int arena_left = 0;
int arena_right = 640;
int arena_bottom = 480;
int arena_top = 0;

bool width_wrap_flag = 0;
bool height_wrap_flag = 0;
bool old_width_wrap_flag = 0;
bool old_height_wrap_flag = 0;



// Wall detection
// #define hitBottom(b) (b>int2fix5(380))
// #define hitTop(b) (b<int2fix5(100))
// #define hitLeft(a) (a<int2fix5(100))
// #define hitRight(a) (a>int2fix5(540))

inline bool hitBottom(fix5 b){
  return (b>=int2fix5(arena_bottom));
}

inline bool hitTop(fix5 b){
  return (b<int2fix5(arena_top));
}

inline bool hitLeft(fix5 a){
  return (a<int2fix5(arena_left));
}

inline bool hitRight(fix5 a){
  return (a>int2fix5(arena_right));
}

struct boid {
  fix5 x ;
  fix5 y ;
  fix5 vx ;
  fix5 vy ;
};

struct boid flock_of_boids[NUM_BOIDS];

// Boid on core 0
fix5 boid0_x ;
fix5 boid0_y ;
fix5 boid0_vx ;
fix5 boid0_vy ;

// Boid on core 1
fix5 boid1_x ;
fix5 boid1_y ;
fix5 boid1_vx ;
fix5 boid1_vy ;

// Create a flock
void spawnFlock(struct boid* flock_of_boids)
{
  for (int i = 0; i<NUM_BOIDS; i++) {
    // Start in center of screen
    flock_of_boids[i].x = int2fix5(10) ;
    flock_of_boids[i].y = int2fix5(100) ;
    flock_of_boids[i].vx = int2fix5(rand() % 20 + 1) ;
    flock_of_boids[i].vy = int2fix5(rand() % 10 + 1) ;
  }
}




// Position Update method 
void positionUpdate(struct boid* flock_of_boids, int core_num)
{
  if (core_num == 1) {
    for (int i = 0; i<NUM_BOIDS; i += 2) {

      drawRect(fix2int5(flock_of_boids[i].x), fix2int5(flock_of_boids[i].y), 2, 2, BLACK);

      flock_of_boids[i].vx = flock_of_boids[i].vx - multfix5(flock_of_boids[i].vx, CD );
      flock_of_boids[i].vy = flock_of_boids[i].vy + G30 - multfix5(flock_of_boids[i].vy, CD );


      if (hitRight(flock_of_boids[i].x)) {
        flock_of_boids[i].vx = flock_of_boids[i].vx - turnfactor ;
      }
      if (hitLeft(flock_of_boids[i].x)) {
        flock_of_boids[i].vx = flock_of_boids[i].vx + turnfactor ;
      } 

      if (hitTop(flock_of_boids[i].y)) {
        flock_of_boids[i].vy = flock_of_boids[i].vy + turnfactor ;
      }
      if (hitBottom(flock_of_boids[i].y)) {
        flock_of_boids[i].vy = - multfix5(flock_of_boids[i].vy, RC);
        flock_of_boids[i].y = int2fix5(479);
      }
      
      flock_of_boids[i].x = flock_of_boids[i].x + flock_of_boids[i].vx ;
      flock_of_boids[i].y = flock_of_boids[i].y + flock_of_boids[i].vy ;
      
      //Draw each boid

      drawRect(fix2int5(flock_of_boids[i].x), fix2int5(flock_of_boids[i].y), 2, 2, color);
    }
  } else if (core_num == 0) {
    
    for (int i = 1; i<NUM_BOIDS; i += 2) {
      drawRect(fix2int5(flock_of_boids[i].x), fix2int5(flock_of_boids[i].y), 2, 2, BLACK);

      flock_of_boids[i].vx = flock_of_boids[i].vx - multfix5(flock_of_boids[i].vx, CD );
      flock_of_boids[i].vy = flock_of_boids[i].vy + G30 - multfix5(flock_of_boids[i].vy, CD );


      if (hitRight(flock_of_boids[i].x)) {
        flock_of_boids[i].vx = flock_of_boids[i].vx - turnfactor ;
      }
      if (hitLeft(flock_of_boids[i].x)) {
        flock_of_boids[i].vx = flock_of_boids[i].vx + turnfactor ;
      } 

      if (hitTop(flock_of_boids[i].y)) {
        flock_of_boids[i].vy = flock_of_boids[i].vy + turnfactor ;
      }
      if (hitBottom(flock_of_boids[i].y)) {
        flock_of_boids[i].vy = - multfix5(flock_of_boids[i].vy, RC);
        flock_of_boids[i].y = int2fix5(479);
      }
      
      flock_of_boids[i].x = flock_of_boids[i].x + flock_of_boids[i].vx ;
      flock_of_boids[i].y = flock_of_boids[i].y + flock_of_boids[i].vy ;
      
      //Draw each boid

      drawRect(fix2int5(flock_of_boids[i].x), fix2int5(flock_of_boids[i].y), 2, 2, color);
    }
  }
}


// ==================================================
// === users serial input thread (on core 0)
// ==================================================
static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt);
    // stores user input
    static int user_input_1 ;
    static int user_input_2 ;
    static int user_input_3 ;
    static float user_input_4 ;
    // static int time_counter = 0;
    // static int begin_time, spare_time;
    static int width_input = 0;
    static int height_input = 0;
    static int n_of_scout1_input = 0;
    static int n_of_scout2_input = 0;
    static float biasval_input_1 = 0;
    static float biasval_input_2 = 0;
    static int bias_flag_input = 0;

    // wait for 0.1 sec
    PT_YIELD_usec(100000) ;
    // announce the threader version
    sprintf(pt_serial_out_buffer, "Protothreads RP2040 v1.0\n\r");
    // non-blocking write
    serial_write ;
      while(1) {
        // // print prompt
        sprintf(pt_serial_out_buffer, "input a width(0 for wrapping): ");
        // non-blocking write
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        // convert input string to number
        sscanf(pt_serial_in_buffer,"%d", &user_input_1) ;
        width_input = user_input_1;


        

        PT_YIELD_usec(1) ;

      } // END WHILE(1)
  PT_END(pt);
} // timer thread

// This monitors the spare time for maintaining the frame rate
static int spare_time_for_display ;

// Animation on core 0
static PT_THREAD (protothread_anim(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);

    // Variables for maintaining frame rate
    static int begin_time ;

    // Spawn a boid
    // for (int i=0; i<NUM_BOIDS; i++) {
    //   spawnBoid(&boid0_x, &boid0_y, &boid0_vx, &boid0_vy, 0);
    // }

    spawnFlock(flock_of_boids);
 
    while(1) {
      // Measure time at start of thread
      begin_time = time_us_32() ;    

      // update boid's position and velocity
      positionUpdate(flock_of_boids,0) ;
      
      // delay in accordance with frame rate
      spare_time_for_display = FRAME_RATE - (time_us_32() - begin_time) ;
      // yield for necessary amount of time
      PT_YIELD_usec(spare_time_for_display) ;
     // NEVER exit while
    } // END WHILE(1)
  PT_END(pt);
} // animation thread


// information display
static PT_THREAD (protothread_vga_information(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);

    // Variables for maintaining display rate (1Hz)
    static int begin_time ;
    static int spare_time ;
    static int elapsed_time = 0;

    setTextColor(WHITE) ;
    setTextSize(1) ;
    // Will be used to write dynamic text to screen
    static char vgatext[40];
 
    while(1) {
      // Measure time at start of thread
      begin_time = time_us_32() ;

      // Static text on VGA
      setCursor(65, 0) ;
      writeString("Boids Lab") ;
      setCursor(65, 10) ;
      writeString("Number of boids:") ;
      setCursor(165, 10) ;
      sprintf(vgatext, "%d", NUM_BOIDS) ;
      writeString(vgatext) ;
      setCursor(65, 20) ;
      writeString("Current spare time(us):") ;
      setCursor(250, 0) ;
      writeString("Elapsed time:") ;

      // Dynamic text on VGA
      fillRect(330, 0, 176, 30, BLACK);
      sprintf(vgatext, "%d", elapsed_time) ;
      setCursor(330, 0) ;
      writeString(vgatext) ;

      // Dynamic text on VGA
      fillRect(205, 20, 176, 30, BLACK);
      setCursor(205, 20) ;
      if (spare_time_for_display > 0) {
        sprintf(vgatext, "%d", spare_time_for_display) ;
        writeString(vgatext) ;
      } else {
        writeString("not enough time!!!") ;
      }

      elapsed_time++;

      // delay in accordance with display rate (1Hz)
      spare_time = 1000000 - (time_us_32() - begin_time) ;

      // somethings wrong with this!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      // yield for necessary amount of time
      PT_YIELD_usec(spare_time) ;
     // NEVER exit while
    } // END WHILE(1)
  PT_END(pt);
} // animation thread

// Animation on core 1
static PT_THREAD (protothread_anim1(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);

    // Variables for maintaining frame rate
    static int begin_time ;
    static int spare_time ;

    // Spawn a boid
    // spawnBoid(&boid1_x, &boid1_y, &boid1_vx, &boid1_vy, 1);

    while(1) {
      // Measure time at start of thread
      begin_time = time_us_32() ;
      positionUpdate(flock_of_boids,1) ;      
      spare_time = FRAME_RATE - (time_us_32() - begin_time) ;
      // yield for necessary amount of time
      PT_YIELD_usec(spare_time) ;
     // NEVER exit while
    } // END WHILE(1)
  PT_END(pt);
} // animation thread

// ========================================
// === core 1 main -- started in main below
// ========================================
void core1_main(){
  // Add animation thread
  pt_add_thread(protothread_anim1);
  // Add information display on VGA
  pt_add_thread(protothread_vga_information);
  // Start the scheduler
  pt_schedule_start ;

}

// ========================================
// === main
// ========================================
// USE ONLY C-sdk library
int main(){
  // overclock
  set_sys_clock_khz(250000, true);

  // initialize stio
  stdio_init_all() ;

  // initialize VGA
  initVGA() ;

  // start core 1 
  multicore_reset_core1();
  multicore_launch_core1(&core1_main);

  // add threads
  pt_add_thread(protothread_serial);
  pt_add_thread(protothread_anim);

  // start scheduler
  pt_schedule_start ;
} 
