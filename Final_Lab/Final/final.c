
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

// === the fixed point macros ========================================
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0)) // 2^15
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a) 
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a,b) (fix15)(div_s64s64( (((signed long long)(a)) << 15), ((signed long long)(b))))
#define max(a,b) ((a>b)?a:b)
#define min(a,b) ((a<b)?a:b)

// uS per frame
#define FRAME_RATE 33000

// the color of the boid
char color = WHITE ;

// number of boids
#define NUM_BOIDS 600
#define turnfactor float2fix15(0.2)
#define visualRange int2fix15(40)
#define protectedRange int2fix15(8)
#define centeringfacotor float2fix15(0.0005)
#define matchingfactor float2fix15(0.1)
#define avoidfactor float2fix15(0.05)
#define maxspeed int2fix15(6)
#define minspeed int2fix15(3)
#define maxbias float2fix15(0.2)
#define bias_increment float2fix15(0.0004)
// #define biasval_1 float2fix15(0.001)
// #define biasval_2 float2fix15(0.002)

int old_arena_left = 100;
int old_arena_right = 540;
int old_arena_bottom = 380;
int old_arena_top = 100;
int arena_left = 100;
int arena_right = 540;
int arena_bottom = 380;
int arena_top = 100;

bool width_wrap_flag = 0;
bool height_wrap_flag = 0;
bool old_width_wrap_flag = 0;
bool old_height_wrap_flag = 0;
int scoup_1_num = 10;
int scoup_2_num = 20;
float biasval_1 = 0.001;
float biasval_2 = 0.002;
bool bias_flag = 0;


// Wall detection
// #define hitBottom(b) (b>int2fix15(380))
// #define hitTop(b) (b<int2fix15(100))
// #define hitLeft(a) (a<int2fix15(100))
// #define hitRight(a) (a>int2fix15(540))

inline bool hitBottom(fix15 b){
  return (b>int2fix15(arena_bottom));
}

inline bool hitTop(fix15 b){
  return (b<int2fix15(arena_top));
}

inline bool hitLeft(fix15 a){
  return (a<int2fix15(arena_left));
}

inline bool hitRight(fix15 a){
  return (a>int2fix15(arena_right));
}

struct boid {
  fix15 x ;
  fix15 y ;
  fix15 vx ;
  fix15 vy ;
  fix15 biasval;
};

struct boid flock_of_boids[NUM_BOIDS];

// Boid on core 0
fix15 boid0_x ;
fix15 boid0_y ;
fix15 boid0_vx ;
fix15 boid0_vy ;

// Boid on core 1
fix15 boid1_x ;
fix15 boid1_y ;
fix15 boid1_vx ;
fix15 boid1_vy ;

// Create a boid
void spawnBoid(fix15* x, fix15* y, fix15* vx, fix15* vy, int direction)
{
  // Start in center of screen
  *x = int2fix15(320) ;
  *y = int2fix15(240) ;
  // Choose left or right
  if (direction) *vx = int2fix15(3) ;
  else *vx = int2fix15(-3) ;
  // Moving down
  *vy = int2fix15(1) ;
}

// Create a flock
void spawnFlock(struct boid* flock_of_boids)
{
  for (int i = 0; i<NUM_BOIDS; i++) {
    // Start in center of screen
    flock_of_boids[i].x = int2fix15(rand() % 440 + 100) ;
    flock_of_boids[i].y = int2fix15(rand() % 280 + 100) ;
    flock_of_boids[i].vx = int2fix15(rand() % 3 + 3) ;
    flock_of_boids[i].vy = int2fix15(rand() % 3 +3) ;
    
    if(i < scoup_1_num){
      flock_of_boids[i].biasval = float2fix15(biasval_1);
    }
    else if ( i > scoup_1_num && i < scoup_2_num){
      flock_of_boids[i].biasval = float2fix15(biasval_2);
    }
    else{
      flock_of_boids[i].biasval = 0;
    }
    // Choose left or right
    // if (direction) *vx = int2fix15(3) ;
    // else *vx = int2fix15(-3) ;
    // // Moving down
    // *vy = int2fix15(1) ;
  }
}

// Draw the boundaries
void eraseArena() {
   // if no wrap at all
  if(old_width_wrap_flag == 0 && old_height_wrap_flag == 0){
    // arena_left
    drawVLine(old_arena_left, old_arena_top, old_arena_bottom - old_arena_top, BLACK) ;
    // arena_right 
    drawVLine(old_arena_right, old_arena_top, old_arena_bottom - old_arena_top, BLACK) ;
    // arena_top
    drawHLine(old_arena_left, old_arena_top, old_arena_right - old_arena_left, BLACK) ;
    // arena_bottom
    drawHLine(old_arena_left, old_arena_bottom, old_arena_right - old_arena_left, BLACK) ;
  }
  else if(old_width_wrap_flag == 0 && old_height_wrap_flag == 1){
    // arena_left
    drawVLine(old_arena_left, 0, 479, BLACK) ;
    // arena_right 
    drawVLine(old_arena_right, 0, 479, BLACK) ;
  }
  else if(old_width_wrap_flag == 1 && old_height_wrap_flag == 0){
    // arena_top
    drawHLine(0, old_arena_top, 639, BLACK) ;
    // arena_bottom
    drawHLine(0, old_arena_bottom, 639, BLACK) ;
  }
  else{
    // do nothing
    ;
  }
}

// Draw the boundaries
void drawArena() {
  // if no wrap at all
  if(width_wrap_flag == 0 && height_wrap_flag == 0){
    drawVLine(arena_left, arena_top, arena_bottom - arena_top, WHITE) ;
    // arena_right 
    drawVLine(arena_right, arena_top, arena_bottom - arena_top, WHITE) ;
    // arena_top
    drawHLine(arena_left, arena_top, arena_right - arena_left, WHITE) ;
    // arena_bottom
    drawHLine(arena_left, arena_bottom, arena_right - arena_left, WHITE) ;
  }
  else if(width_wrap_flag == 0 && height_wrap_flag == 1){
    // arena_left
    drawVLine(arena_left, 0, 479, WHITE) ;
    // arena_right 
    drawVLine(arena_right, 0, 479, WHITE) ;

  }
  else if(width_wrap_flag == 1 && height_wrap_flag == 0){
    // arena_top
    drawHLine(0, arena_top, 639, WHITE) ;
    // arena_bottom
    drawHLine(0, arena_bottom, 639, WHITE) ;
  }
  else{
    // do nothing
    ;
  }
}

// Detect wallstrikes, update velocity and position
// void wallsAndEdges(fix15* x, fix15* y, fix15* vx, fix15* vy)
// {
//   // Reverse direction if we've hit a wall
//   if (hitTop(*y)) {
//     *vy = (-*vy) ;
//     *y  = (*y + int2fix15(5)) ;
//   }
//   if (hitBottom(*y)) {
//     *vy = (-*vy) ;
//     *y  = (*y - int2fix15(5)) ;
//   } 
//   if (hitRight(*x)) {
//     *vx = (-*vx) ;
//     *x  = (*x - int2fix15(5)) ;
//   }
//   if (hitLeft(*x)) {
//     *vx = (-*vx) ;
//     *x  = (*x + int2fix15(5)) ;
//   } 

//   // Update position using velocity
//   *x = *x + *vx ;
//   *y = *y + *vy ;
// }

// void wallsAndEdges(fix15* x, fix15* y, fix15* vx, fix15* vy)
// {
//   // Reverse direction if we've hit a wall
//   if (hitTop(*y)) {
//     *vy = *vy + turnfactor ;
//   }
//   if (hitBottom(*y)) {
//     *vy = *vy - turnfactor ;
//   } 
//   if (hitRight(*x)) {
//     *vx = *vx - turnfactor ;
//   }
//   if (hitLeft(*x)) {
//     *vx = *vx + turnfactor ;
//   } 

//   // Update position using velocity
//   *x = *x + *vx ;
//   *y = *y + *vy ;
// }

// Position Update method 
// void singlePositionUpdate(struct boid* boid)
// {
//   if (hitTop(boid->y)) {
//     boid->vy = boid->vy + turnfactor ;
//   }
//   if (hitBottom(boid->y)) {
//     boid->vy = boid->vy - turnfactor ;
//   } 
//   if (hitRight(boid->x)) {
//     boid->vx = boid->vx - turnfactor ;
//   }
//   if (hitLeft(boid->x)) {
//     boid->vx = boid->vx + turnfactor ;
//   } 

//   // Update position using velocity
//   boid->x = boid->x + boid->vx ;
//   boid->y = boid->y + boid->vy ;
// }

// Position Update method 
void positionUpdate(struct boid* flock_of_boids, int core_num)
{
  if (core_num == 1) {
    for (int i = 0; i<NUM_BOIDS; i += 2) {
      fix15 xpos_avg = 0;
      fix15 ypos_avg = 0;
      fix15 xvel_avg = 0; 
      fix15 yvel_avg = 0;
      int neighboring_boids = 0;
      fix15 close_dx = 0; 
      fix15 close_dy = 0;

      drawRect(fix2int15(flock_of_boids[i].x), fix2int15(flock_of_boids[i].y), 2, 2, BLACK);

      for (int j = 0; j < NUM_BOIDS; j++) {
        if (j == i){
          continue;
        }
        fix15 dx = flock_of_boids[i].x - flock_of_boids[j].x;
        fix15 dy = flock_of_boids[i].y - flock_of_boids[j].y;
        if (abs(dx) < visualRange && abs(dy) < visualRange) {

          fix15 delta_distance = max(abs(dx),abs(dy)) + (min(abs(dx),abs(dy))>>1);

          if (delta_distance < protectedRange) {
            close_dx += flock_of_boids[i].x - flock_of_boids[j].x;
            close_dy += flock_of_boids[i].y - flock_of_boids[j].y;
          }
          else if(delta_distance < visualRange) {
            xpos_avg += flock_of_boids[j].x;
            ypos_avg += flock_of_boids[j].y;
            xvel_avg += flock_of_boids[j].vx;
            yvel_avg += flock_of_boids[j].vy;
            neighboring_boids += 1;
          }
        }
      }
      if ( neighboring_boids > 0) {
        xpos_avg = divfix(xpos_avg,int2fix15(neighboring_boids));
        ypos_avg = divfix(ypos_avg,int2fix15(neighboring_boids));
        xvel_avg = divfix(xvel_avg,int2fix15(neighboring_boids));
        yvel_avg = divfix(yvel_avg,int2fix15(neighboring_boids));

        flock_of_boids[i].vx = (flock_of_boids[i].vx + multfix15((xpos_avg - flock_of_boids[i].x), centeringfacotor) + multfix15((xvel_avg - flock_of_boids[i].vx), matchingfactor));
        flock_of_boids[i].vy = (flock_of_boids[i].vy + multfix15((ypos_avg - flock_of_boids[i].y), centeringfacotor) + multfix15((yvel_avg - flock_of_boids[i].vy), matchingfactor));
      }
      flock_of_boids[i].vx = flock_of_boids[i].vx + multfix15(close_dx, avoidfactor);
      flock_of_boids[i].vy = flock_of_boids[i].vy + multfix15(close_dy, avoidfactor);


      if (width_wrap_flag == 0){
        if (hitRight(flock_of_boids[i].x)) {
          flock_of_boids[i].vx = flock_of_boids[i].vx - turnfactor ;
        }
        if (hitLeft(flock_of_boids[i].x)) {
          flock_of_boids[i].vx = flock_of_boids[i].vx + turnfactor ;
        } 
      }

      if (height_wrap_flag == 0){
        if (hitTop(flock_of_boids[i].y)) {
          flock_of_boids[i].vy = flock_of_boids[i].vy + turnfactor ;
        }
        if (hitBottom(flock_of_boids[i].y)) {
          flock_of_boids[i].vy = flock_of_boids[i].vy - turnfactor ;
        }
      } 
      

      fix15 speed = max(abs(flock_of_boids[i].vx), abs(flock_of_boids[i].vy)) + (min(abs(flock_of_boids[i].vx), abs(flock_of_boids[i].vy))>>1);

      if (speed < minspeed){
        flock_of_boids[i].vx = multfix15(divfix(flock_of_boids[i].vx,speed),minspeed);
        flock_of_boids[i].vy = multfix15(divfix(flock_of_boids[i].vy,speed),minspeed);
      }
      if (speed > maxspeed){
        flock_of_boids[i].vx = multfix15(divfix(flock_of_boids[i].vx,speed),maxspeed);
        flock_of_boids[i].vy = multfix15(divfix(flock_of_boids[i].vy,speed),maxspeed);
      }

      // Update position using velocity
      if (width_wrap_flag == 1){
        if (flock_of_boids[i].x >= int2fix15(640)){
          flock_of_boids[i].x -= int2fix15(640);
        }
        else if (flock_of_boids[i].x < int2fix15(0)){
          flock_of_boids[i].x += int2fix15(640);
        }
      }

      if (height_wrap_flag == 1){
        if (flock_of_boids[i].y >= int2fix15(480)){
          flock_of_boids[i].y -= int2fix15(480);
        }
        else if (flock_of_boids[i].y < int2fix15(0)){
          flock_of_boids[i].y += int2fix15(480);
        }
      }

      if (bias_flag) {
        if (i < scoup_1_num){
          if (flock_of_boids[i].vx > 0){
            flock_of_boids[i].biasval = min(maxbias, flock_of_boids[i].biasval + bias_increment);
          }
          else{
            flock_of_boids[i].biasval = max(bias_increment, flock_of_boids[i].biasval - bias_increment);
          }
        }else if (i > scoup_1_num && i < scoup_2_num){
          if (flock_of_boids[i].vx < 0){
            flock_of_boids[i].biasval = min(maxbias, flock_of_boids[i].biasval + bias_increment);
          }
          else{
            flock_of_boids[i].biasval = max(bias_increment, flock_of_boids[i].biasval - bias_increment);
          }
        }
      }

      // if (i < scoup_1_num){
      //   flock_of_boids[i].vx = flock_of_boids[i].vx - (flock_of_boids[i].vx >> 10) + flock_of_boids[i].biasval;
      // }
      // else if(i > scoup_1_num && i < scoup_2_num){
      //   flock_of_boids[i].vx = flock_of_boids[i].vx - (flock_of_boids[i].vx >> 10) - flock_of_boids[i].biasval;
      // }

      if (i < scoup_1_num){
        flock_of_boids[i].vx = multfix15(int2fix15(1) - flock_of_boids[i].biasval, flock_of_boids[i].vx) + flock_of_boids[i].biasval;
      }
      else if(i > scoup_1_num && i < scoup_2_num){
        flock_of_boids[i].vx = multfix15(int2fix15(1) - flock_of_boids[i].biasval, flock_of_boids[i].vx) - flock_of_boids[i].biasval;
      }

      flock_of_boids[i].x = flock_of_boids[i].x + flock_of_boids[i].vx ;
      flock_of_boids[i].y = flock_of_boids[i].y + flock_of_boids[i].vy ;
      
      //Draw each boid
      if(i < scoup_1_num){
        drawRect(fix2int15(flock_of_boids[i].x), fix2int15(flock_of_boids[i].y), 2, 2, RED);
      }
      else if(i > scoup_1_num && i < scoup_2_num){
        drawRect(fix2int15(flock_of_boids[i].x), fix2int15(flock_of_boids[i].y), 2, 2, GREEN);
      }
      else {
        drawRect(fix2int15(flock_of_boids[i].x), fix2int15(flock_of_boids[i].y), 2, 2, color);
      }
    }
  } else if (core_num == 0) {
    for (int i = 1; i<NUM_BOIDS; i += 2) {
      fix15 xpos_avg = 0;
      fix15 ypos_avg = 0;
      fix15 xvel_avg = 0; 
      fix15 yvel_avg = 0;
      int neighboring_boids = 0;
      fix15 close_dx = 0; 
      fix15 close_dy = 0;

      drawRect(fix2int15(flock_of_boids[i].x), fix2int15(flock_of_boids[i].y), 2, 2, BLACK);

      for (int j = 0; j < NUM_BOIDS; j++) {
        if (j == i){
          continue;
        }
        fix15 dx = flock_of_boids[i].x - flock_of_boids[j].x;
        fix15 dy = flock_of_boids[i].y - flock_of_boids[j].y;
        if (abs(dx) < visualRange && abs(dy) < visualRange) {

          fix15 delta_distance = max(abs(dx),abs(dy)) + (min(abs(dx),abs(dy))>>1);

          if (delta_distance < protectedRange) {
            close_dx += flock_of_boids[i].x - flock_of_boids[j].x;
            close_dy += flock_of_boids[i].y - flock_of_boids[j].y;
          }
          else if(delta_distance < visualRange) {
            xpos_avg += flock_of_boids[j].x;
            ypos_avg += flock_of_boids[j].y;
            xvel_avg += flock_of_boids[j].vx;
            yvel_avg += flock_of_boids[j].vy;
            neighboring_boids += 1;
          }
        }
      }
      if ( neighboring_boids > 0) {
        xpos_avg = divfix(xpos_avg,int2fix15(neighboring_boids));
        ypos_avg = divfix(ypos_avg,int2fix15(neighboring_boids));
        xvel_avg = divfix(xvel_avg,int2fix15(neighboring_boids));
        yvel_avg = divfix(yvel_avg,int2fix15(neighboring_boids));

        flock_of_boids[i].vx = (flock_of_boids[i].vx + multfix15((xpos_avg - flock_of_boids[i].x), centeringfacotor) + multfix15((xvel_avg - flock_of_boids[i].vx), matchingfactor));
        flock_of_boids[i].vy = (flock_of_boids[i].vy + multfix15((ypos_avg - flock_of_boids[i].y), centeringfacotor) + multfix15((yvel_avg - flock_of_boids[i].vy), matchingfactor));
      }
      flock_of_boids[i].vx = flock_of_boids[i].vx + multfix15(close_dx, avoidfactor);
      flock_of_boids[i].vy = flock_of_boids[i].vy + multfix15(close_dy, avoidfactor);


      if (width_wrap_flag == 0){
        if (hitRight(flock_of_boids[i].x)) {
          flock_of_boids[i].vx = flock_of_boids[i].vx - turnfactor ;
        }
        if (hitLeft(flock_of_boids[i].x)) {
          flock_of_boids[i].vx = flock_of_boids[i].vx + turnfactor ;
        } 
      }

      if (height_wrap_flag == 0){
        if (hitTop(flock_of_boids[i].y)) {
          flock_of_boids[i].vy = flock_of_boids[i].vy + turnfactor ;
        }
        if (hitBottom(flock_of_boids[i].y)) {
          flock_of_boids[i].vy = flock_of_boids[i].vy - turnfactor ;
        }
      } 
      

      fix15 speed = max(abs(flock_of_boids[i].vx), abs(flock_of_boids[i].vy)) + (min(abs(flock_of_boids[i].vx), abs(flock_of_boids[i].vy))>>1);

      if (speed < minspeed){
        flock_of_boids[i].vx = multfix15(divfix(flock_of_boids[i].vx,speed),minspeed);
        flock_of_boids[i].vy = multfix15(divfix(flock_of_boids[i].vy,speed),minspeed);
      }
      if (speed > maxspeed){
        flock_of_boids[i].vx = multfix15(divfix(flock_of_boids[i].vx,speed),maxspeed);
        flock_of_boids[i].vy = multfix15(divfix(flock_of_boids[i].vy,speed),maxspeed);
      }

      // Update position using velocity
      if (width_wrap_flag == 1){
        if (flock_of_boids[i].x >= int2fix15(640)){
          flock_of_boids[i].x -= int2fix15(640);
        }
        else if (flock_of_boids[i].x < int2fix15(0)){
          flock_of_boids[i].x += int2fix15(640);
        }
      }

      if (height_wrap_flag == 1){
        if (flock_of_boids[i].y >= int2fix15(480)){
          flock_of_boids[i].y -= int2fix15(480);
        }
        else if (flock_of_boids[i].y < int2fix15(0)){
          flock_of_boids[i].y += int2fix15(480);
        }
      }

      if (bias_flag) {
        if (i < scoup_1_num){
          if (flock_of_boids[i].vx > 0){
            flock_of_boids[i].biasval = min(maxbias, flock_of_boids[i].biasval + bias_increment);
          }
          else{
            flock_of_boids[i].biasval = max(bias_increment, flock_of_boids[i].biasval - bias_increment);
          }
        }else if (i > scoup_1_num && i < scoup_2_num){
          if (flock_of_boids[i].vx < 0){
            flock_of_boids[i].biasval = min(maxbias, flock_of_boids[i].biasval + bias_increment);
          }
          else{
            flock_of_boids[i].biasval = max(bias_increment, flock_of_boids[i].biasval - bias_increment);
          }
        }
      }

      // if (i < scoup_1_num){
      //   flock_of_boids[i].vx = flock_of_boids[i].vx - (flock_of_boids[i].vx >> 10) + flock_of_boids[i].biasval;
      // }
      // else if(i > scoup_1_num && i < scoup_2_num){
      //   flock_of_boids[i].vx = flock_of_boids[i].vx - (flock_of_boids[i].vx >> 10) - flock_of_boids[i].biasval;
      // }

      if (i < scoup_1_num){
        flock_of_boids[i].vx = multfix15(int2fix15(1) - flock_of_boids[i].biasval, flock_of_boids[i].vx) + flock_of_boids[i].biasval;
      }
      else if(i > scoup_1_num && i < scoup_2_num){
        flock_of_boids[i].vx = multfix15(int2fix15(1) - flock_of_boids[i].biasval, flock_of_boids[i].vx) - flock_of_boids[i].biasval;
      }

      flock_of_boids[i].x = flock_of_boids[i].x + flock_of_boids[i].vx ;
      flock_of_boids[i].y = flock_of_boids[i].y + flock_of_boids[i].vy ;
      
      //Draw each boid
      if(i < scoup_1_num){
        drawRect(fix2int15(flock_of_boids[i].x), fix2int15(flock_of_boids[i].y), 2, 2, RED);
      }
      else if(i > scoup_1_num && i < scoup_2_num){
        drawRect(fix2int15(flock_of_boids[i].x), fix2int15(flock_of_boids[i].y), 2, 2, GREEN);
      }
      else {
        drawRect(fix2int15(flock_of_boids[i].x), fix2int15(flock_of_boids[i].y), 2, 2, color);
      }
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

        sprintf(pt_serial_out_buffer, "input a height(0 for wrapping): ");
        // non-blocking write
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        // convert input string to number
        sscanf(pt_serial_in_buffer,"%d", &user_input_2) ;
        height_input = user_input_2;

        sprintf(pt_serial_out_buffer, "bias or not? (0/1) ");
        // non-blocking write
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        // convert input string to number
        sscanf(pt_serial_in_buffer,"%d", &bias_flag_input) ;
        bias_flag = bias_flag_input;

        sprintf(pt_serial_out_buffer, "input number of boids in scout group 1: ");
        // non-blocking write
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        // convert input string to number
        sscanf(pt_serial_in_buffer,"%d", &user_input_3) ;
        n_of_scout1_input = user_input_3;

        sprintf(pt_serial_out_buffer, "input number of boids in scout group 2: ");
        // non-blocking write
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        // convert input string to number
        sscanf(pt_serial_in_buffer,"%d", &user_input_3) ;
        n_of_scout2_input = user_input_3;

        sprintf(pt_serial_out_buffer, "input biasval for group 1: ");
        // non-blocking write
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        // convert input string to number
        sscanf(pt_serial_in_buffer,"%f", &user_input_4) ;
        biasval_input_1 = user_input_4;
        //bias_increment = float2fix15()

        sprintf(pt_serial_out_buffer, "input biasval for group 2: ");
        // non-blocking write
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        // convert input string to number
        sscanf(pt_serial_in_buffer,"%f", &user_input_4) ;
        biasval_input_2 = user_input_4;

        // update boid color
        if (width_input == 0) {
          old_width_wrap_flag = width_wrap_flag;
          width_wrap_flag = 1;
          old_arena_left = arena_left;
          old_arena_right = arena_right;
          arena_left = 0;
          arena_right = 0;
        }
        else if(width_input > 0 && width_input < 640){
          old_width_wrap_flag = width_wrap_flag;
          width_wrap_flag = 0;
          old_arena_left = arena_left;
          old_arena_right = arena_right;
          arena_left = 320 - width_input/2;
          arena_right = 320 + width_input/2;
        }

        // update boid color
        if (height_input == 0) {
          old_height_wrap_flag = height_wrap_flag;
          height_wrap_flag = 1;
          old_arena_top = arena_top;
          old_arena_bottom = arena_bottom;
          arena_top = -1;
          arena_bottom = -1;
        }
        else if(height_input > 0 && height_input < 480){
          old_height_wrap_flag = height_wrap_flag;
          height_wrap_flag = 0;
          old_arena_top = arena_top;
          old_arena_bottom = arena_bottom;
          arena_top = 240 - height_input/2;
          arena_bottom = 240 + height_input/2;
        }

        eraseArena();
        
        scoup_1_num = n_of_scout1_input;
        scoup_2_num = n_of_scout2_input + scoup_1_num;

        for (int i=0; i < scoup_1_num; i++) {
          flock_of_boids[i].biasval = float2fix15(biasval_input_1);
        }

        for (int i=scoup_1_num; i < scoup_2_num; i++) {
          flock_of_boids[i].biasval = float2fix15(biasval_input_2);
        }

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

      // erase boid
      // for(int i=0; i<NUM_BOIDS; i++){  
      //   drawRect(fix2int15(flock_of_boids[i].x), fix2int15(flock_of_boids[i].y), 2, 2, BLACK);    
      // }
      // update boid's position and velocity
      positionUpdate(flock_of_boids,0) ;
      // draw the boid at its new position  
      // for(int i=0; i<NUM_BOIDS; i++){
      //   drawRect(fix2int15(flock_of_boids[i].x), fix2int15(flock_of_boids[i].y), 2, 2, color); 
      // }
      // draw the boundaries
      // eraseArena();
      drawArena();
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
