/*
 * radar_vecs.c
 *
 *  Created on: Feb 20, 2014
 *      Author: petera
 */
typedef struct {
  s8_t x1,y1,x2,y2;
  s8_t dx, dy;
} radar_vec_t;

radar_vec_t radar_vecs[64] = {
    {    .x1= -25,    .y1=  -4,    .x2=  -6,    .y2=  19,   .dx=  19,   .dy=  23},
    {    .x1= -24,    .y1=  -4,    .x2=  -5,    .y2=  19,   .dx=  19,   .dy=  23},
    {    .x1= -23,    .y1=  -4,    .x2=  -5,    .y2=  19,   .dx=  18,   .dy=  23},
    {    .x1= -23,    .y1=  -5,    .x2=  -5,    .y2=  19,   .dx=  18,   .dy=  24},
    {    .x1= -22,    .y1=  -5,    .x2=  -5,    .y2=  19,   .dx=  17,   .dy=  24},
    {    .x1= -21,    .y1=  -6,    .x2=  -5,    .y2=  19,   .dx=  16,   .dy=  25},
    {    .x1= -20,    .y1=  -6,    .x2=  -5,    .y2=  19,   .dx=  15,   .dy=  25},
    {    .x1= -20,    .y1=  -6,    .x2=  -5,    .y2=  19,   .dx=  15,   .dy=  25},
    {    .x1= -19,    .y1=  -6,    .x2=  -4,    .y2=  19,   .dx=  15,   .dy=  25},
    {    .x1= -18,    .y1=  -7,    .x2=  -4,    .y2=  19,   .dx=  14,   .dy=  26},
    {    .x1= -17,    .y1=  -7,    .x2=  -4,    .y2=  19,   .dx=  13,   .dy=  26},
    {    .x1= -17,    .y1=  -7,    .x2=  -4,    .y2=  19,   .dx=  13,   .dy=  26},
    {    .x1= -16,    .y1=  -8,    .x2=  -4,    .y2=  19,   .dx=  12,   .dy=  27},
    {    .x1= -15,    .y1=  -8,    .x2=  -3,    .y2=  18,   .dx=  12,   .dy=  26},
    {    .x1= -14,    .y1=  -8,    .x2=  -3,    .y2=  18,   .dx=  11,   .dy=  26},
    {    .x1= -14,    .y1=  -8,    .x2=  -3,    .y2=  18,   .dx=  11,   .dy=  26},
    {    .x1= -13,    .y1=  -8,    .x2=  -3,    .y2=  18,   .dx=  10,   .dy=  26},
    {    .x1= -12,    .y1=  -9,    .x2=  -3,    .y2=  18,   .dx=   9,   .dy=  27},
    {    .x1= -11,    .y1=  -9,    .x2=  -3,    .y2=  18,   .dx=   8,   .dy=  27},
    {    .x1= -10,    .y1=  -9,    .x2=  -2,    .y2=  18,   .dx=   8,   .dy=  27},
    {    .x1= -10,    .y1=  -9,    .x2=  -2,    .y2=  18,   .dx=   8,   .dy=  27},
    {    .x1=  -9,    .y1=  -9,    .x2=  -2,    .y2=  18,   .dx=   7,   .dy=  27},
    {    .x1=  -8,    .y1=  -9,    .x2=  -2,    .y2=  18,   .dx=   6,   .dy=  27},
    {    .x1=  -7,    .y1= -10,    .x2=  -2,    .y2=  18,   .dx=   5,   .dy=  28},
    {    .x1=  -6,    .y1= -10,    .x2=  -1,    .y2=  18,   .dx=   5,   .dy=  28},
    {    .x1=  -6,    .y1= -10,    .x2=  -1,    .y2=  18,   .dx=   5,   .dy=  28},
    {    .x1=  -5,    .y1= -10,    .x2=  -1,    .y2=  18,   .dx=   4,   .dy=  28},
    {    .x1=  -4,    .y1= -10,    .x2=  -1,    .y2=  18,   .dx=   3,   .dy=  28},
    {    .x1=  -3,    .y1= -10,    .x2=  -1,    .y2=  18,   .dx=   2,   .dy=  28},
    {    .x1=  -2,    .y1= -10,    .x2=  -1,    .y2=  18,   .dx=   1,   .dy=  28},
    {    .x1=  -2,    .y1= -10,    .x2=   0,    .y2=  18,   .dx=   2,   .dy=  28},
    {    .x1=  -1,    .y1= -10,    .x2=   0,    .y2=  18,   .dx=   1,   .dy=  28},
    {    .x1=   0,    .y1= -10,    .x2=   0,    .y2=  18,   .dx=   0,   .dy=  28},
    {    .x1=   1,    .y1= -10,    .x2=   0,    .y2=  18,   .dx=  -1,   .dy=  28},
    {    .x1=   2,    .y1= -10,    .x2=   0,    .y2=  18,   .dx=  -2,   .dy=  28},
    {    .x1=   2,    .y1= -10,    .x2=   1,    .y2=  18,   .dx=  -1,   .dy=  28},
    {    .x1=   3,    .y1= -10,    .x2=   1,    .y2=  18,   .dx=  -2,   .dy=  28},
    {    .x1=   4,    .y1= -10,    .x2=   1,    .y2=  18,   .dx=  -3,   .dy=  28},
    {    .x1=   5,    .y1= -10,    .x2=   1,    .y2=  18,   .dx=  -4,   .dy=  28},
    {    .x1=   6,    .y1= -10,    .x2=   1,    .y2=  18,   .dx=  -5,   .dy=  28},
    {    .x1=   6,    .y1= -10,    .x2=   1,    .y2=  18,   .dx=  -5,   .dy=  28},
    {    .x1=   7,    .y1= -10,    .x2=   2,    .y2=  18,   .dx=  -5,   .dy=  28},
    {    .x1=   8,    .y1=  -9,    .x2=   2,    .y2=  18,   .dx=  -6,   .dy=  27},
    {    .x1=   9,    .y1=  -9,    .x2=   2,    .y2=  18,   .dx=  -7,   .dy=  27},
    {    .x1=  10,    .y1=  -9,    .x2=   2,    .y2=  18,   .dx=  -8,   .dy=  27},
    {    .x1=  10,    .y1=  -9,    .x2=   2,    .y2=  18,   .dx=  -8,   .dy=  27},
    {    .x1=  11,    .y1=  -9,    .x2=   3,    .y2=  18,   .dx=  -8,   .dy=  27},
    {    .x1=  12,    .y1=  -9,    .x2=   3,    .y2=  18,   .dx=  -9,   .dy=  27},
    {    .x1=  13,    .y1=  -8,    .x2=   3,    .y2=  18,   .dx= -10,   .dy=  26},
    {    .x1=  14,    .y1=  -8,    .x2=   3,    .y2=  18,   .dx= -11,   .dy=  26},
    {    .x1=  14,    .y1=  -8,    .x2=   3,    .y2=  18,   .dx= -11,   .dy=  26},
    {    .x1=  15,    .y1=  -8,    .x2=   3,    .y2=  18,   .dx= -12,   .dy=  26},
    {    .x1=  16,    .y1=  -8,    .x2=   4,    .y2=  19,   .dx= -12,   .dy=  27},
    {    .x1=  17,    .y1=  -7,    .x2=   4,    .y2=  19,   .dx= -13,   .dy=  26},
    {    .x1=  17,    .y1=  -7,    .x2=   4,    .y2=  19,   .dx= -13,   .dy=  26},
    {    .x1=  18,    .y1=  -7,    .x2=   4,    .y2=  19,   .dx= -14,   .dy=  26},
    {    .x1=  19,    .y1=  -6,    .x2=   4,    .y2=  19,   .dx= -15,   .dy=  25},
    {    .x1=  20,    .y1=  -6,    .x2=   5,    .y2=  19,   .dx= -15,   .dy=  25},
    {    .x1=  20,    .y1=  -6,    .x2=   5,    .y2=  19,   .dx= -15,   .dy=  25},
    {    .x1=  21,    .y1=  -6,    .x2=   5,    .y2=  19,   .dx= -16,   .dy=  25},
    {    .x1=  22,    .y1=  -5,    .x2=   5,    .y2=  19,   .dx= -17,   .dy=  24},
    {    .x1=  23,    .y1=  -5,    .x2=   5,    .y2=  19,   .dx= -18,   .dy=  24},
    {    .x1=  23,    .y1=  -4,    .x2=   5,    .y2=  19,   .dx= -18,   .dy=  23},
    {    .x1=  24,    .y1=  -4,    .x2=   5,    .y2=  19,   .dx= -19,   .dy=  23},
};
