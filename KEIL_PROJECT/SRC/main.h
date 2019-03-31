#ifndef  MAIN_H_
#define  MAIN_H_
void initialize();
void en_interrupts();
void delay();
void weighted_filter(uint16_t *x,uint16_t *h, uint16_t *y, int xSize,  int hSize);
void deriva_filter(uint16_t *x,int *h, int16_t *y, int xSize,  int hSize);
void turnServo(double angle);
#endif  /*  ifndef  MAIN_H_  */
