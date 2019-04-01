#ifndef  MAIN_H_
#define  MAIN_H_
void initialize(void);
void median_filter(uint16_t *x, uint16_t *y, int x_size);
void convolve(uint16_t *x, int16_t *h, uint16_t *y, int xSize, int hSize, int correction);
void der_convolve(uint16_t *x, int16_t *h, int16_t *y, int xSize, int hSize, int correction);
void TurnCar(uint16_t index);
#endif  /*  ifndef  MAIN_H_  */
