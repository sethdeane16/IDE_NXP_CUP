#ifndef  FILTERS_H_
#define  FILTERS_H_
void der_convolve(uint16_t *x, int16_t *h, int16_t *y, int xSize, int hSize, int correction);
void convolve(uint16_t *x, int16_t *h, uint16_t *y, int xSize, int hSize, int correction);
void median_filter(uint16_t *x, uint16_t *y, int x_size);
#endif  /*  ifndef  FILTERS_H_  */
