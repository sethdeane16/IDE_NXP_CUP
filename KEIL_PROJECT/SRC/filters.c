
#include "MK64F12.h"

/* 
 * Function: median_filter
 * -----------------------
 *  Apply a median filter to get rid of any peaks in the signal
 *
 *  x: input array
 *  y: output array
 *  n: size of arrays
 *
 *  Returns: void
 */
void median_filter(uint16_t *x, uint16_t *y, int x_size) {
    // A three point median filter
    for (int i=0; i < x_size; i++) {

        // first element
        if (i == 0) {
            if(x[i] < x[i+1]) {
                y[i] = x[i];
            }
            else {
                y[i] = x[i+1];
            }
        }

        // last element
        else if(i == x_size - 1) {
            if(x[i] > x[i-1]) {
                y[i] = x[i];
            }
            else {
                y[i] = x[i-1];
            }
        }

        // middle elements
        else {
            if ((x[i-1] <= x[i]) && (x[i-1] <= x[i+1])) {
                y[i] = (x[i] <= x[i+1]) ? x[i] : x[i+1];
            }
            else if ((x[i] <= x[i-1]) && (x[i] <= x[i+1])) {
                y[i] = (x[i-1] <= x[i+1]) ? x[i-1] : x[i+1];
            }
            else {
                y[i] = (x[i-1] <= x[i]) ? x[i-1] : x[i];
            }
        }
    }
}


/* 
 * Function: convolve
 * ------------------
 *  Filters a 1D signal with the given inputs.
 *
 *  x: input array
 *  h: filter array
 *  y: output array
 *  xSize: length of x array
 *  hSize: length of h array
 *  correction: correction factor for given filter
 *              e.g. if filter is 1,2,1 correction is sum = 4
 *
 *  Returns: void
 */
void convolve(uint16_t *x, int16_t *h, uint16_t *y, int xSize, int hSize, int correction) {
    for (int i=(hSize-1);i <  xSize; i++)
    {
        double sum = 0.0;
        for (int j=hSize; j >=0; j--)
        {
            sum += h[j] * x[i-j];   //inner dot product
        }
        y[i] = sum / correction;
    }
}

/* 
 * Function: der_convolve
 * ----------------------
 *  Filters to the derivative of a given signal. Outputs a signed int array.
 *
 *  x: input array
 *  h: filter array
 *  y: output array
 *  xSize: length of x array
 *  hSize: length of h array
 *  correction: correction factor for given filter
 *              e.g. if filter is 1,2,1 correction is sum = 4
 *
 * Returns:
 *  void
 */
void der_convolve(uint16_t *x, int16_t *h, int16_t *y, int xSize, int hSize, int correction) {
    for (int i=(hSize-1);i <  xSize; i++)
    {
        double sum = 0.0;
        for (int j=hSize; j >=0; j--)
        {
            sum += h[j] * x[i-j];   //inner dot product
        }
        y[i] = sum / correction;
    }
}
