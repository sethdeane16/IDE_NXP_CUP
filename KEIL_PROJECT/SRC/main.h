#ifndef  MAIN_H_
#define  MAIN_H_
void initialize(void);
void filter_main(uint16_t* camera_sig, int16_t* deriv_sig);
Struct left_right_index(int16_t* array, int size);
#endif  /*  ifndef  MAIN_H_  */
