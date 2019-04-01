#ifndef PWM_H_
#define PWM_H_

void SetMotorDutyCycleL(unsigned int DutyCycle, unsigned int Frequency, int dir);
void SetMotorDutyCycleR(unsigned int DutyCycle, unsigned int Frequency, int dir);
void SetServoDutyCycle(double DutyCycle);
void init_PWM(void);
void PWM_ISR(void);

#endif /* PWM_H_ */
