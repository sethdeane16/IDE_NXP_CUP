/*
 * Proportion Integration Derivative Control Code for car
 * Should get control speed based off previous what it will be doing next
 *
 * File:    pid.c
 * Authors: Seth Deane & Brian Powers
 * Created: April 6th 2019
 */

 /* SpeedControl
 * Description:
 *   Function that controls speed based off previous
 *
 * Parameters:
 *   void
 *
 * Returns:
 *   void
 */
// void SpeedControl()
// {
//     // Main
//     int CountOld = 0;
//     int MotorSpeed = 0;
//
//     Count = ReadEncoder();
//     Vact= (2*pi*r)*(Count – CountOld)*Hz/CountsPerRev;
//     if (Vact > Vhi)
//     {
//         MotorSpeed= 0;
//     }
//     else if(Vact < Vlo)
//     {
//         MotorSpeed= Kc;
//     }
//     // Main
//     CountOld= Count;
// }
//
//
// // Vdes and Vact need to be solved for before use
// void PID()
// {
//     // Main
//     MotorSpeedOld = 0;
//     ErrOld1 = 0;
//     ErrOld2 = 0;
//     Kp = 0.45;
//     Ki = 0.15;
//     Kd = 0.20;
//     Vdes = ; // Find normal speeds
//
//     Vact = ReadEncoder();
//     Err = Vdes - Vact;
//     MotorSpeed = MotorSpeedOld + Kp*(Err-ErrOld1) + Ki*(Err+ErrOld1)/2 +
//                 Kd*(Err - 2*ErrOld1 + ErrOld2);
//     // Clip???
//     MotorSpeed= clip(MotorSpeed,-100,+100)
//
//     // Main
//     MotorSpeedOld = MotorSpeed;
//     ErrOld2 = ErrOld1;
//     ErrOld1 = Err;
// }
