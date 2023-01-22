#pragma once

#include <iostream>

class PID
{
    public:
        /**
         * @param p Proportional Gain
         * @param i Integral Gain
         * @param d Derivative Gain
         * @param min Minimum Output
         * @param max Maximum Output
         */
        PID(double p, double i, double d, double min, double max){
            this->kP = p;
            this->kI = i;
            this->kD = d;
            this->minOutput = min;
            this->maxOutput = max;
        }

        /**
         * Basic PID calculator
         * @param setPoint its in the name
         * @param measurement current reading
         * @param dt time of loop
         * @return calculated voltage to motors
         */
        double calculate(double setPoint, double measuremant, double dt){
            error = setPoint - measuremant;
            sumError += ((error + prevError)/2)*dt;
            double derivError = (error - prevError)/dt;
            output = kP*error + kI*sumError + kD*derivError;

            prevError = error;
            if(output > maxOutput){
                output = maxOutput;
            }else if(output < minOutput){
                output = minOutput;
            }

            return output;
        }

        void setMinMaxOutputs(double min, double max){
            if(minOutput > maxOutput) {
                std::cout << "Youre stupid";
            }else{
                minOutput = min;
                maxOutput = max;
            }
        }

        void reset(){
            sumError = 0.;
            prevError = 0.;
            output = 0.;
        }


    private:
        double kP, kI, kD, kF;
        double error = 0.;
        double output = 0.;
        double sumError = 0.;
        double prevError = 0.;
        double minOutput = 0.;
        double maxOutput = 1.;
        
};