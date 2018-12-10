#include "PID.h"
#include <algorithm>
#include <iostream>


using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, int n_step) {
    
    // initialize variables

    // PID values
    PID::Kp = Kp;
    PID::Ki = Ki;
    PID::Kd = Kd;

    // for error 
    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;
    prev_cte = 0.0;
    total_error = 0.0;
    best_error = numeric_limits<double>::max();

    // for twiddle
    param_index = 0;
    changes[0] = Kp*0.01;
    changes[1] = Ki*0.01;
    changes[2] = Kd*0.01;
    condition = false;

    // for loop count
    count = 0;

    // how many step will you consider for PID control?
    PID::n_step = n_step;

}

void PID::UpdateError(double cte) {

    p_error = cte;
    i_error += cte;
    store_cte.push_back(cte);

    // limit i_error 
    if (count > n_step){
        i_error -= store_cte[0];
    }

    d_error = cte - prev_cte;
    prev_cte = cte;

    // trigger for twiddle
    if (count <= n_step){
        count += 1;
    }

    TotalError();

}

double PID::TotalError() {

    total_error += prev_cte*prev_cte;

    // limit total_error
    if (count > n_step){
        total_error -= store_cte[0]*store_cte[0];
        store_cte.erase(store_cte.begin());
    }
    
}

double PID::SteerAngle(){

    return -1.0*(p_error*Kp + i_error*Ki + d_error*Kd);
}

void PID::Twiddle(){

    std::cout << "Twiddle is running, " << "Current index = " <<param_index << std::endl;
    std::cout << "Current PID values -> " << "Kp = " << Kp << ", Ki = " << Ki << ", Kd = " << Kd << std::endl;

    double parameters[3] = {Kp, Ki, Kd};
    double current_error = total_error / n_step;

    if (condition == false){
        parameters[param_index] += changes[param_index];
    }
    

    std::cout << "Following condition is satisfied: ";
    if (current_error <= best_error && condition == false){

        best_error = current_error;
        changes[param_index] *= 1.01;
        param_index = (param_index + 1) % 3;
        std::cout << "Condition 1" << std::endl;

    }
    else if (current_error > best_error && condition == false){

        parameters[param_index] -= 2*changes[param_index];
        condition = true;
        std::cout << "Condition 2" << std::endl;

    }
    else if (condition == true){

        if (current_error < best_error){

            best_error = current_error;
            changes[param_index] *= 1.01;
            param_index = (param_index + 1) % 3;
            std::cout << "Condition 3" << std::endl;

        }
        else{
            
            parameters[param_index] += changes[param_index];
            changes[param_index] *= 0.99;
            std::cout << "Condition 4" << std::endl;

        }

        condition = false;

    }

    // update PID
    Kp = parameters[0];
    Ki = parameters[1];
    Kd = parameters[2];
    std::cout << "After Twiddle PID values -> " << "Kp = " << Kp << ", Ki = " << Ki << ", Kd = " << Kd << std::endl;

}