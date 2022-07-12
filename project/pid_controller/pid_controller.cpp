/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
    Kp = Kpi;
    Ki = Kii;
    Kd = Kdi;
    output_lim_max = output_lim_maxi;
    output_lim_min= output_lim_mini;

    pro_cte = 0.0;
    int_cte = 0.0;
    diff_cte = 0.0;
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
    double prev_cte = pro_cte;
    if(delta_time == 0.0)
        diff_cte = std::numeric_limits<double>::infinity();
    else
        diff_cte = (cte - prev_cte)/delta_time;

    int_cte += cte*delta_time;
    
    pro_cte = cte;
    
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    double control = -Kp * pro_cte - Ki * int_cte - Kd * diff_cte;

    if(output_lim_min < control && control < output_lim_max)
        return control;
    else if (control <= output_lim_min)
        return output_lim_min;
    else
        return output_lim_max;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
    delta_time = new_delta_time;
}