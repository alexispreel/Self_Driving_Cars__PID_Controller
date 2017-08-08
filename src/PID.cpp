#include <iostream>
#include "PID.h"

using namespace std;

PID::PID() {}

PID::~PID() {}

//void PID::Init(std::vector<double> K, std::vector<double> dK) {
void PID::Init(std::vector<double> K) {
    
    // 
    this->K = K;
    
    // 
    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;
    
}

void PID::UpdateError(double cte, double delta_t) {
    //
    i_error += cte;
    //std::cout << "cte - p_error = " << cte << " - " << p_error << " = " << cte - p_error << std::endl;
    //std::cout << "delta_t = " << delta_t << std::endl;
    d_error = (cte - p_error) / delta_t;
    p_error = cte;
}

double PID::TotalError() {
    //
    return K[0] * p_error + K[1] * i_error + K[2] * d_error;
}