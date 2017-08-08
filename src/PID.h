#ifndef PID_H
#define PID_H

#include <vector>

class PID {
    public:
        /*
        * Errors
        */
        double p_error;
        double i_error;
        double d_error;
        //std::vector<double> E;
        
        /*
        * Coefficients
        */ 
        std::vector<double> K;
        
        /*
        * Constructor
        */
        PID();

        /*
        * Destructor.
        */
        virtual ~PID();

        /*
        * Initialize PID.
        */
        //void Init(std::vector<double> K, std::vector<double> dK);
        void Init(std::vector<double> K);

        /*
        * Update the PID error variables given cross track error.
        */
        void UpdateError(double cte, double delta_t);

        /*
        * Calculate the total PID error.
        */
        double TotalError();
};

#endif /* PID_H */
