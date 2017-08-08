#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
//#include <math.h>
//#include <vector>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_last_of("]");
    if (found_null != std::string::npos) {
        return "";
    }
    else if (b1 != std::string::npos && b2 != std::string::npos) {
        return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}

int main()
{
    uWS::Hub h;
    
    PID pid;
    
    // Time variables
    double time_cur = 0.0;
    std::cout << "Twiddle? (yes/no)" << std::endl;
    double time_prv = clock() / 1000;
    double time_tot = 0.0;
    
    // Run twiddle or not
    bool twiddle = false;
    
    // Twiddle parameters
    // Counter of web sockets events
    int twd_run = 0;
    // Interval between twiddle checkpoints
    const int twd_run_itv = 15;
    // Interval before switching to next hyper-parameter
    const int twd_prm_itv = 6;
    // Twiddle cursor for checkpoints (0, 1, 2, 3)
    int twd_check = -1;
    // Twiddle variation in cursor
    int twd_check_d = 1;
    // Current cumulated squared CTE
    double cur_error = 0.0;
    // Best cumulated squared CTE
    double best_error = 0.0;
    
    // Hyper-parameter index being "twiddled"
    int twd_index = 0;
    // Twiddle gain: inversed of normalized dot product of incremental variation vector
    double twd_gain = 1.0;
    // Previous twiddle gain
    double twd_gain_prv = 1.0;
    // Tolerance
    const double twd_tolerance = 6;
    
    // Prompting for twiddle
    std::string twiddle_s;
    std::cout << "Twiddle? (yes/no)" << std::endl;
    std::cin >> twiddle_s;
    if (twiddle_s == "yes") {twiddle = true;} else {twiddle = false;}
    
    // Initializing vector of parameters
    double Kp_init;
    double Ki_init;
    double Kd_init;
    std::cout << "Initial P, I and D?" << std::endl;
    std::cin >> Kp_init;
    std::cin >> Ki_init;
    std::cin >> Kd_init;
    //pid.Init({0.1, 0.0002, 3.0});
    //pid.Init({0.1331, 0.000201008, 3.0});
    pid.Init({Kp_init, Ki_init, Kd_init});
    std::cout << "initial pid.K:" << std::endl;
    for (unsigned int i = 0; i < pid.K.size(); i++) {
        std::cout << "   pid.K[" << i << "] = " << pid.K[i] << std::endl;
    }
    
    // Incremental twiddle variations for each hyper-parameter
    std::vector<double> twd_dK(pid.K.size());
    // Weights to normalize incremental parameter variations (constant)
    std::vector<double> twd_weights(pid.K.size());
    // Incremental twiddle variation factor (constant)
    std::vector<double> twd_ddK(pid.K.size());
    
    // Throttle default values: regular mode or twiddle mode
    double throt_value_dflt;
    double throt_value_twd;
    
    if (twiddle) {
        // Initializing vector of incremental variations for twiddle
        std::cout << "initial twd_dK:" << std::endl;
        for (unsigned int i = 0; i < pid.K.size(); i++) {
            if (pid.K[i] == 0) {
                // Defaulting to variation 1
                twd_dK[i] = 1.0;
            } 
            else {
                // Initializing to one more decimal than initial value
                twd_dK[i] = pow(10.0, floor(log10(pid.K[i])) - 1);
            }
            std::cout << "   twd_dK[" << i << "] = " << twd_dK[i] << std::endl;
        }
        
        // Initializing vector of weights for twiddle
        std::cout << "initial weights:" << std::endl;
        for (unsigned int i = 0; i < twd_dK.size(); i++) {
            twd_weights[i] = 1 / twd_dK[i];
            std::cout << "   twd_weights[" << i << "] = " << twd_weights[i] << std::endl;
        }
        
        // Initializing vector of variation factor for twiddle
        std::cout << "initial twd_ddK:" << std::endl;
        for (unsigned int i = 0; i < twd_dK.size(); i++) {
            twd_ddK[i] = 1.1;
            std::cout << "   twd_ddK[" << i << "] = " << twd_ddK[i] << std::endl;
        }
        
        // Initializing speed while running twiddle
        std::string throt_value_twd_s;
        std::cout << "throttle value when running twiddle? ([0,1])" << std::endl;
        std::cin >> throt_value_twd_s;
        throt_value_twd = stod(throt_value_twd_s);
    }
    
    // Initializing speed in normal mode
    std::string throt_value_dflt_s;
    std::cout << "throttle value when running normal mode? ([0,1])" << std::endl;
    std::cin >> throt_value_dflt_s;
    throt_value_dflt = stod(throt_value_dflt_s);
    
    h.onMessage([&pid, &time_cur, &time_prv, &time_tot, &twiddle, &twd_run, &twd_run_itv, &twd_prm_itv, &twd_check, &twd_check_d, &cur_error, &best_error, &twd_dK, &twd_weights, &twd_ddK, &twd_index, &twd_gain, &twd_gain_prv, &twd_tolerance, &throt_value_dflt, &throt_value_twd](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        
        // Attempt to record images from simulator... Any suggestion welcome!
        //std::cout << "strlen(data) = " << strlen(data) << std::endl;
        //for (unsigned int i = 0; i < 10; i++) {
            //std::cout << "data[" << i << "] = " << data[i] << std::endl;
        //}
        
        if (length && length > 2 && data[0] == '4' && data[1] == '2')
        {
            auto s = hasData(std::string(data).substr(0, length));
            if (s != "") {
                auto j = json::parse(s);
                std::string event = j[0].get<std::string>();
                if (event == "telemetry") {
                    // j[1] is the data JSON object
                    double cte = std::stod(j[1]["cte"].get<std::string>());
                    double steer_value;
                    double throt_value;
                    double dot_twd_dK;
                    bool adjust_prm;
                    double twd_dK_prv;
                    
                    // 
                    twd_run += 1;
                    
                    // Getting time delta for error update and total time for twiddle
                    time_cur = clock() / 1000;
                    //double delta_t = 1.0;
                    double delta_t = time_cur - time_prv;
                    time_tot += time_cur;
                    time_prv = time_cur;
                    
                    // Updating errors
                    pid.UpdateError(cte, delta_t);
                    steer_value = -pid.TotalError();
                    
                    // Keeping steer_value in [-1,1]
                    if (steer_value > 1) steer_value = 1;
                    if (steer_value < -1) steer_value = -1;
                    
                    // Initializing speed to twiddle value
                    if (twiddle && best_error == 0.0) {
                        throt_value = throt_value_twd;
                    }
                    
                    // Accumulating squared error for twiddle
                    if (twiddle) {cur_error += pow(cte, 2);}
                    
                    // Twiddle checkpoint, every 15 web socket event
                    if (twiddle && (twd_gain < twd_tolerance) && (twd_run % twd_run_itv == 0)) {
                        std::cout << "****** twiddle checkpoint "<< "@" << twd_run << " : twd_check = " << twd_check << " : twd_gain = " << twd_gain << std::endl;
                        
                        // Switching parameter, every 6*15=90 web socket event
                        if (twd_run % (twd_prm_itv * twd_run_itv) == 0) {
                            std::cout << "************************************" << std::endl;
                            
                            // Finishing processing current parameter before switching
                            std::cout << "adjust_prm = " << adjust_prm << std::endl;
                            if (adjust_prm) {
                                // Adjusting parameter
                                pid.K[twd_index] += twd_dK[twd_index];
                                std::cout << " -> adjusting pid.K[" << twd_index << "] = " << pid.K[twd_index] << " with " << "twd_dK[" << twd_index << "] = " << twd_dK[twd_index] << std::endl;
                                
                            }
                            
                            // Switching to next parameter
                            twd_index = (twd_index + 1) % (twd_dK.size());
                            
                            // Resetting twiddle checkpoint
                            twd_check = 0; //1??
                            twd_check_d = 1;
                        }
                        
                        // Incrementing twiddle check index
                        twd_check += twd_check_d;
                        
                        // Initializing flag for parameter adjustement
                        adjust_prm = false;
                        
                        // 
                        if (twd_check == 0) {
                            
                            std::cout << "*** run 0" << std::endl;
                            // Initializing best error
                            best_error = cur_error;
                            std::cout << "initializing best_error = " << best_error << std::endl;
                            
                            // Adjusting parameter
                            pid.K[twd_index] += twd_dK[twd_index];
                            std::cout << "adjusting pid.K[" << twd_index << "] = " << pid.K[twd_index] << " with " << "twd_dK[" << twd_index << "] = " << twd_dK[twd_index] << std::endl;
                            
                            // Sending data to #2 for next twiddle checkpoint
                            twd_check_d = 2;
                            
                        }
                        
                        //
                        if (twd_check == 1) {
                            std::cout << "*** run 1" << std::endl;
                            
                            // Adjusting parameter
                            pid.K[twd_index] += twd_dK[twd_index];
                            std::cout << " -> adjusting pid.K[" << twd_index << "] = " << pid.K[twd_index] << " with " << "twd_dK[" << twd_index << "] = " << twd_dK[twd_index] << std::endl;
                            
                            // Sending data to #2 for next twiddle checkpoint
                            twd_check_d = 1;
                            
                        }
                        
                        //
                        if (twd_check == 2) {
                            std::cout << "*** run 2" << std::endl;
                            std::cout << "cur_error = " << cur_error << std::endl;
                            std::cout << "best_error = " << best_error << std::endl;
                            if (cur_error < best_error) {
                                // Updating best error
                                best_error = cur_error;
                                
                                // Narrowing down delta
                                twd_dK_prv = twd_dK[twd_index];
                                twd_dK[twd_index] *= twd_ddK[twd_index];
                                std::cout << "   -> param up: twd_dK[" << twd_index << "] = " << twd_dK[twd_index] << " = " << twd_dK_prv << " * (" << twd_ddK[twd_index] << ")" << std::endl;
                                
                                // Recomputing gain
                                dot_twd_dK = std::inner_product(twd_dK.begin(), twd_dK.end(), twd_weights.begin(), 0.0) / twd_dK.size();
                                twd_gain = 1 / dot_twd_dK;
                                if (twd_gain < 1) {
                                    twd_gain = 1;
                                }
                                
                                // Adjusting parameter
                                pid.K[twd_index] += twd_dK[twd_index];
                                std::cout << "   -> adjusting pid.K[" << twd_index << "] = " << pid.K[twd_index] << " with " << "twd_dK[" << twd_index << "] = " << twd_dK[twd_index] << std::endl;
                                
                                // Sending data to #2 for next twiddle checkpoint
                                twd_check_d = 0;
                            }
                            else {
                                // Reverting change
                                pid.K[twd_index] -= 2 * twd_dK[twd_index];
                                std::cout << "   -> reverting : pid.K[" << twd_index << "] = " << pid.K[twd_index] << " with " << "twd_dK[" << twd_index << "] = " << twd_dK[twd_index] << std::endl;
                                
                                // Sending data to #3 for next twiddle checkpoint
                                twd_check_d = 1;
                                
                                // Opening flag for parameter adjustement
                                adjust_prm = true;
                            }
                            
                        }
                        
                        //
                        if (twd_check == 3) {
                            std::cout << "*** run 3" << std::endl;
                            std::cout << "cur_error = " << cur_error << std::endl;
                            std::cout << "best_error = " << best_error << std::endl;
                            
                            //
                            if (cur_error < best_error) {
                                // Updating best error
                                best_error = cur_error;
                                
                                // Narrowing down delta
                                twd_dK_prv = twd_dK[twd_index];
                                twd_dK[twd_index] *= twd_ddK[twd_index];
                                std::cout << "     -> param up: twd_dK[" << twd_index << "] = " << twd_dK[twd_index] << " = " << twd_dK_prv << " * (" << twd_ddK[twd_index] << ")" << std::endl;
                                
                                // Recomputing gain
                                dot_twd_dK = std::inner_product(twd_dK.begin(), twd_dK.end(), twd_weights.begin(), 0.0) / twd_dK.size();
                                twd_gain = 1 / dot_twd_dK;
                                
                                // Opening flag for parameter adjustement
                                adjust_prm = true;
                            }
                            else {
                                // Adjusting parameter
                                pid.K[twd_index] += twd_dK[twd_index];
                                std::cout << "     -> adjusting pid.K[" << twd_index << "] = " << pid.K[twd_index] << " with " << "twd_dK[" << twd_index << "] = " << twd_dK[twd_index] << std::endl;
                                
                                // Decreasing delta
                                twd_dK_prv = twd_dK[twd_index];
                                twd_dK[twd_index] *= (2 - twd_ddK[twd_index]);
                                std::cout << "     -> param down: twd_dK[" << twd_index << "] = " << twd_dK[twd_index] << " = " << twd_dK_prv << " * (" << 2 - twd_ddK[twd_index] << ")" << std::endl;
                                
                                // Recomputing gain
                                dot_twd_dK = std::inner_product(twd_dK.begin(), twd_dK.end(), twd_weights.begin(), 0.0) / twd_dK.size();
                                twd_gain = 1 / dot_twd_dK;
                            }
                            
                            // Sending data to #1 for next checkpoint
                            twd_check = 0;
                            twd_check_d = 1;
                        }
                        
                        // Reinitializing cumulated squared error for next round
                        cur_error = 0.0;
                        
                        // Initializing speed
                        if (twd_check <= 0) {
                            throt_value = throt_value_twd;
                        }
                        std::cout << twd_gain << " vs. " << twd_gain_prv << std::endl;
                        
                        // If gain over 1.5 and increasing, speeding up a bit
                        if (twd_gain > 1.5 && twd_gain != twd_gain_prv) {
                            if (twd_gain > twd_gain_prv) {
                                // Increasing twiddle speed
                                throt_value *= 1.001;
                                std::cout << "increasing speed at " << throt_value << std::endl;
                            }
                            else {
                                // Decreasing twiddle speed
                                throt_value *= 0.999;
                                //throt_value *= 0.952;
                                std::cout << "decreasing speed at " << throt_value << std::endl;
                            }
                            
                            // Keeping throt_value in [min,max]
                            if (throt_value < throt_value_twd) {
                                throt_value = throt_value_twd;
                            }
                            if (throt_value > throt_value_dflt) {
                                throt_value = throt_value_dflt;
                            }
                        }
                        
                        // Resetting prior gain
                        twd_gain_prv = twd_gain;
                        
                    }
                    
                    // If normal mode (no twiddle) or optimal parameters found
                    else {
                        if (twiddle) {
                            if (twd_gain >= twd_tolerance) {
                                throt_value = throt_value_dflt;
                                std::cout << "### final parameters found: " << std::endl;
                                for (unsigned int i = 0; i < pid.K.size(); i++) {
                                    std::cout << "  -> pid.K[" << i << "] = " << pid.K[i] << std::endl;
                                }
                            }
                        }
                        else {
                            throt_value = throt_value_dflt;
                        }
                    }
                    
                    // DEBUG
                    std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " Throttle Value: " << throt_value << std::endl;
                    
                    json msgJson;
                    msgJson["steering_angle"] = steer_value;
                    msgJson["throttle"] = throt_value;
                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    //std::cout << msg << std::endl;
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });
    
    // We don't need this since we're not using HTTP but if it's removed the program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1)
        {
            res->end(s.data(), s.length());
        }
        else
        {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });
    
    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });
    
    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });
    
    int port = 4567;
    if (h.listen(port))
    {
        std::cout << "Listening to port " << port << std::endl;
    }
    else
    {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    } 
    h.run();
}
