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
    
    // 
    double time_cur = 0.0;
    double time_prv = clock();
    double time_tot = 0.0;
    
    //
    //bool twiddle = true;
    int twd_run = 0;
    const int twd_run_itv = 15;
    const int twd_prm_itv = 6;
    int twd_check = -1;
    int twd_check_d = 1;
    double cur_error = 0.0;
    double best_error = 0.0;
    double twd_gain = 1.0;
    double twd_gain_prv = 1.0;
    
    // Initializing parameters
    //pid.Init({0.134611, 0.000270736, 3.05349});
    pid.Init({0.1, 0.0002, 3.0});
    
    // 
    std::vector<double> twd_dK{0.01, 0.00001, 0.1};         // parameter variations
    //std::vector<double> twd_dK{0.1, 0.0001, 1.0};         // parameter variations
    const std::vector<double> twd_weights{1 / twd_dK[0], 1 / twd_dK[1], 1 / twd_dK[2]};   // to normalize parameter variations
    //const double dot_twd_dK_0 = std::inner_product(twd_dK.begin(), twd_dK.end(), twd_weights.begin(), 0.0);     // initialized as number of parameters (here 3)
    //std::cout << "dot_twd_dK_0 = " << dot_twd_dK_0 << std::endl;
    //std::vector<double> twd_ddK{1.3, 1.3, 1.3};       // parameter variation factor
    const std::vector<double> twd_ddK{1.1, 1.1, 1.1};       // parameter variation factor
    int twd_index = 0;
    const double twd_tolerance = 3;
    
    //h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    //h.onMessage([&pid, &time_cur, &time_prv, &time_tot, &twd_dK, &twd_weights, &dot_twd_dK_0, &twd_ddK, &twd_index, &twd_tolerance, &twd_run, &twd_run_itv, &twd_prm_itv, &cur_error, &best_error, &twd_check, &twd_check_d](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    h.onMessage([&pid, &time_cur, &time_prv, &time_tot, &twd_dK, &twd_weights, &twd_ddK, &twd_index, &twd_tolerance, &twd_run, &twd_run_itv, &twd_prm_itv, &cur_error, &best_error, &twd_check, &twd_check_d, &twd_gain, &twd_gain_prv](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2')
        {
            auto s = hasData(std::string(data).substr(0, length));
            if (s != "") {
                auto j = json::parse(s);
                std::string event = j[0].get<std::string>();
                if (event == "telemetry") {
                    // j[1] is the data JSON object
                    double cte = std::stod(j[1]["cte"].get<std::string>());
                    //double speed = std::stod(j[1]["speed"].get<std::string>());
                    //double angle = std::stod(j[1]["steering_angle"].get<std::string>());
                    double steer_value;
                    double throt_value;
                    //double sum_twd_dK = std::accumulate(twd_dK.begin(), twd_dK.end(), 0.0);
                    double dot_twd_dK;
                    //double twd_gain;
                    //std::cout << "twd_gain: " << twd_gain << " = " << dot_twd_dK_0 << " / " << dot_twd_dK << std::endl;
                    bool adjust_prm;
                    double twd_dK_prv;
                    
                    // 
                    twd_run += 1;
                    
                    // Getting time delta for error update and total time for twiddle
                    time_cur = clock();
                    double delta_t = time_cur - time_prv;
                    time_tot += time_cur;
                    time_prv = time_cur;
                    
                    // Updating errors
                    pid.UpdateError(cte, delta_t);
                    steer_value = -pid.TotalError();
                    
                    // Keeping steer_value in [-1,1]
                    if (steer_value > 1) steer_value = 1;
                    if (steer_value < -1) steer_value = -1;
                    
                    // Accumulating squared error for twiddle
                    cur_error += pow(cte, 2);
                    
                    // Twiddle checkpoint
                    if ((twd_gain < twd_tolerance) && (twd_run % twd_run_itv == 0)) {
                        std::cout << "****** twiddle checkpoint "<< "@" << twd_run << " : twd_check = " << twd_check << " : twd_gain = " << twd_gain << std::endl;
                        
                        //
                        //dot_twd_dK = std::inner_product(twd_dK.begin(), twd_dK.end(), twd_weights.begin(), 0.0) / twd_dK.size();
                        //twd_gain = 1 / dot_twd_dK;
                        
                        //
                        if (twd_run % (twd_prm_itv * twd_run_itv) == 0) {
                            std::cout << "************************************" << std::endl;
                            
                            //
                            std::cout << "adjust_prm = " << adjust_prm << std::endl;
                            if (adjust_prm) {
                                // Adjusting parameter
                                pid.K[twd_index] += twd_dK[twd_index];
                                std::cout << " -> adjusting pid.K[" << twd_index << "] = " << pid.K[twd_index] << " with " << "twd_dK[" << twd_index << "] = " << twd_dK[twd_index] << std::endl;
                                
                            }
                            
                            // Refining variation factor
                            // not sure about that...
                            //twd_ddK[twd_index] *= 0.9;
                            
                            // Switching to next parameter
                            twd_index = (twd_index + 1) % (twd_dK.size());
                            
                            // Resetting twiddle checkpoint
                            twd_check = 0; //1??
                            twd_check_d = 1;
                        }
                        
                        // 
                        twd_check += twd_check_d;
                        
                        // 
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
                            
                            // 
                            twd_check_d = 2;
                            
                            // Let run...
                            //cur_error = 0.0;
                        }
                        
                        //
                        if (twd_check == 1) {
                            std::cout << "*** run 1" << std::endl;
                            
                            // Adjusting parameter
                            pid.K[twd_index] += twd_dK[twd_index];
                            std::cout << " -> adjusting pid.K[" << twd_index << "] = " << pid.K[twd_index] << " with " << "twd_dK[" << twd_index << "] = " << twd_dK[twd_index] << std::endl;
                            
                            //
                            twd_check_d = 1;
                            
                            // Let run...
                            //cur_error = 0.0;
                        }
                        
                        //
                        if (twd_check == 2) {
                            std::cout << "*** run 2" << std::endl;
                            std::cout << "cur_error = " << cur_error << std::endl;
                            std::cout << "best_error = " << best_error << std::endl;
                            if (cur_error < best_error) {
                                //
                                best_error = cur_error;
                                
                                //
                                twd_dK_prv = twd_dK[twd_index];
                                twd_dK[twd_index] *= twd_ddK[twd_index];
                                std::cout << "   -> param up: twd_dK[" << twd_index << "] = " << twd_dK[twd_index] << " = " << twd_dK_prv << " * (" << twd_ddK[twd_index] << ")" << std::endl;
                                
                                //
                                dot_twd_dK = std::inner_product(twd_dK.begin(), twd_dK.end(), twd_weights.begin(), 0.0) / twd_dK.size();
                                twd_gain = 1 / dot_twd_dK;
                                
                                // Adjusting parameter
                                pid.K[twd_index] += twd_dK[twd_index];
                                std::cout << "   -> adjusting pid.K[" << twd_index << "] = " << pid.K[twd_index] << " with " << "twd_dK[" << twd_index << "] = " << twd_dK[twd_index] << std::endl;
                                
                                //
                                //twd_check -= 1;
                                twd_check_d = 0;
                            }
                            else {
                                // Reverting change
                                pid.K[twd_index] -= 2 * twd_dK[twd_index];
                                std::cout << "   -> reverting : pid.K[" << twd_index << "] = " << pid.K[twd_index] << " with " << "twd_dK[" << twd_index << "] = " << twd_dK[twd_index] << std::endl;
                                
                                //
                                twd_check_d = 1;
                                adjust_prm = true;
                            }
                            
                            // Let run...
                            //cur_error = 0.0;
                        }
                        
                        //
                        if (twd_check == 3) {
                            std::cout << "*** run 3" << std::endl;
                            std::cout << "cur_error = " << cur_error << std::endl;
                            std::cout << "best_error = " << best_error << std::endl;
                            
                            //
                            if (cur_error < best_error) {
                                //
                                best_error = cur_error;
                                
                                // 
                                twd_dK_prv = twd_dK[twd_index];
                                twd_dK[twd_index] *= twd_ddK[twd_index];
                                std::cout << "     -> param up: twd_dK[" << twd_index << "] = " << twd_dK[twd_index] << " = " << twd_dK_prv << " * (" << twd_ddK[twd_index] << ")" << std::endl;
                                
                                //
                                dot_twd_dK = std::inner_product(twd_dK.begin(), twd_dK.end(), twd_weights.begin(), 0.0) / twd_dK.size();
                                twd_gain = 1 / dot_twd_dK;
                                
                                //
                                adjust_prm = true;
                            }
                            else {
                                //
                                pid.K[twd_index] += twd_dK[twd_index];
                                std::cout << "     -> adjusting pid.K[" << twd_index << "] = " << pid.K[twd_index] << " with " << "twd_dK[" << twd_index << "] = " << twd_dK[twd_index] << std::endl;
                                
                                //
                                twd_dK_prv = twd_dK[twd_index];
                                twd_dK[twd_index] *= (2 - twd_ddK[twd_index]);
                                std::cout << "     -> param down: twd_dK[" << twd_index << "] = " << twd_dK[twd_index] << " = " << twd_dK_prv << " * (" << 2 - twd_ddK[twd_index] << ")" << std::endl;
                                
                                //
                                dot_twd_dK = std::inner_product(twd_dK.begin(), twd_dK.end(), twd_weights.begin(), 0.0) / twd_dK.size();
                                twd_gain = 1 / dot_twd_dK;
                            }
                            
                            // 
                            //cur_error = 0.0;
                            twd_check = 0;
                            twd_check_d = 1;
                        }
                        
                        //
                        cur_error = 0.0;
                        
                    }
                    
                    // Driving veeery slowly for twiddle, otherwise regular speed
                    if (twd_gain < twd_tolerance) {
                        if (twd_check <= 0) {
                            throt_value = 0.075;
                        }
                        std::cout << twd_gain << " vs. " << twd_gain_prv << std::endl;
                        if (twd_gain > twd_gain_prv) {
                            //
                            throt_value *= 1.1;
                            std::cout << "increasing speed at " << throt_value << std::endl;
                        }
                        else {
                            //
                            throt_value *= 0.9;
                            std::cout << "decreasing speed at " << throt_value << std::endl;
                        }
                        if (throt_value < 0.075) {
                            throt_value = 0.075;
                        }
                        if (throt_value > 0.3) {
                            throt_value = 0.3;
                        }
                        
                        //
                        twd_gain_prv = twd_gain;
                    }
                    else {
                        throt_value = 0.3;
                        std::cout << "### final parameters found: " << std::endl;
                        for (unsigned int i = 0; i < pid.K.size(); i++) {
                            std::cout << "  -> pid.K[" << i << "] = " << pid.K[i] << std::endl;
                        }
                    }
                    
                    ////
                    //twd_gain_prv = twd_gain;
                    
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
