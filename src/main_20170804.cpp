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
    const int twd_run_itv = 20;
    int twd_check = -1;
    int twd_check_d = 1;
    double cur_error = 0.0;
    double best_error = 0.0;
    
    // Initializing parameters
    //pid.Init({0.134611, 0.000270736, 3.05349}, {0.01, 0.0001, 0.001});
    
    //pid.Init({0.1, 0.000270736, 3.05349}, {0.01, 0.0001, 0.001});
    pid.Init({0.1, 0.000270736, 3.05349});
    std::vector<int> twd_K{1, 0, 0};
    std::vector<double> twd_dK{0.01, 0.01, 0.01};
    const std::vector<double> twd_ddK{1.3, 1.3, 1.3};
    
    //h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    h.onMessage([&pid, &time_cur, &time_prv, &time_tot, &twd_K, &twd_dK, &twd_ddK, &twd_run, &twd_run_itv, &cur_error, &best_error, &twd_check, &twd_check_d](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
                    double speed = std::stod(j[1]["speed"].get<std::string>());
                    double angle = std::stod(j[1]["steering_angle"].get<std::string>());
                    double steer_value;
                    unsigned int twd_index;
                    
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
                    //std::cout << "twd_run = " << twd_run << " : twd_run % twd_run_itv = " << twd_run % twd_run_itv << std::endl;
                    if ((std::accumulate(twd_K.begin(), twd_K.end(), 0) > 0) && (twd_run % twd_run_itv == 0)) {
                        
                        // 
                        twd_check += twd_check_d;
                        std::cout << "****** twd_check = " << twd_check << std::endl;
                        
                        // 
                        if (twd_check == 0) {
                            
                            //
                            for (unsigned int i = 0; i < pid.K.size(); i++) {
                                if (twd_K[i] == 1) {
                                    twd_index = i;
                                }
                                else {
                                    if (twd_K[i] == 1) {
                                        twd_index = i;
                                    }
                                    else {
                                        if (twd_K[i] == 1) {
                                            twd_index = i;
                                        }
                                    }
                                }
                            }
                            
                            std::cout << "@" << twd_run << " : run 0" << std::endl;
                            // Initializing best error
                            best_error = cur_error;
                            std::cout << "initializing best_error = " << best_error << std::endl;
                            
                            // Adjusting parameter
                            for (unsigned int i = 0; i < pid.K.size(); i++) {
                                pid.K[i] += twd_K[i] * twd_dK[i];
                            }
                            std::cout << "adjusting pid.K[" << twd_index << "] = " << pid.K[twd_index] << " with " << "twd_dK[" << twd_index << "] = " << twd_dK[twd_index] << std::endl;
                            
                            // 
                            twd_check_d = 2;
                            
                            // Let run...
                            //cur_error = 0.0;
                        }
                        
                        //
                        if (twd_check == 1) {
                            std::cout << "@" << twd_run << " : run 1" << std::endl;
                            
                            // Adjusting parameter
                            for (unsigned int i = 0; i < pid.K.size(); i++) {
                                pid.K[i] += twd_K[i] * twd_dK[i];
                            }
                            std::cout << " -> adjusting pid.K[" << twd_index << "] = " << pid.K[twd_index] << " with " << "twd_dK[" << twd_index << "] = " << twd_dK[twd_index] << std::endl;
                            
                            //
                            twd_check_d = 1;
                            
                            // Let run...
                            //cur_error = 0.0;
                        }
                        
                        //
                        if (twd_check == 2) {
                            std::cout << "@" << twd_run << " : run 2" << std::endl;
                            std::cout << "cur_error = " << cur_error << std::endl;
                            std::cout << "best_error = " << best_error << std::endl;
                            if (cur_error < best_error) {
                                //
                                best_error = cur_error;
                                
                                //
                                for (unsigned int i = 0; i < pid.K.size(); i++) {
                                    //twd_dK[i] *= twd_K[i] * 1.1;
                                    twd_dK[i] *= twd_K[i] * twd_ddK[i];
                                }
                                std::cout << "   -> param up: twd_dK[" << twd_index << "] = " << twd_dK[twd_index] << std::endl;
                                
                                //
                                for (unsigned int i = 0; i < pid.K.size(); i++) {
                                    pid.K[i] += twd_K[i] * twd_dK[i];
                                }
                                std::cout << "   -> adjusting pid.K[" << twd_index << "] = " << pid.K[twd_index] << " with " << "twd_dK[" << twd_index << "] = " << twd_dK[twd_index] << std::endl;
                                
                                //
                                //twd_check -= 1;
                                twd_check_d = 0;
                            }
                            else {
                                //
                                for (unsigned int i = 0; i < pid.K.size(); i++) {
                                    pid.K[i] -= twd_K[i] * 2 * twd_dK[i];
                                }
                                std::cout << "   -> reverting : pid.K[" << twd_index << "] = " << pid.K[twd_index] << " with " << "twd_dK[" << twd_index << "] = " << twd_dK[twd_index] << std::endl;
                                
                                //
                                twd_check_d = 1;
                            }
                            
                            // Let run...
                            //cur_error = 0.0;
                        }
                        
                        //
                        if (twd_check == 3) {
                            std::cout << "@" << twd_run << " : run 3" << std::endl;
                            std::cout << "cur_error = " << cur_error << std::endl;
                            std::cout << "best_error = " << best_error << std::endl;
                            
                            //
                            if (cur_error < best_error) {
                                //
                                best_error = cur_error;
                                
                                //
                                for (unsigned int i = 0; i < pid.K.size(); i++) {
                                    //twd_dK[i] *= twd_K[i] * 1.1;
                                    twd_dK[i] *= twd_K[i] * twd_ddK[i];
                                }
                                std::cout << "     -> param up: twd_dK[" << twd_index << "] = " << twd_dK[twd_index] << std::endl;
                            }
                            else {
                                //
                                for (unsigned int i = 0; i < pid.K.size(); i++) {
                                    pid.K[i] += twd_K[i] * twd_dK[i];
                                }
                                std::cout << "     -> adjusting pid.K[" << twd_index << "] = " << pid.K[twd_index] << " with " << "twd_dK[" << twd_index << "] = " << twd_dK[twd_index] << std::endl;
                                
                                //
                                for (unsigned int i = 0; i < pid.K.size(); i++) {
                                    //twd_dK[i] *= twd_K[i] * 0.9;
                                    twd_dK[i] *= twd_K[i] * (2 - twd_ddK[i]);
                                }
                                std::cout << "     -> param down: twd_dK[" << twd_index << "] = " << twd_dK[twd_index] << std::endl;
                            }
                            
                            // 
                            //cur_error = 0.0;
                            twd_check = 0;
                            twd_check_d = 1;
                        }
                        
                        //
                        cur_error = 0.0;
                    }
                    
                    // Driving very slowly for twiddle, otherwise regular speed
                    double throt_value;
                    if (std::accumulate(twd_K.begin(), twd_K.end(), 0) > 0) {
                        throt_value = 0.05;
                    }
                    else {
                        throt_value = 0.3;
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
