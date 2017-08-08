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
    bool twiddle = true;
    int twd_run = 0;
    const int twd_run_itv = 3;
    int twd_check = -1;
    const double dfactor = 0.1;
    double cur_error = 0.0;
    double best_error = 0.0;
    //bool twd_run_0 = true;
    //int twd_run_0 = 1;
    //bool twd_run_1 = true;
    //int twd_run_1 = 1;
    //bool twd_run_2 = true;
    //int twd_run_2 = 1;
    //bool twd_run_3 = true;
    //int twd_run_3 = 1;
    
    // Initializing parameters
    //pid.Init(0.35, 0.01, 0.004);
    //pid.Init({0.2, 0.004, 3.0});
    //pid.Init({0.0, 0.0, 0.0});
    //pid.Init(0.134611, 0.000270736, 3.05349, 0.1, 0.1, 0.1);
    pid.Init(0.0, 0.000270736, 3.05349, 0.1, 0.1, 0.1);
    
    //h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    //h.onMessage([&pid, &time_cur, &time_prv, &time_tot, &twiddle, &time_itv, &dfactor, &cur_error, &best_error](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    //h.onMessage([&pid, &time_cur, &time_prv, &time_tot, &twiddle, &twd_run, &twd_run_itv, &dfactor, &cur_error, &best_error, &twd_run_0, &twd_run_1, &twd_run_2, &twd_run_3](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    h.onMessage([&pid, &time_cur, &time_prv, &time_tot, &twiddle, &twd_run, &twd_run_itv, &dfactor, &cur_error, &best_error, &twd_check](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
                    
                    // 
                    twd_run += 1;
                    
                    // 
                    time_cur = clock();
                    double delta_t = time_cur - time_prv;
                    //std::cout << "delta_t = " << delta_t << std::endl;
                    time_tot += time_cur;
                    time_prv = time_cur;
                    
                    //
                    pid.UpdateError(cte, delta_t);
                    steer_value = -pid.TotalError();
                    //std::cout << "steer_value = " << steer_value << std::endl;
                    
                    // Keeping steer_value in [-1,1]
                    if (steer_value > 1) steer_value = 1;
                    if (steer_value < -1) steer_value = -1;
                    
                    // Accumulating squared error
                    cur_error += pow(cte, 2);
                    
                    // Twiddle
                    //std::cout << "twd_run = " << twd_run << " : twd_run % twd_run_itv = " << twd_run % twd_run_itv << std::endl;
                    if (twd_run % twd_run_itv == 0) {
                        
                        // 
                        twd_check += 1;
                        
                        std::cout << "*** twiddle check" << std::endl;
                        //std::cout << "twd_run_0 = " << twd_run_0 << " : twd_run_1 = " << twd_run_1 << " : twd_run_2 = " << twd_run_2 << " : twd_run_3 = " << twd_run_3 << std::endl;
                        std::cout << "****** twd_check = " << twd_check << std::endl;
                        
                        // 
                        //if (twd_run_0 && !twd_run_1 && !twd_run_2 && !twd_run_3) {
                        //if (twd_run_0 == 1 and twd_run_1 == 0 and twd_run_2 == 1 and twd_run_3 == 1) {
                        if (twd_check == 0) {
                            std::cout << "@" << twd_run << " : run 0 : " << "cur_error = " << cur_error << std::endl;
                            std::cout << " : best_error = " << best_error;
                            best_error = cur_error;
                            
                            // 
                            std::cout << " : best_error = " << best_error << " : " << "pid.Kp = " << pid.Kp << " : " << "pid.dKp = " << pid.dKp << std::endl;
                            
                            // Let run...
                            cur_error = 0.0;
                            //twd_run_0 = false;
                            //twd_run_0 = true;
                        }
                        
                        //
                        //if (!twd_run_0 && twd_run_1 && twd_run_2 && twd_run_3) {
                        if (twd_check == 1) {
                            std::cout << "@" << twd_run << " : run 1 : " << "cur_error = " << cur_error << std::endl;
                            std::cout << " : best_error = " << best_error;
                            pid.Kp += pid.dKp;
                            
                            // 
                            std::cout << " : best_error = " << best_error << " : " << "pid.Kp = " << pid.Kp << " : " << "pid.dKp = " << pid.dKp << std::endl;
                            
                            // Let run...
                            cur_error = 0.0;
                            //twd_run_1 = false;
                            //twd_run_1 = 0;
                        }
                        
                        //
                        //if (!twd_run_0 && !twd_run_1 && twd_run_2 && twd_run_3) {
                        //if (twd_run_0 == 0 && twd_run_1 == 0 && twd_run_2 == 1 && twd_run_3 == 1) {
                        if (twd_check == 2) {
                            std::cout << "@" << twd_run << " : run 2 : " << "cur_error = " << cur_error << std::endl;
                            std::cout << " : best_error = " << best_error;
                            if (cur_error < best_error) {
                                best_error = cur_error;
                                pid.dKp *= 1.1; //(1 + dfactor);
                                std::cout << " : best_error better" << " : " << "pid.Kp = " << pid.Kp << " : " << "pid.dKp = " << pid.dKp << std::endl;
                            }
                            else {
                                pid.Kp -= 2 * pid.dKp;
                                std::cout << " : best_error worse" << " : " << "pid.Kp = " << pid.Kp << " : " << "pid.dKp = " << pid.dKp << std::endl;
                                //cur_error = 0.0;
                            }
                            
                            // Let run...
                            cur_error = 0.0;
                            //twd_run_2 = false;
                            //twd_run_2 = 0;
                        }
                        
                        //
                        //if (!twd_run_0 && !twd_run_1 && !twd_run_2 && twd_run_3) {
                        //if (twd_run_0 == 0 and twd_run_1 == 0 and twd_run_2 == 0 and twd_run_3 == 1) {
                        if (twd_check == 3) {
                            std::cout << "@" << twd_run << " : run 3 : " << "cur_error = " << cur_error << std::endl;
                            std::cout << " : best_error = " << best_error;
                            if (cur_error < best_error) {
                                best_error = cur_error;
                                pid.dKp *= 1.1;
                                std::cout << " : best_error better" << " : " << "pid.Kp = " << pid.Kp << " : " << "pid.dKp = " << pid.dKp << std::endl;
                            }
                            else {
                                pid.Kp += pid.dKp;
                                pid.dKp *= 0.9;
                                std::cout << " : best_error worse" << " : " << "pid.Kp = " << pid.Kp << " : " << "pid.dKp = " << pid.dKp << std::endl;
                            }
                            
                            // 
                            cur_error = 0.0;
                            //twd_run_1 = true;
                            //twd_run_1 = 1;
                            //twd_run_2 = true;
                            //twd_run_2 = 1;
                            //twd_run_3 = true;
                            //twd_run_3 = 1;
                            //twd_check = -1;
                            twd_check = 0;
                        }
                        
                    }
                    
                    // 
                    double throt_value;
                    throt_value = 0.3;
                    if (fabs(cte) > 0.5) {
                        //throt_value *= 0.9;
                    }
                    
                    // DEBUG
                    //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
                    
                    json msgJson;
                    msgJson["steering_angle"] = steer_value;
                    msgJson["throttle"] = throt_value;
                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    std::cout << msg << std::endl;
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
