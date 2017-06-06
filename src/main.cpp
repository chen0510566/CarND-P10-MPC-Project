#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi()
{
    return M_PI;
}

double deg2rad(double x)
{
    return x * pi() / 180;
}

double rad2deg(double x)
{
    return x * 180 / pi();
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.rfind("}]");
    if (found_null != string::npos)
    {
        return "";
    }
    else
    {
        if (b1 != string::npos && b2 != string::npos)
        {
            return s.substr(b1, b2 - b1 + 2);
        }
    }
    return "";
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order)
{
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++)
    {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++)
    {
        for (int i = 0; i < order; i++)
        {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x)
{
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++)
    {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}

/**@brief transform points from map frame into vehicle frame;
 *
 * transform the points from map frame into vehicle frame;
 * @param v_x [IN]: the x cooridinate of the vehicle frame origin in map frame;
 * @param v_y [IN]: the y cooridinate of the vehicle frame origin in map frame;
 * @param v_psi [IN]: the yaw angle of the vehicle;
 * @param px [IN]: the x coordinates of the points to be transformed, in map frame;
 * @param py [IN]: the y coordinates of the points to be transformed, in map frame;
 * @param tx [OUT]: the transformed x coordinates in vehicle frame;
 * @param ty [OUT]: the transformed y coordinates in vehicle frame;
 */
void TransformPoints(const double v_x, const double v_y, const double v_psi, const std::vector<double> px, const std::vector<double> py, std::vector<double> &tx, std::vector<double> &ty)
{
    tx.reserve(px.size());
    ty.reserve(px.size());

    double cos_psi = cos(-v_psi);
    double sin_psi = sin(-v_psi);
    double delta_x = 0.0;
    double delta_y = 0.0;
    for (int i = 0; i < px.size(); ++i)
    {
        delta_x = px[i] - v_x;
        delta_y = py[i] - v_y;

        tx.push_back(cos_psi * delta_x - sin_psi * delta_y);
        ty.push_back(sin_psi * delta_x + cos_psi * delta_y);
    }

    double dist = 0.0;
    for (int i = 1; i < px.size(); ++i)
    {
        dist += sqrt((tx[i]-tx[i-1])*(tx[i]-tx[i-1])+(ty[i]-ty[i-1])*(ty[i]-ty[i-1]));
    }
    std::cerr<<"ref path dist:"<<dist<<std::endl;

}

void PredictState(const Eigen::VectorXd &state, const Eigen::VectorXd coef, Eigen::VectorXd& pred_state)
{
    double x = state[0];
    double y = state[1];
    double psi = state[2];
    double v = state[3];
    double cte = state[4];
    double epsi = state[5];
    double last_steer = -state[6]*0.436332;
    double last_throttle = state[7]*40;

    double dt = 0.1;

    double pred_x = x + v*cos(psi)*dt;
    double pred_y = y + v*sin(psi)*dt;
    double pred_psi = psi + v*last_steer*dt/2.67;
    double pred_v = v+last_throttle*dt;
    double f0 = polyeval(coef, x);
    double psides0 = atan(coef[1]+2*coef[2]*x+3*coef[3]*x*x);
    double pred_cte = (f0-y)+(v*sin(epsi)*dt);
    double pred_epsi = (psi - psides0) + v*last_steer*dt/2.67;

    pred_state[0] = pred_x;
    pred_state[1] = pred_y;
    pred_state[2] = pred_psi;
    pred_state[3] = pred_v;
    pred_state[4] = pred_cte;
    pred_state[5] = pred_epsi;
    pred_state[6] = state[6];
    pred_state[7] = state[7];

}

int main()
{

    uWS::Hub h;

    // MPC is initialized here!
    MPC mpc;

    h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
                {
                    static double last_steer = 0.0;
                    static double last_speed = 0.0;
                    // "42" at the start of the message means there's a websocket message event.
                    // The 4 signifies a websocket message
                    // The 2 signifies a websocket event
                    string sdata = string(data).substr(0, length);
                    cout << sdata << endl;
                    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2')
                    {
                        string s = hasData(sdata);
                        if (s != "")
                        {
                            auto j = json::parse(s);
                            string event = j[0].get<string>();
                            if (event == "telemetry")
                            {
                                // j[1] is the data JSON object
                                vector<double> ptsx = j[1]["ptsx"];
                                vector<double> ptsy = j[1]["ptsy"];
                                double px = j[1]["x"];
                                double py = j[1]["y"];
                                double psi = j[1]["psi"];
                                double v = j[1]["speed"];
                                v *= 0.447;//converted to m/s;

                                //transformed the way points into vehicle frame;
                                vector<double> transformed_ptsx;
                                vector<double> transofrmed_ptsy;
                                TransformPoints(px, py, psi, ptsx, ptsy, transformed_ptsx, transofrmed_ptsy);

                                //transform vehicle state into vehicle frame;
                                px = 0.0;
                                py = 0.0;
                                psi = 0.0;

                                //fit the way points by 3rd polynomials;
                                Eigen::VectorXd coef = polyfit(Eigen::VectorXd::Map(transformed_ptsx.data(), transformed_ptsx.size()), Eigen::VectorXd::Map(transofrmed_ptsy.data(), transformed_ptsx.size()), 3);


                                //calculate cte and epsi;
                                double cte = polyeval(coef, 0.0);
                                double epsi = atan(coef[1]);
                                Eigen::VectorXd state(8);
                                state << px, py, psi, v, cte, epsi, last_steer, last_speed;
                                std::cout << "state:" << px << "\t" << py << "\t" << psi << "\t" << v << "\t" << cte << "\t" << epsi << std::endl;

                                Eigen::VectorXd pred_state(8);
                                PredictState(state, coef, pred_state);

                                double steer_value = 0.0;
                                double throttle_value = 0.0;

                                //Display the MPC predicted trajectory
                                vector<double> mpc_x_vals;
                                vector<double> mpc_y_vals;

                                std::vector<double> control = mpc.Solve(pred_state, coef, mpc_x_vals, mpc_y_vals);

                                steer_value = -control[0]/deg2rad(25);
                                throttle_value = control[1]/40.0;
                                std::cerr << "control:" << steer_value << "\t" << throttle_value << std::endl;

                                json msgJson;
                                msgJson["steering_angle"] = steer_value;
                                msgJson["throttle"] = throttle_value;

                                last_steer = steer_value;
                                last_speed = throttle_value;


                                //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                                // the points in the simulator are connected by a Green line

                                msgJson["mpc_x"] = mpc_x_vals;
                                msgJson["mpc_y"] = mpc_y_vals;

                                //Display the waypoints/reference line

                                //add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                                // the points in the simulator are connected by a Yellow line

                                vector<double> next_x_vals;
                                vector<double> next_y_vals;
//                                next_x_vals = transformed_ptsx;
//                                next_y_vals = transofrmed_ptsy;
                                next_x_vals.reserve(transformed_ptsx.size());
                                next_y_vals.reserve(transofrmed_ptsy.size());
                                for (int i = 0; i < transformed_ptsx.size(); ++i)
                                {
                                    next_x_vals.push_back(transformed_ptsx[i]);
                                    next_y_vals.push_back(polyeval(coef, transformed_ptsx[i]));
                                }

                                msgJson["next_x"] = next_x_vals;
                                msgJson["next_y"] = next_y_vals;


                                auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                                std::cout << msg << std::endl;
                                // Latency
                                // The purpose is to mimic real driving conditions where
                                // the car does actuate the commands instantly.
                                //
                                // Feel free to play around with this value but should be to drive
                                // around the track with 100ms latency.
                                //
                                // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
                                // SUBMITTING.
                                this_thread::sleep_for(chrono::milliseconds(100));
                                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                            }
                        }
                        else
                        {
                            // Manual driving
                            std::string msg = "42[\"manual\",{}]";
                            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                        }
                    }
                });

    // We don't need this since we're not using HTTP but if it's removed the
    // program
    // doesn't compile :-(
    //    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t)
    //                    {
    //                        const std::string s = "<h1>Hello world!</h1>";
    //                        if (req.getUrl().valueLength == 1)
    //                        {
    //                            res->end(s.data(), s.length());
    //                        }
    //                        else
    //                        {
    //                            // i guess this should be done more gracefully?
    //                            res->end(nullptr, 0);
    //                        }
    //                    });

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
                   {
                       std::cout << "Connected!!!" << std::endl;
                   });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length)
                      {
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
