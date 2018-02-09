# CarND-Controls-MPC
In this project I handled MPC controller to drive autonomous driving simulator around a given trak using by C++ in Udacity's Self-Driving Car Nanodegree Program.

You can find starter code created by the Udacity on [here](https://github.com/udacity/CarND-MPC-Project).

---

[image1]: ./assets/omw1.png "Result image 1"
[image2]: ./assets/omw2.png "Result image 2"
[image3]: ./assets/omw3.png "Result image 3"
[image4]: ./assets/omw4.png "Result image 4"

## Dependencies

* [Simulator](https://github.com/udacity/self-driving-car-sim/releases)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.

## How to Use

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
  * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run Simulator with Project 5: MPC Controller
5. Click Start button
6. Run it: `./mpc` on your terminal.

## The Model
The Model Predictive Control(MPC) uses the bicycle model whose front and back wheels work together, and also vehicles are able to apply same condition.

The states of the vehicle contents:

* **x**: x-coordinate
* **y**: y-coordinate
* **psi**: orientation angle
* **v**: velocity
* **cte**: crosstrack error
* **epsi**: orientation error

The actuation factors are:

* **delta**: steering angle
* **a**: throttle

You can see below, they are C++ codes for updating the vehicle state,

```
fg[x_start+i+1] = x1 - (x0 + v0*CppAD::cos(psi0)*dt);
fg[y_start+i+1] = y1 - (y0 + v0*CppAD::sin(psi0)*dt);
fg[psi_start+i+1] = psi1 - (psi0 - v0*(delta/Lf)*dt);
fg[v_start+i+1] = v1 - (v0 + a*dt);
fg[cte_start+i+1] = cte1 - ((f0 - y0) + v0*CppAD::sin(epsi0)*dt);
fg[epsi_start+i+1] = epsi1 - ((psi0 - psides0) - v0*(delta/Lf)*dt);
```

## Timestep Length and Elapsed Duration
I chose timestep length `N=10` and elapsed duration between timesteps `dt=0.1`. I want to trade length off against elapsed duration due to the fact that the vehicle do not drive out of the track.

## Polynomial Fitting and MPC Preprocessing
Before fitting a polynomial to the waypoints, I must calculate for moving the points to the origin and rotate the path to follow the vehicle orientation like below:

```
Eigen::VectorXd ptsx_(ptsx.size());
Eigen::VectorXd ptsy_(ptsy.size());

double dx = 0;
double dy = 0;

for (unsigned int i=0; i < ptsx.size(); i++) {
  dx = ptsx[i] - px;
  dy = ptsy[i] - py;
  ptsx_[i] = dx*cos(-psi) - dy*sin(-psi);
  ptsy_[i] = dx*sin(-psi) + dy*cos(-psi);
}
```

After that, the polynomial is fitted by `polyfit` like below,
```
auto coeffs = polyfit(ptsx_, ptsy_, 3);
```


## Model Predictive Control with Latency
The MPC handles a `0.1s` latency, which simulates latency between sensors and processing, before calculating by `mpc.solve`. Look at the C++ code lines below,

```
const double latency = 0.1;
const double Lf = 2.67;
double px_ = v*latency;
double py_ = 0;
double psi_ = -v*steer_value*latency/Lf;
double v_ = v + throttle_value*latency;
double cte_ = cte + v*sin(epsi)*latency;
double epsi_ = epsi + psi_;
```

## Results
| ![][image1] | ![][image2] |
|:-----------:|:-----------:|
| ![][image3] | ![][image4] |
