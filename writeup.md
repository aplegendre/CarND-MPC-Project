# **Model Predictive Control**
## Writeup for Term 2, Project 5


---

**Model Predictive Control Project**

The goal of this project is to implement Model Predictive Control to drive the car in the simulator around track 1 without going off the road. Waypoints are provided by the simulator and the throttle and steering inputs must be calculated using a standard kinematic model that accounts for 100 milliseconds of latency.


## [Rubric](https://review.udacity.com/#!/rubrics/896/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Compilation

#### 1. Your code should compile.

My code should be able to be compiled with `cmake` and `make` without issue. I did not make any changes to CMakeLists.txt.

### Implementation

#### 1. The Model

I implemented the model predicitive control (MPC) algorithm that was presented in the lessons and in the MPC quiz. I had some difficulty with getting the controller to make predictions due to errors in my original MPC equations, so I heavily referenced the walkthrough Q&A video for this project as I debugged the code. Many of the core elements including the pre-processing transformation and the tuned model parameters were taken directly from the recommendations of that video.

The model I used includes 6 elements in the state vector: x position, y position, heading (`psi`), velocity (`v`), cross-track error (`cte`), and steering error (`epsi`). It also includes 2 actuators: throttle (`a`) and steering angle (`delta`).

The update equations follow the standard kinematic model that was presented in the lesson:\
  `x_{t+1} = x_t + v_t * cos(psi) * dt`\
  `y_{t+1} = y_t + v_t * sin(psi) * dt`\
  `psi_{t+1} = psi_t - v_t / L_f * dt * delta`\
  `v_{t+1} = v_t + a_t * dt`\
  `cte_{t+1} = f(x_t) - y_t + v_t * sin(epsi_t) * dt`\
  `epsi_{t+1} = psi_t - f'(x_t) - v_t / L_f * dt * delta_t`\
where `L_f = 2.67` as provided in the lesson, `f(x)` is the best fit polynomial to the waypoints, and `dt` is the prediction timestep for the model predictive controller. Note that the steering inputs (`delta`) are inverted in these equations due to the setup of the simulator. This was a point of frustration for me since I had orginally inverted the input in only one of the two equations that have it. This resulted in an inability for the MPC to calculate a solution at all and ultimately led me to the Q&A video for debug.

The controller takes the fitted polynomial and adjusts each of the values within the above defined constraint equations and within the actuator limits to find the inputs that will optimize the following cost functions, which are all summed together for each prediction time step.

Cost equations based on the desired reference state are used to target zero cross-track error, zero steering error, and a desired reference speed (in this case, 70):\
  `Cost = 2000 * cte^2 + 2000 * epsi^2 + (v - v_ref)^2`\
The weights (2000, 2000, 1) were taken from the Q&A video and not further tuned due to good performance.

Cost equations based on both absolute and differential actuator minimization were used to ensure a smooth and continuous path along the track:\
  `Cost = 5 * delta^2 + 200 * (delta_{t+1} - delta_t)^2 + 5 * a^2 + 10 * (a_{t+1} - a_t)^2`\
Again, the weights were taken from the Q&A video without further tuning.

#### 2. Timestep Length and Elapsed Duration (N & dt)
The number of timesteps chosen was `N = 10` and the MPC timestep duration was `dt = 100ms` as suggested by the Q&A video. I had originally used `N=25` and `dt = 50ms` as shown in the MPC quiz, but changed them while debugging other parts of the code. I did not see any performance issues with the recommended steps, so I kept them. If the calculated path had not been smooth, then I might have decreased `dt` and increased `N`. If the controller calculation was introducing significant extra latency, then I would have increased `dt` and decreased `N`.

#### 3. Polynomial Fitting and MPC Preprocessing
As recommended by the Q&A video, I fit a third order polynomial to the waypoints. Before fitting the curve and passing the state to the MPC, I transformed the waypoint coordinates to the car's coordinate system and set x, y, and $\psi$ all to zero to represent the state in the same frame of reference. This simplified the implementation of the MPC, in particular for calculating cte and epsi, and avoided later transformations of the points anyway for the visualizations. 

I had orginally attempted to operate within the global coordinate system instead and to have a generalized implementation of the curve fitting for any order polynomial, but moved away from that implementation during debugging.

#### 4. Model Predictive Control with Latency
I handled the 100ms latency by using the kinematic model and the last actuator inputs to predict the state at the time the actuators would actually be applied. I passed this prediction to the MPC algorithm as the initial state instead of the actual state. This meant that the proper actuator inputs would be calculated for the future state of the vehicle rather than one that would be in the past after accounting for latency.


### Simulation

#### 1. The vehicle must successfully drive a lap around the track.

The vehicle successfully drove around the track on my laptop while 100ms of latency was applied.