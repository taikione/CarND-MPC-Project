# Model Predictive Control (Project5)
Here I will consider the rubric points individually and describe the model for this project.

# The Model
the model has state, actuators and update equations as follows.

#### State
- x: The global x position of vehicle
- y: The global y position of vehicle
- psi: The orientation of the vehicle in radians
- v: The current velocity in mph
- cte: The current cross track error
- epsi: The orientation error

#### Actuators
- delta: steering_angle
- a: throttle

#### Update equations
```
x1 = (x0 + v0 * CppAD::cos(psi0) * dt);
y1 = (y0 + v0 * CppAD::sin(psi0) * dt);
psi1 = (psi0 - v0 * delta0 / Lf * dt);
v1 = (v0 + a0 * dt);
cte1 = ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
epsi1 = (psi0 - psides0 - v0 * delta0 / Lf  * dt);
```
`delta0` and `a0` are actuators at time `t`. dt is duration value. These update equations are defined in `L94-99` of `MPC.cpp`

#### Model cost
The Cost of Models are defined as follows. Also these are defined in `L52-57` of `MPC.cpp`
```
State cost =  cte * cte + espi * espi + (v - reference velocity) * (v - reference velocity)
Actuators cost = delta * delta + 5 * a * a

Model = State cost + Actuators cost
```
The state cost represents the difference between state and reference state. The actuators cost represents how large the value of actuators.
In this model, I want to implement smooth driving, so I added state and actuators cost and set minimizing these cost value.
In this implementation, as the vehicle velocity close target value, the vehicle repeated deceleration and acceleration, so I multiplied throttle cost by 5 to prevent these operations.

# How to deside N and dt values
At first, I initialized the values of N and dt to 25 and 0.05. And I was gradually reduced values of N, and set N to 10.
After that, value of dt was increase, set dt to 0.1.

Finally, N and dt are defined as below. (`L9-10 MPC.cpp`)
```
N = 10
dt = 0.1
```

# MPC Preprocessing
All the input to `MPC::Solve()` was transformed into the vehicle orientation and coordinate. The vehicle orientation was set to be `(x, y, psi)` = `(0, 0, 0)`.

