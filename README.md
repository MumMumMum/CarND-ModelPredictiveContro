# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

Let start with Model Predictive Control.
Predictive control involves Prediction of controls for the next state based on current state.
Here we predict for a fixed duration of time frame(T = N*dt) and take the output solution(control) and Apply to the systemto generate New state.
The Prediction is mathematical solution provided by an OPtimizer(IPopt optimizer in our case).
This length of T is very crucial and is hyper parameter for the model.This T should be generally speaking gretter than setlling time of the system for model to predict well.As in every system has latency time to come to steady state after the controls  are being applied. So design of this T should be larger then setlling time but not large so that the computation time increase and you miss the prediction for new states and accuracy falls. Its a trade off and you will get a hang of it will tuning the system.

At every T time step we measure our output and assess new state(x,y,psi,v,CTE,EPSI) so we geat a measure of direction of our prediction and we apply this new state to our model to generate new controls . So our measuremnts act like feedback to system.
And keep T optimum ensure that system will not overshoot or get unstable.AS we predict for fixed time T,This is called receeding Horizon in model prediction control.

We now need  Mathematical  Model which approximates the system.The model should have states, Constraints,  and control.
The project use a simple bicycle model.
     > // x_[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
     > // y_[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
     > // psi_[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
     > // v_[t] = v[t-1] + a[t-1] * dt
     > // cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
     > // epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt

I started with MPC quiz, There the mpc tuning graph and the quiz soultion was of great help.
Lot of steps from quiz were taken into project.Only main.cpp was not there in quiz.

Later on implemeted the project as per quiz directive.
I was no where when the car recieved simulator commands.
Forgot to mention about IPOPT was a big step in project setup.

Later on added changes to position x,y as if the car is in X direction so predicted will be straight line along x axis.

Once the code was ready started moving the weights on cost function.Initally the car was looping back so checked on forumns.
Found that latency needs to be implemnetd.
I added but my solution behaves same even without latency code.
But changing this gives me a great start first time the loop back and oscillation have been removed.

>result.push_back(solution.x[delta_start+9]);
>result.push_back(solution.x[a_start+9]);
Here I just dont take the first value for steer and accelration, I take somewhere in between the array .

Then I go on changing weights looking at cte and delta.
I put some values in denominator and to ensure NAN does not occur I added a holy constant 1.appl
The denominator values are inverse in relation.

This is still far from complete.
But I need some inputs like,
Is that latency implemenattion good?
Is it necesscary? I dont see any chnages in My solution.
Will the output behave differently on different hardware as my simultaor is on windows and 
application on VM ubuntu.
So please lend me some inputs to go about this solutiuon.
## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
