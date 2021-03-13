# CarND-PID-Control
Self-Driving Car Engineer Nanodegree Program
Ryan O'Shea

---

## Dependencies

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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)


## Results 
The code can be seen in action on the virtual track in the video below:

https://www.youtube.com/watch?v=RQlgXcRmRM8

It has a bit of wobble and some of its turns are close but I believe these could be fixed by further tuning of the PID parameters. It could also likely be improved by using the steering angle to determine when the car is in a tight turn and then lowering the target speed. This would allow the car to make the turns in a safer manner and probably prevent some of the situations where the car gets close to the side of the rode when turning sharply. 

## Code Explanation
The PID controllers for the steering and throttle are both setup outside of the onMessage function so their class variables can be used to save data between messages. With each message received from the simulator the error for each of the PID components is updated and then used to calculate the ideal steering angle and throttle value. In order to update the error values, the PID functions were implemented in the following way:

``` c++
p_error = cte;
d_error = cte - prev_cte;
i_error += cte;

// Update the previous cte so it can be used next time
prev_cte = cte;
```

This is almost a direct translation from the python PID control code that I wrote during the lessons. After the error values have been updated they are used to calulate the desired output using the following code:

``` c++
double total_error = -Kp*p_error - Kd*d_error - Ki*i_error;
```

This is a direct implementation of the PID error calculation function presented to us in class. These functions are used for both the steering angle and throttle calculation but with different PID objects. For the steering angle PID controller the cte reported by the simulator is used. For the throttle PID controller, the difference between the desired speed and the current speed reported by the simulator is used. I used 40 mph as the car's ideal speed as it represented a good compromise in getting the track done quickly and safely. When tested at higher speeds the car's turns and occasional wobble produced unsafe conditions for the car. Higher speeds could likely be sustained with different PID steering control parameters. 

## Parameter Tuning
Due to the nature of having to run the PID controller on a simulator instead of text file dataset like in the lessons, it seemed like it would be incredibly time consuming to implement Twiddle tuning of the parameters. Because of this I used the logic behind Twiddle to manually tune the parameters by hand. This was fairly dificult as sometimes the car would see improvement in certain areas of driving while suffering in others after a parameter tweak. The parameters for the steering angle controller were tuned before the throttle controller existed so it was tuned independently of it. Once that was working properly I added the throttle controller and used the steering angle controllers parameters as the starting point. These parameters actually worked very well so they have been kept in the final implementation. As mentioned above the parameters can certainly use more tuning. If Twiddle could be integrated into the simulator it would certainly be ideal for this.
