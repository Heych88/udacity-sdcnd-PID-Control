## Project: PID Controller

---


**Steps to complete the project:**  


The goals / steps of this project are the following:

* Fill in `PID.cpp` with your PID controller code.
* Create and initalise the PID controller for steering angle and vehicle speed in the `main.cpp`.
* Tune the hyperparameters (P, I, D coefficients). This can be done through manual tuning, twiddle, SGD, or something else, or a combination!
* Perform PID system Analysis for the controller following the [project rubric](https://review.udacity.com/#!/rubrics/824/view).

[//]: # (Image References)

[image1]: ./img/PID_en.svg
[image2]: ./img/PID_Compensation_Animated.gif


## [Rubric](https://review.udacity.com/#!/rubrics/824/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---

### Project Implementation
#### 1. Implement the PID algorithm in the `PID.cpp` file and implement the control systems by completing the `main.cpp` file and tune the hyperparameters to successfully drive a lap around the track.

Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.

The PID algorithms implementation can be found in the `PID.cpp` file located in `/src`.

The PID class is initialised by calling the `PID::Init()` function with the required hyper parameter gains and if twiddle is used to optimise the parameters. The code is located between lines 20 and 49.

The PID update algorithm is run by calling the `PID::UpdateError()` function with the observed system error. The code is located between lines 53 and 77.

To find the hyperparameters, a hill climbing gradient descent method was used called twiddle. The twiddle code can be found in the file `main.cpp` between lines 61 and 103. It adjusts one parameter at a time based on the objective of increasing the system run time.

### Reflection
#### 1. Describe the effect each of the P, I, D components had in your implementation.

Below shows the PID algorithm used in this project. [[1](By Arturo Urquizo - http://commons.wikimedia.org/wiki/File:PID.svg, CC BY-SA 3.0, https://commons.wikimedia.org/w/index.php?curid=17633925)]

![A block diagram of a PID controller in a feedback loop. r(t) is the desired process value or "set point", and y(t) is the measured process value.][image1]

A PID controller involves three steps.
1. Calculate the plant error *e(t)* by negating the plants measured output *y(t)* from the the system desired setpoint *r(t)*.
`error(t) = r(t) - y(t)`

2. Update the system gain to minimise the plant error by calculating the three system modules, Proportional, Integral and Differential parameters.
 * **Proportional (P)** acts on the present values error and adjust its final contribution based on the *Kp* hyperparameter.
 ` P = Kp * error(t) `

 * **Integral (I)** acts on the summation of all the past and present values error and adjust its final contribution based on the *Ki* parameter. This attempts to minimise drift and model offsets. For example, if the current output has a slight offset, the integral of the error will accumulate over time, and the controller will respond by applying a stronger action towards the setpoint.
 ` I = Ki * sum(error(t) + error(t-1) + error(t-2) + ...)`

 * **Differential (D)** accounts for possible future trends of the error, based on its current rate of change and adjust its final contribution based on the *Kd* parameter. For example, when the controller gain is large, the control output will reduce the rate of change in the gain, effectively dampening the system. The derivative is negative when the error is positive, reducing the size of the *I* and *P* gains and the greater the difference between the current and previous errors, the more the D module reduces the strength of the action to prevent overshooting.
 ` D = Kd * (error(t) - error(t-1)) / delta_t`

3. Sum each modules gain and pass it to the plant's input.
`output = P + I + D`

The below animation shows the effect of each term, outlined in step 2 above, on the system. [[2](By Physicsch - Own work, CC0, https://commons.wikimedia.org/w/index.php?curid=40528698)]

![Effects of varying PID parameters (Kp,Ki,Kd) on the step response of a system.][image2]

As can be seen above, increasing the *Kp* parameter shifts the output towards the desired setpoint but will cause the system to oscillate around or near the setpoint due to the module only looking at the current error.

Adjusting the *Ki* value pushes the system to average the error at the setpoint.

Increasing *Kd* dampens the system gain reducing oscillation and setpoint overshot. It is important to note that as *Kd* increases the system will become overdamped and take a very long time to approach the setpoint. Another important consideration is that high-frequency oscillations can be amplified with the derivative module and should only be used when the system updates are faster than the model oscillations frequencies.

#### 2. Describe how the final hyperparameters were chosen.

The hyperparameters were found by using both the manual method and the gradient descent (twiddle) method. The manual method was used to tune the speed PID controller and the twiddle method was used to tune the initial steer controller followed by a slight manual adjustment. The twiddle code can be found in the file `main.cpp` between lines 61 and 103, is a hill climbing method that adjusts one parameter at a time with the objective of increasing the system run time.

The manual parameter tuning works as follows:
1. Set all parameters to 0
2. Increase only Kp and observe the speed
3. Increase the value until the cars speed oscillates around the target speed decrease if excessive oscillation
4. Repeat step 3 until the speed is slightly oscillating about the  target speed
5. Repeat steps 2 to 4 now with the derivative Kd parameter until the osscilations subside/stop. The result may not be exactly the target speed, but the goal is to stop the oscillations.
6. Repeat steps 2 to 4 now with the integral Ki parameter until the speed is the same as the target speed.

The twiddle method works as follows:
1. Init the parameters with their starting values, often all zero
2. Run the system and observe the error
3. A. If the first run, save the error as the best error best error so far, increase the current adjustable parameter by 1.1 times its current value.

 B. not first, compare the error with the best error and update. if error is worse, decrease the current parameter by the 2 * delta size variable and also decrease the value of the delta variable.
4. Check if the error or the delta value is below the accepted threshold level or runtime has not been exceeded.
5. Repeat steps 2 to 4 until step 4 is no longer valid
6. Change to the next parameter to be tuned, with the previously tuned parameters and repeat steps 2 to 5 until all parameters have been tuned.

Due to how the twiddle error is calculated, only system runtime, the parameters are not finely optimised for the system. The method as of writing this only locates usable hyperparameters. Stability can be improved with small manual adjustments.

Further improvements will require the tracking of speed and sensing is the system stops moving, thus increasing the runtime but crashing in the process.

Another improvement will be to include the total run error as a second parameter check, thus determining if the same runtime was more or less accurate with the new parameters.

The final hyperparameters used are:

PID | Kp | Ki | Kd
---|---|---|---
Steer | -0.091 | -0.001 | -1.52121
Speed | 0.5 | 0.00024 | 1.1

### Resources
[PID Controller wiki](https://en.wikipedia.org/wiki/PID_controller).

[1] [PID block diagram By Arturo Urquizo](By Arturo Urquizo - http://commons.wikimedia.org/wiki/File:PID.svg, CC BY-SA 3.0, https://commons.wikimedia.org/w/index.php?curid=17633925)

[2] [Effects of varying parameters GIF By Physicsch](By Physicsch - Own work, CC0, https://commons.wikimedia.org/w/index.php?curid=40528698)
