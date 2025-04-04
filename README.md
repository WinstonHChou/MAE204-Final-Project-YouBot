# MAE204-Final-Project-YouBot
WI 25 MAE 204 Final Project: YouBot

## Course Overview
This is the Final Project for [MAE 204: Robotics](https://tolley.eng.ucsd.edu/Teaching) at UCSD, taught by Professor [Michael T. Tolley](https://tolley.eng.ucsd.edu/).

## Project Description
The project involves controlling the motion of a mobile manipulator robot
called a **youBot** composed of a mobile base with four Mecanum wheels and a 5R arm.
This software will consist of 4 main components:
1. Kinematics Simulator
2. Reference Trajectory Generator
3. Feedforward plus Feedback Controller
4. Main Script

## Prerequisites
- Install **`MATLAB 2024b`**
- Install [**`CoppeliaSim EDU`**](https://www.coppeliarobotics.com/), and download [**`V-REP_scenes`**](https://hades.mech.northwestern.edu/images/5/5c/V-REP_scenes.zip)
- Clone the repository:
```bash
git clone --recurse-submodules https://github.com/WinstonHChou/MAE204-Final-Project-YouBot.git
```

## Running the Project
### Standard Run "`main.m`":
- __Task__: Select task type `(1 to 5)` at `line 18`
    ```cpp
    TASK DEFINITION:
    1 -> "best": P Controller + FeedForward
    2 -> "overshoot": PI Controller, No FeedForward
    3 -> "new_task": Customized Case
    4 -> "feedforward": Only FeedForward, and starts at exact initial state of the trajectory
    5 -> "speed_limit": Has maximum joint velocity of 5, compared to default of 40
    ```
    ```matlab
    task = tasks(1); % USER INPUT: 1="best", 2="overshoot", 3="new_task", 4="feedforward", 5="speed_limit"
    ```
- __Cube Poses__: Select cube poses at `line 20-23` (If chosen `"new_task"`)
    ```matlab
    % If choose "new_task", set custom initial and goal poses 
    % for the block/cube ([x, y, theta] in world frame {s})
    cube_initial = [1, 1, 0];
    cube_final = [1, -1, -pi/3];
    ```
- __Maximum Joint Velocity__: Select maximum joint velocity at `line 25, 26` (Not Applicable for `"speed_limit"`)
    ```matlab
    % Maximum Joint Velocity
    max_joint_vel = 40;
    ```

### Optional:
- __Feedback Gains Tuning__: Check out `line 28 - 99`, and look for the following at each case to **tune PI controller** and **enable/disable FeedForward**:
    ```matlab
    Kp = zeros(6,6);
    Ki = zeros(6,6);
    FF_enabled = true;
    ```

### Outputs:
- CSV files for CoppeliaSim EDU, **Scene 6: CSV Mobile Manipulation youBot**, are saved in `src/`, with naming of **"<CASE_NAME>_state_array"**. Please load the csv you want to test into CoppeliaSim.

## Results
### "Best" Case
- Error Twist Plot:
    ![img](png/best_twist_error.png)
- Manipulability Plot:
    ![img](png/best_manipulability.png)
### "Overshoot" Case
- Error Twist Plot:
    ![img](png/overshoot_twist_error.png)
- Manipulability Plot:
    ![img](png/overshoot_manipulability.png)
### "New Task" Case
- Error Twist Plot:
    ![img](png/new_task_twist_error.png)
- Manipulability Plot:
    ![img](png/new_task_manipulability.png)
### "Feedforward" Case
- Error Twist Plot:
    ![img](png/feedforward_twist_error.png)
- Manipulability Plot:
    ![img](png/feedforward_manipulability.png)
### "Speed Limit" Case
- Error Twist Plot:
    ![img](png/speed_limit_twist_error.png)
- Manipulability Plot:
    ![img](png/speed_limit_manipulability.png)


## Acknowledgments
This project is part of the **MAE 204** course at **UC San Diego**. For project instructions, please refer to [Final Project Guideline](pdf/MAE204_WI25_Final_Project.pdf).