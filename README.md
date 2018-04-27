# Extended Kalman Filter Project (Starter Code)

Self-Driving Car Engineer Nanodegree Program

In this project you will utilize a Kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower than the tolerance outlined in the project rubric. 

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   - On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `

## Accuracy

Based on the rubric points for accuracy, `RMSE [px, py, vx, vy]` must be less than `[.11, .11, 0.52, 0.52]`.  

> From the rubric:
>
> “Your algorithm will be run against Dataset 1 in the simulator which is the same as "[data/obj_pose-laser-radar-synthetic-input.txt](https://github.com/mhBahrami/Extended-Kalman-Filter/blob/master/data/obj_pose-laser-radar-synthetic-input.txt)" in the repository. We'll collect the positions that your algorithm outputs and compare them to ground truth data. Your px, py, vx, and vy RMSE should be less than or equal to the values [.11, .11, 0.52, 0.52].”

I put the results of accuracy in "[results/output.txt](https://github.com/mhBahrami/Extended-Kalman-Filter/blob/master/results/output.txt)." You can see that after a while the accuracy has been improved and met the requirement ([Line 44](https://github.com/mhBahrami/Extended-Kalman-Filter/blob/master/results/output.txt#L44)).

## License

[MIT License](https://github.com/mhBahrami/Extended-Kalman-Filter/blob/master/LICENSE).