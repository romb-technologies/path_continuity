
## About

A Qt application for demostration of how path continuity affect velocity and steering values of a mobile robot. This tool was developed as a complementary addition to the "Path  continuity  for  multi-wheeled  AGVs" paper submitted to the RA-L journal. For more information, contact authors.

<img src="https://raw.githubusercontent.com/romb-technologies/path_continuity/ral/figures/app.png" alt="path2" width="750"/> 

## Dependencies

- qt5
- [Bezier](https://github.com/romb-technologies/Bezier)
- [dlib (v19-21)](http://dlib.net/)

## Examples
### Discountinuous path for vehicle with two actuated S/D wheels
Nominal values for vehicle with two actuated wheels moving along two path segments. Both segments utilize "tangential" mode with alpha = 0 and their parametric curves are G_1 continuous. Even though speed limit profiles (solid lines) are not continuous, the simulated speed profile (dashed line) is adjusted to respect the limits. Additionally, it can be observed that all speed profiles reach their limits simultaneously. Vehicle orientation is continuous, but steering angles (solid line) and angular velocities (dashed line) are prominently discontinuous. A real vehicle with finite steering speed can not achieve perfect following, even under ideal conditions. In practice, severe oscillations may occur at the segment junction.

<img src="https://raw.githubusercontent.com/romb-technologies/path_continuity/ral/figures/path_1.png" alt="path1" width="275"/>  <img src="https://raw.githubusercontent.com/romb-technologies/path_continuity/ral/figures/speed_1.png" alt="speed1" width="275"/>  <img src="https://raw.githubusercontent.com/romb-technologies/path_continuity/ral/figures/angle_1.png" alt="angle1" width="275"/>

### Countinuous path for vehicle with two actuated S/D wheels
Nominal values for vehicle with two actuated wheels moving along two path segments. Both segments utilize the "tangential" mode with angle offset alpha = 0, while their parametric curves are G_3 continuous. Speed limit profiles are continuous so there is no need for simulated speed profile to compensate for the discontinuities as in previous example, and additionaly, all of the observed values are now continuous.

<img src="https://raw.githubusercontent.com/romb-technologies/path_continuity/ral/figures/path_2.png" alt="path2" width="275"/>  <img src="https://raw.githubusercontent.com/romb-technologies/path_continuity/ral/figures/speed_2.png" alt="speed2" width="275"/>  <img src="https://raw.githubusercontent.com/romb-technologies/path_continuity/ral/figures/angle_2.png" alt="angle2" width="275"/>

### Countinuous path for vehicle with six actuated S/D wheels
Nominal values for a vehicle with six actuated wheels moving along two path segments with angle offset alpha=14. The first segment utilizes the "tangential" mode, while the second segment utilizes the "exponential" mode with n=1.7. Curves are modified to be continuous. All of the observed speed profiles respect their corresponding limits, and all the observed values are continuous.
 
<img src="https://raw.githubusercontent.com/romb-technologies/path_continuity/ral/figures/path_3.png" alt="path3" width="275"/>  <img src="https://raw.githubusercontent.com/romb-technologies/path_continuity/ral/figures/speed_3.png" alt="speed3" width="275"/>  <img src="https://raw.githubusercontent.com/romb-technologies/path_continuity/ral/figures/angle_3.png" alt="angle3" width="275"/>


## Simulation data

Simulation data used in the article can be found in "simulation_data" folder.
