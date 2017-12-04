# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Model Documentation
At the start of the project, I was completely lost as I was not sure how to connect the lessons to the problem to be solved. However, the project walkthrough by David Silver and Aaron Brown gave a very good place to start. I was able to get the car moving and avoid collisions with the vehicle ahead.

To be able to switch lanes safely, I chose to use Finite State Machine. I used 3 states to complete this project:

1. Keep Lane: This is the state when the car should continue to drive in the same lane it is in. 
2. Change Lane Right: Execute a change to the right lane
3. Change Lane Left: Execute a change to the left lane

The FSM lessons talk about additional states that could be useful - prepare for lane change to left or right. However, I did not use these states. It would be useful to handle some more complex scenarios where we may want to look behind or the extreme lanes (look at right lane while in left lane). 

Each step, check if the ego vehicle is close to the vehicle ahead. If not, try to reach the max speed.
If too close to vehicle ahead, then check if the center lane has a safe gap. I defined safe gap as 30 meters free space ahead and 5 meters behind in the desired lane. If this gap is available, move to state change lane right. If safe gap is not available, remain in same state, set the desired speed to vehicle ahead of ego vehicle and slow down vehicle till it reaches target speed.

If in center lane, similar logic is used. I check the right lane and left lane. Our target lane is the lane that has larger gap ahead or faster lane if gap on both are same.  If none of the lanes have a safe gap, reduce speed to target speed (speed of vehicle ahead). Based on target lane selection, change lane to left or right.

While in right lane, the logic is very similar to the logic in left lane, except we check for gap on the left. 

### Future enhancements
It would be good to come up with a cost function to decide on the correct lane change behavior.
Also, it would be more helpful to look for gaps on neighboring lanes a little behind ego vehicle, if those lanes are faster than ego lane, but no gap is found around current location. This would enable the vehicle to slow down to reach the gap and then change lanes. 

