# Gazebo planar move plugin with uncertainty/drift

## Velocity command uncertainty


``` 
v_true  =  (1 + eps_v) * v_com + eps_vw * w_com 
om_true =                         



```

## Odometry uncertainty

TODO: 
check velocity error noise, static only?
what update rate should be used for velocity comm?
How do we buffer the velocity commands? buffer latest and async read this with mutex in update hook?


