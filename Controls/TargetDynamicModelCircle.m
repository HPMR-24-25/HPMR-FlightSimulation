function x_dot = TargetDynamicModelCircle(x, t)
x_dot = [
    -pi/8*1000*sin(pi/8*t); 
    pi/8*1000*cos(pi/8*t);
    0;
    0;
    0;
    0];