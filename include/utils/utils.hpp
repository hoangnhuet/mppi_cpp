#include<iostream>
#include<bits/stdc++.h>
class Twist
{
    float linear;
    float angular;
    Twist(float lin, float ang)
    {
        linear = lin;
        angular = ang;
    }
    Twist()
    {
        linear = 0;
        angular = 0;
    }
};
class Pose
{
    float x,y,yaw;
};
typedef std::vector<Pose> Path; 
