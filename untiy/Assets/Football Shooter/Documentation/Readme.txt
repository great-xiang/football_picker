
How to use 



Open the Demo scene and shoot the ball and also you can control the ball during play.
The GAMEOBJECT SoccorBall has the chlid object Football1 which has the script "ShootAI.cs"
if you want to change the ball make the new ball the child object of the SoccorBall Gameobject and apply all the components to new ball which are already on football1
Various variables are defined to controll the ball in the script.
These variables are in "Shoot.cs" that is the parent class of the "ShootAI.cs" script.
In order to change the behaviour of the ball you just have the change the values of the variable that are defined in "Shoot.cs" on line 37 to line 71.
If you want to change the reset position of the ball please change the Mindistance and MaxDistance insode Shoot.cs
If cloth is attached to the net you have to fix every node of cloth component in order to achieve accuracy.

NOTE: Please change the value inside the Script (in start method), do not alter the values in editor as they are initlized in the script so editor changes has no effect on the scripts.
Lower the values of variables in the script "Shoot.cs" in order to achieve accuracy
 