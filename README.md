# Craft-Sim

Simple simulation of satellite attitude control with ROS2 as a showcase for job applications. Started as a requirement for a job application.

Since the use of AI was allowed in said job application, Claude code was used to generate the initial draft using the prompt in the `prompt.md` file and pushed with commit [2f1968d](https://github.com/LazieKat/Craft-Sim/commit/2f1968d1c54008ccb18a82b53e80d27fc92f6892). This is the first time I used Claude code, it seems useful although the initial code did not work out of the box and required some corrections in the CMakeLists.txt, Setup.cfg, and renaming a duplicated package. Those changes are in the same commit mentioned earlier.

The next commit includes some of my own modifications but nothing major for this simple demo. I restructured the python ROS2 nodes to be in the same `src` directory. I also added a rate limit for the simulation since real spacecrafts cannot rotate at arbitrary angular velocities.

The point of this demo is to show that I am able of designing code in modules that run in parallel and communicate with each other. It also shows that I know how ROS2 works and I am able to modify and debug code that was not written by me (in that case written by AI).

Issues in the current code include but not limitted to:
- Use of euler angles instead of rotation matrices or quaternion, this is prone to gimbal lock
- No error integral reset logic. Error will accumlate over and cause steady state to be at an offset (integral windup)

I Might expand this project later since the idea is fun to make. I do not plan on letting an AI agent writing the full thing though, it does not follow my style, and it is not as clean as I expected.
