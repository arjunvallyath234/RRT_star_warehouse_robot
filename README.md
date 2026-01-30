https://github.com/user-attachments/assets/778dd9cd-f704-437f-be51-ee3904a077e3

This simulation demonstrates how RRT* (Rapidly-exploring Random Tree Star) behaves in a dynamic warehouse environment.

🔹 Scenario

A mobile robot must reach a goal location

The environment contains static obstacles (racks/walls)

Other robots move independently while performing their own tasks

Whenever the robot is about to collide with an obstacle or a moving robot, it triggers replanning

RRT* is used to generate a new collision-free path in real time

🔹 What this simulation helps visualize

How RRT* incrementally builds a tree in free space

How replanning occurs when the current path becomes unsafe

Why sampling-based planners are effective in cluttered environments

The difference between finding a path quickly vs optimizing it over time

📌 The green curve shows the current planned trajectory, updated whenever a collision risk is detected.

✅ Why RRT*? (Pros)

✔️ Asymptotically optimal – the path converges to the shortest path given enough samples
✔️ Works well in high-dimensional and complex spaces
✔️ No need for an explicit grid or full map discretization
✔️ Naturally supports online replanning in dynamic environments
✔️ Simple to implement and extend

⚠️ Limitations of RRT* (Cons)

❌ Computationally expensive due to rewiring
❌ Replanning frequency can grow in highly dynamic scenes
❌ Not time-optimal without additional cost shaping
❌ Raw RRT* paths can be jagged (often need smoothing)
❌ No guarantees on real-time performance without constraints

📚 Use cases

Warehouse robots

Mobile robots in shared workspaces

UAV / autonomous navigation

Learning-based planners that use RRT* as a baseline

## Disclaimer
This repository is intended for educational and simulation purposes.
The implementation was created with the assistance of AI tools (ChatGPT)
and manually refined to study RRT* behavior in dynamic environments.

