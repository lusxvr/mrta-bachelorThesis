# mrta-bachelorThesis

Repository for my Bachelor Thesis at RSI TUM

Solving a Multi Robot Task Allocation and Scheduling Problem as a Multiple Traveling Salesman Problem

## Structure

File Structure follows a complexity evolution:

basic -> ... + capacities -> ... + time windows -> ... + individual agent costs -> ... + json output -> ... + yaml output

The old versions with the task allocation algorithm in a lower complexity are in /oldVersions

The current and most recent file is mtsp_main.py

For evaluation, the eval_algo.py is used. This runns, a random generated scenario with a specified amount of tasks for a specified amount of iterations. Three different allocation algorithms are compared: random allocation, nearest neighbour allocation and the developed allocation using OR-Tools. It creates json files with the calculated route distance for each agent for each approach for the number of iterations.

File Structure: [[[mrta_iter1_r1, mrta_iter1_r2],[rand_iter1_r1, rand_iter1_r2],[nn_iter1_r1, nn_iter1_r2]], [... for iter2], ...]

Naming Structure: dayofyear_hour_min_sec_numtasks_numiter.json

To calculate metrics and visualisation from the data, data_analysis_benchmarks.py is used. 


## Json Output

Make sure to have template.json in the directory

Algorithm creates json File for each assigned task which follow this naming scheme:

#### xytask_z.json

x: Number of the Agent/Route this task is assigned to (starts at 0)

y: Order/Sequence of the Tasks in the Route (starts at 0)

Z: Name/Identifier of the assigned task to this agent-time slot

#### e.g.:

00task_1.json - As the first task for the first agent the algorithm assigned task 1

01task_4.json - As the second task for the first agent the algorithm assigned task 4

13task_5.json - As the fourth task for the second agent the algorithm assigned task 6

## Yaml Output

Make sure to have template.yaml in the directory

Creates skills.yaml file which holds the generated string sequence for executing the tasks with the robots

One assigned Task is splitted into Skills:

For Pickup:
1. JointPose TravelPose
2. BasePose
3. JointPose workbench_scan_pose
4. UpdateObjectPose
5. JointPose storage_scan_pose
6. UpdateStorage
7. CartPose (Obj greifen)
8. Gripper
9. JointPose (Einlager)
10. CartPose (Gripper release)

For Dropoff:
1. JointPose (zu obj in lager) [Pickup 9]
2. Gripper [Pickup 8]
3. CartPose (zu Endlocation)
4. CartPose (Gripper release)

Pickup and Dropoff Skills are generated together, saved to different lists and merged at the end to create the correct sequence in one go