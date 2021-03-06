# mrta-bachelorThesis

Repository for my Bachelor Thesis at RSI TUM

Solving a Multi Robot Task Allocation and Scheduling Problem as a Multiple Traveling Salesman Problem

## Structure

File Structure follows a complexity evolution:

basic -> ... + capacities -> ... + time windows -> ... + individual agent costs -> ... + json output -> ... + yaml output

The old versions with the task allocation algorithm in a lower complexity are in /oldVersions

The current and most recent file is mtsp_main.py


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

One assigned Task is splitted into 3 Skills:

1. Move to neutral Pose for Base Movement
2. New Base Pose
3. New Cart Pose