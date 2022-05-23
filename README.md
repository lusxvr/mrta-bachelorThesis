# mrta-bachelorThesis

Repository for my Bachelor Thesis at RSI TUM

Solving a Multi Robot Task Allocation and Scheduling Problem as a Multiple Traveling Salesman Problem

## Structure

File Structure follows a complexity evolution:

basic -> capacities -> capacities + time windows -> capacities + time windows + individual agent costs


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
