workspace based planning for painting robot aims to generate below variables:
1. mobile platform positions based on the defined effective workspace
2. rod climbing mechanism positions and manipulator trajectories for coverage painting planning inside each defined effective workspace 


The applied planning algorithm before 202007 is shown as follows:
1. The generation and visualization of planning result: roslaunch painting_robot_demo paintingrobot_states_visualization
2. The applied python files include:
2.1. coverage_planning_offline_farubim.py for generating planning result
2.2. paintingrobot_planningresult_visualization.py for visualizing planning result 

The proposed optimal coverage planning algorithm on 202008 is shown as follows:
1. the modified planning file target: main1.m, and main2.m 

coverage_planning_offline_farubim.py
2. the visualization file can be modified for the visualization of rod climbing mechanism

the visualization is:
1. painting_globalpaths_visualization.py
2. painting_mobileplatform_cells_visualization.py

targets list is shown as follows:
1. write mobileplatform_planner.py
2. write climbing_mechanism_and_manipulator_planner.py
3. modify painting_mobileplatform_cells_visualization.py

algorithm: a novel rtsp algorithm 
input: renovation_cell_waypaths, renovation_cell_mobileplatform_positions
output: climbing_mechanism and manipulator_joints   














