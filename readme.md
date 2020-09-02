
# the intial framework for our problem  
# while(1):
    # 1. sample climbing mechanism joints 
    # 2. for each climbing mechanism joints
        # 2.1 obtain the corresponding sampled manipulator base positions based on the kinematic model
        # 2.2 obtain the waypaths inside the workspace of sampled manipulator base positions with octotree
        # 2.3 connect these waypaths with Cartesian-space tsp
        # 2.4 obtain the joint-space tsp solver
    # 3. compute the motion cost combining climbing mechanism and manipulator 
    # 4. compute the painting awards
    # 5. based on step 3 and step 4, the joint list is computed.
    # the painting cost is the motion cost 
    # the painting award is the net painting area 

# the modified framework for our problem  
# while(1):
    # 1. sample climbing mechanism joints 
    # 2. for each climbing mechanism joints
        # 2.1 obtain the corresponding sampled manipulator base positions based on the kinematic model
        # 2.2 select the waypaths inside the workspace of sampled manipulator base positions 
        # 2.3 select again to obtain the waypaths on which waypoints has colision-free inverse kinematic solutions [] 
        # 2.3 connect these selected waypaths with Cartesian-space tsp solver
        # 2.4 obtain the joint-space tsp solver
    # 3. compute the motion cost combining climbing mechanism and manipulator 
    # 4. compute the painting awards
    # 5. based on step 3 and step 4, the joint list is computed.
    # the painting cost is the motion cost 
    # the painting award is the net painting area 

# the reference website is:
https://blog.csdn.net/Bobsweetie/article/details/70912036

https://blog.csdn.net/u013834525/article/details/80447931

https://blog.csdn.net/yaked/article/details/52996509
