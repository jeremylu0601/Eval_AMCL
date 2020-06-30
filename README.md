# Eval_AMCL

1.  Save_Data.py is used to save the data from Gazebo and AMCL

    It is a ROS node subscribing to gazebo/model_states and amcl_pose. The former is treated as ground truth, and the later is regarded as estimation.

    The data from Gazebo is saved in gt.csv, and the data of AMCL is saved in amcl.csv. Both files are saved in the location where the terminal is. 

    Please execute it after launching the navigation successfully but before clicking “2D Nav Goal” on Rviz

    Please end the execution after “Goal reached” shows on the terminal where the navigation is launched


2.  Plot.py can be used to plot the result.

    It is a script to plot the performance of CPU and GPU on the same figure on 3 different travels (paths)
    
    please make the folder structure like below
    
        >dataset
          >500
            >cpu
              >1
                amcl.csv
                gt.csv
              >2
              >3
            >gpu
              >1
                amcl.csv
                gt.csv
              >2
              >3
          >1000
          >2000
          >4000
          >6000
          >8000
          >10000
          Plot.py
      
       500, 1000, … mean the number of particles, i.e., the value of max_particles and min_particles
       
       1, 2, 3 mean 3 travels

