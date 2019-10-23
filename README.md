# AdvancedRobotics

## Excercise 1.

### Question 1: Explain how Elfin controller computed_torque_controller.cpp implements joint space inverse dynamics controller

By using given URDF description we can build KDL Object.

      if (!kdl_parser::treeFromUrdfModel(urdf, kdl_tree_))
              {
                  ROS_ERROR("Failed to construct kdl tree");
                  return false;
              }
              else
              {
                  ROS_INFO("Constructed kdl tree");
              }
        
From this KDL tree we can build inverse dynamics solver which we can use to compute robot model. 

        // *** 2.2 Compute model(M,C,G) ***
        id_solver_->JntToMass(q_, M_);
        id_solver_->JntToCoriolis(q_, qdot_, C_);
        id_solver_->JntToGravity(q_, G_); 

With this robot model we can calculate the actual control command we give to each joint (with PD values).

        // *** 2.3 Apply Torque Command to Actuator ***
        aux_d_.data = M_.data * (qd_ddot_.data + Kp_.data.cwiseProduct(e_.data) + Kd_.data.cwiseProduct(e_dot_.data));
        comp_d_.data = C_.data + G_.data;
        tau_d_.data = aux_d_.data + comp_d_.data;


### Question 2: What are URDF and KDL packages?

URDF = Unified Robot Description Format is xml based file that describes dimensions for the actual robot model.

<link
    name="elfin_base">
    <inertial>
      <origin
        xyz="-0.0319402056221453 0.427120668618383 -5.39351164605318E-06"
        rpy="0 0 0" />
      <mass
        value="4.3052612459755" />
      <inertia
        ixx="0.291480854811486"
        ixy="-0.00222433961096429"
        ixz="-2.01923964457691E-06"
        iyy="0.0113685309214344"
        iyz="-3.62155285906403E-06"
        izz="0.298007263296196" />
    </inertial>
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://elfin_description/meshes/elfin3/elfin_base.STL" />
      </geometry>
      <material
        name="metal_white"/>
    </visual>
    <collision>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://elfin_description/meshes/elfin3/elfin_base.STL" />
      </geometry>
    </collision>
  </link>

  
KDL = Kinematics and Dynamics Library. It is possible to represent the kinematic chain by a KDL Object. In this excercise we use URDF file to 
build a KDL object. By using this KDL Object we can calculate frame transformations and kinematics for the robot.

### Question 3: ![alt text](https://github.com/betrri/AdvancedRobotics/blob/master/pd%2Bgrav.png)

No implementation was done for this question but rather an study for the given and computed_torque_controller.cpp.

Gravity compensation is done by giving the solver a gravity vector along the z-axis.

        gravity_ = KDL::Vector::Zero(); // ?
        gravity_(2) = -9.81;            // 0: x-axis 1: y-axis 2: z-axis

        id_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));
        
PD gains are used to compute the actual control command

        // *** 2.3 Apply Torque Command to Actuator ***
        aux_d_.data = M_.data * (qd_ddot_.data + Kp_.data.cwiseProduct(e_.data) + Kd_.data.cwiseProduct(e_dot_.data));
        comp_d_.data = C_.data + G_.data;
        tau_d_.data = aux_d_.data + comp_d_.data;
        
 ### Question 4: Implement velocity controller
 
 We get the velocity controller by terminating the Kp values from the computed_torque_controller.cpp
 
 ### Question 5: Implement kinematic controller (using the velocity controller)
 
 Kp_dot = Kp / Kd. This is multiplied to the input of the velocity controller.
 
 ### Question 6:  Implement Task space controller

 --

