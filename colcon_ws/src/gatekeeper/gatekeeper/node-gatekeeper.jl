ENV["JULIA_CONDAPKG_BACKEND"] = "Null"
ENV["JULIA_PYTHONCALL_EXE"] = "/usr/bin/python3"

println("node gatekeeper file started")

using PythonCall

include("gatekeeper-julia.jl")

using .Gatekeeper
GK = Gatekeeper

import RoboticSystems
RS = RoboticSystems

using ComponentArrays
using LinearAlgebra
using ForwardDiff


## RCLPY imports
rclpy = pyimport("rclpy")
node = pyimport("rclpy.node")
dasc_msgs = pyimport("dasc_msgs.msg")
px4_msgs = pyimport("px4_msgs.msg")
geometry_msgs = pyimport("geometry_msgs.msg")
qos = pyimport("rclpy.qos")
transforms3d = pyimport("transforms3d")
np = pyimport("numpy")


function getYaw(q)
    r, p, y = transforms3d.euler.quat2euler([q.w, q.x, q.y, q.z])
    return pyconvert(Float64, y)
end

function yaw2quat(yaw)
    q = transforms3d.quaternions.axangle2quat([0,0,1], pi/2 - yaw)
    return q
end

function rot2quat(R)
    M = np.zeros([3,3])
    for i=1:3, j=1:3
        M[i-1,j-1] = R[i,j]
    end
    q = transforms3d.quaternions.mat2quat(M)
    return q
end


# listen on nominal trajectory
# publish on committed trajectory

mutable struct NodeData
    node_ # node
    stateMsg # current robot state
    state_initialized
    nomTrajMsg
    nomTraj_initialized
    pub_comTraj
    pub_comTrajViz
    pub_predTrajMainViz
    pub_predTrajBackupViz
end



# use a offset to make numbers more reasonable unix time 1689163200 = July 12 2023 12 noon gmt
import Dates
const T_OFFSET_SECONDS = floor(Int64, Dates.datetime2unix(Dates.now())) # 1689163200
# const T_OFFSET_SECONDS = 1689163200

function from_ros_timestamp(stamp)
    t = from_ros_timestamp(pyconvert(Float64, stamp.sec), pyconvert(Float64, stamp.nanosec))
    return t
end

function from_ros_timestamp(sec, nsec)
    return Float64(sec - T_OFFSET_SECONDS) + 1e-9 * nsec
end

function to_ros_timestamp(t)
    sec = floor(Int, t)
    nsec = floor(Int, (t - sec) * 1e9)
    return (sec + T_OFFSET_SECONDS, nsec)
end


function stateCallback(nodeData, msg)
    # println("got a state msg")

    if pynot(msg.xy_valid) || pynot(msg.z_valid) || pynot(msg.v_xy_valid) || pynot(msg.v_z_valid) || pynot(msg.heading_good_for_control)
        return 
    end
    
    nodeData.stateMsg = msg
    nodeData.state_initialized = true


    # replan committed trajectory! 
    run_gatekeeper(nodeData)
end


function convertFromNominal(msg) # nominal trajectory message

    N = pylen(msg.poses)

    xs = [pyconvert(Float64, p.position.x) for p in msg.poses]
    ys = [pyconvert(Float64, p.position.y) for p in msg.poses]
    zs = [pyconvert(Float64, p.position.z) for p in msg.poses]
    yaws = [getYaw(p.orientation) for p in msg.poses]

    t0 = from_ros_timestamp(msg.header.stamp)

    nomTraj = GK.NominalTrajectory(
                                   t0,
                                   pyconvert(Float64, msg.dt),
                                   xs, 
                                   ys,
                                   zs, 
                                   yaws
                                  )


    return nomTraj


end

function convertToPredicted(nomTraj, sol_main, sol_branch)

    predTrajMain = geometry_msgs.PoseArray()
    predTrajMain.header = nomTraj.header

    for t in range(start=0.0, step=pyconvert(Float64, nomTraj.dt), stop=sol_branch.t[1])
        state = sol_main(t)
        
        pose = geometry_msgs.Pose()
        pose.position.x = state.x[1]
        pose.position.y = state.x[2]
        pose.position.z = state.x[3]
        q = rot2quat(state.R)
        pose.orientation.w = q[0]
        pose.orientation.x = q[1]
        pose.orientation.y = q[2]
        pose.orientation.z = q[3]
        predTrajMain.poses.append(pose)
    end
    
    predTrajBackup = geometry_msgs.PoseArray()
    predTrajBackup.header = nomTraj.header
    
    for t in range(start=sol_branch.t[1], step=pyconvert(Float64, nomTraj.dt), stop=sol_branch.t[end])
        state = sol_branch(t)
        
        pose = geometry_msgs.Pose()
        pose.position.x = state.x[1]
        pose.position.y = state.x[2]
        pose.position.z = state.x[3]
        q = rot2quat(state.R)
        pose.orientation.w = q[0]
        pose.orientation.x = q[1]
        pose.orientation.y = q[2]
        pose.orientation.z = q[3]
        predTrajBackup.poses.append(pose)
    end

    return predTrajMain, predTrajBackup

end

function convertToCommitted(nomTrajMsg, sol_main, sol_branch)

    t0 = sol_main.t[1]
    
    ros_t_s, ros_t_ns = to_ros_timestamp(t0)

    comTraj = dasc_msgs.DITrajectory()
    comTraj.header.stamp.sec = ros_t_s
    comTraj.header.stamp.nanosec = ros_t_ns
    comTraj.header.frame_id = nomTrajMsg.header.frame_id
    comTraj.dt = nomTrajMsg.dt

    ax_main(t) = ForwardDiff.derivative(s->sol_main(s; idxs = 4), t)
    ay_main(t) = ForwardDiff.derivative(s->sol_main(s; idxs = 5), t)
    az_main(t) = ForwardDiff.derivative(s->sol_main(s; idxs = 6), t)
    
    ax_branch(t) = ForwardDiff.derivative(s->sol_branch(s; idxs = 4), t)
    ay_branch(t) = ForwardDiff.derivative(s->sol_branch(s; idxs = 5), t)
    az_branch(t) = ForwardDiff.derivative(s->sol_branch(s; idxs = 6), t)

    for t in range(start=sol_main.t[1], step=pyconvert(Float64, comTraj.dt), stop=sol_branch.t[1])
        state = sol_main(t)
        
        pose = geometry_msgs.Pose()
        pose.position.x = state.x[1]
        pose.position.y = state.x[2]
        pose.position.z = state.x[3]
        # pose.orientation = # TODO!
        comTraj.poses.append(pose)
        # println("MAIN: $(state.x)")

        twist = geometry_msgs.Twist()
        twist.linear.x = state.v[1]
        twist.linear.y = state.v[2]
        twist.linear.z = state.v[3]
        comTraj.twists.append(twist)

        acc = geometry_msgs.Accel()
        acc.linear.x = ax_main(t)
        acc.linear.y = ay_main(t) 
        acc.linear.z = az_main(t)
        comTraj.accelerations.append(acc)
    end
    
    for t in range(start=sol_branch.t[1], step=pyconvert(Float64, comTraj.dt), stop=sol_branch.t[end])
        state = sol_branch(t)
        
        pose = geometry_msgs.Pose()
        pose.position.x = state.x[1]
        pose.position.y = state.x[2]
        pose.position.z = state.x[3]
        # pose.orientation = # TODO!
        comTraj.poses.append(pose)
        # println("BRANCH: $(state.x)")

        twist = geometry_msgs.Twist()
        twist.linear.x = state.v[1]
        twist.linear.y = state.v[2]
        twist.linear.z = state.v[3]
        comTraj.twists.append(twist)

        acc = geometry_msgs.Accel()
        acc.linear.x = ax_branch(t)
        acc.linear.y = ay_branch(t) 
        acc.linear.z = az_branch(t)
        comTraj.accelerations.append(acc)
    end


    return comTraj


end

function convertToCommitted2(traj_nom, sol_branch)


    comTraj = dasc_msgs.DITrajectory()
    comTraj.header.stamp = traj_nom.header.stamp
    comTraj.header.frame_id = traj_nom.header.frame_id
    comTraj.dt = traj_nom.dt

    #t0 = sol_main.t[1] # in julia time
    Ts = sol_branch.t[1]
    t0_nomTraj = from_ros_timestamp(traj_nom.header.stamp)

    N = pylen(traj_nom.poses)


    # first append all the poses until Ts
    for i in 1:N

        t = t0_nomTraj + pyconvert(Float64, traj_nom.dt) * (i-1) 
        if t < Ts
          comTraj.poses.append(traj_nom.poses[i-1])
          comTraj.twists.append(traj_nom.twists[i-1])
          comTraj.accelerations.append(traj_nom.accelerations[i-1])
      end

    end
    
    # now determine the stop target
    target_state = sol_branch.prob.p[1]
    stop_pos = target_state[1]
    b1d = target_state[4]
    stop_yaw = atan(b1d[1], b1d[2])
    
    stop_pose = geometry_msgs.Pose()
    stop_pose.position.x = stop_pos[1]
    stop_pose.position.y = stop_pos[2]
    stop_pose.position.z = stop_pos[3]
    q = yaw2quat(stop_yaw)
    stop_pose.orientation.w = q[0]
    stop_pose.orientation.x = q[1]
    stop_pose.orientation.y = q[2]
    stop_pose.orientation.z = q[3]

    # now append the stop target
    for t in range(start=Ts, step=pyconvert(Float64, comTraj.dt), stop=sol_branch.t[end])

        comTraj.poses.append(stop_pose)

        twist = geometry_msgs.Twist()
        comTraj.twists.append(twist)

        acc = geometry_msgs.Accel()
        comTraj.accelerations.append(acc)
    end

    # print some stats here?
    println("NomTraj: [$(t0_nomTraj)::$(t0_nomTraj + traj_nom.dt * (N-1))]  Backup: [$(Ts) :: $(sol_branch.t[end])")

    return comTraj


end

function nomTrajCallbackTimed(nodeData, msg)
    nomTrajCallback(nodeData, msg)
end

function convertFromState(stateMsg)

    # px4 state is in NED, I need it in ENU

    R_ = RS.Rotations.RotZ(pi/2 - pyconvert(Float64, stateMsg.heading)) |> collect

    x0 = ComponentArray(
                        x = pyconvert(Vector{Float64}, [stateMsg.y, stateMsg.x, -stateMsg.z]),
                        v = pyconvert(Vector{Float64}, [stateMsg.vy, stateMsg.vx, -stateMsg.vz]),
                        R = R_,
                        Ω = zeros(3),
                        ω = RS.hover_ω(GK.quad_params)
                       )

    t_us = pyconvert(Int, stateMsg.timestamp_sample)
    t_s = floor(Int, t_us / 1e6)
    t_ns = 1e3 * (t_us - 1e6 * t_s)
    t0 = t_s - T_OFFSET_SECONDS + t_ns * 1e-9 

    return t0, x0
end


function nomTrajCallback(nodeData, msg)

    world_frame = "vicon/world" 
    if pyconvert(String, msg.header.frame_id) != "vicon/world"
        nodeData.node_.get_logger().warn("nom trajectory is not in $(world_frame), is in $(msg.header.frame_id). ignorning msg")
    end

    nodeData.nomTrajMsg = msg
    nodeData.nomTraj_initialized = true

end


function past_nomTraj(nodeData)

    t_now = from_ros_timestamp(nodeData.node_.get_clock().now().to_msg())

    t0_nomTraj = from_ros_timestamp(nodeData.nomTrajMsg.header.stamp)

    N = pylen(nodeData.nomTrajMsg.poses)
    dt = pyconvert(Float64, nodeData.nomTrajMsg.dt)

    return t_now >= t0_nomTraj + dt * (N -1)
end
        


function run_gatekeeper(nodeData)

    if ! nodeData.nomTraj_initialized 
        return
    end

    if ! nodeData.state_initialized 
        return
    end


    println("RUNNING GATEKEEPER!")


    ## check if the nomTraj has run out
    if past_nomTraj(nodeData)
        nodeData.nomTraj_initialized = false
        println("past nom traj; pausing")
        return
    end


    comTraj = dasc_msgs.DITrajectory()
    comTraj.header = nodeData.nomTrajMsg.header
    comTraj.dt = nodeData.nomTrajMsg.dt 


    # convert ros messages to julia format
    t0, x0 = convertFromState(nodeData.stateMsg)
    nomTraj = convertFromNominal(nodeData.nomTrajMsg)

    # convert the sfcs
    sfc_A = [
             [1;; 0 ;; 0.0];
             [-1;; 0 ;; 0.0];
             [0;; 1 ;; 0.0];
             [0;; -1 ;; 0.0];
             [0;; 0 ;; 1];
            ]
    sfc_b = [0.5, 0.5, 0.5, 0.5, 1.5]  
    home_sfc = GK.SFC(sfc_A, sfc_b) # small cuboid in the middle that is safe
    

    # dummy sfc for testing
    fov_sfc = GK.SFC(
                     [[1;; 0 ;; 0.0];
                      # [-1;; 0 ;; 0.0];
                      # [0;; 1 ;; 0.0];
                      # [0;; -1 ;; 0.0];
                      # [0;; 0 ;; 1];
                      # [0;; 0 ;; -1];
                     ],
                     [100.5,
		      # 0.0, 0.5, 0.5, 1.5, 0.0
		      ]
                    ) # x <= 1.5

    sfcs = [
            fov_sfc,
            home_sfc
           ]

    # get the timing right: use the current state timestamp as the 

    # use the gatekeeper module to get the trajectory
    suc, sol_main, sol_branch  = GK.gatekeeper(
                                               t0,
                                               x0,
                                               nomTraj,
                                               sfcs)


    # check if replanning was successful
    if (!suc) 
        println("suc: $(suc)")
        return
    end


    # convert the message to ros message
    comTraj = convertToCommitted2(nodeData.nomTrajMsg, sol_branch)
    # comTraj = convertToCommitted(nodeData.nomTrajMsg, sol_main, sol_branch)
    predTrajs = convertToPredicted(nodeData.nomTrajMsg, sol_main, sol_branch)

    # publish
    publish_committed(nodeData, comTraj)
    publish_predicted(nodeData, predTrajs)
    # publish_sfcs_viz(nodeData, sfcs)
end

function publish_committed(nodeData, trajMsg)

    nodeData.pub_comTraj.publish(trajMsg)

    # constrct the vizualization msg
    vizMsg = geometry_msgs.PoseArray()
    vizMsg.header = trajMsg.header
    vizMsg.poses = trajMsg.poses
    
    nodeData.pub_comTrajViz.publish(vizMsg)

end

function publish_predicted(nodeData, predTrajs)

    nodeData.pub_predTrajMainViz.publish(predTrajs[1])
    nodeData.pub_predTrajBackupViz.publish(predTrajs[2])
end


function main()

    println("node gatekeeper main()")

    rclpy.init()
    node_ = node.Node("gatekeeper")

    println("gatekeeper node started!!")

    # parameters 

    # publishers
    pub_comTraj = node_.create_publisher(dasc_msgs.DITrajectory, 
                                         "committed_trajectory",
                                         10)
    pub_comTrajViz = node_.create_publisher(geometry_msgs.PoseArray, 
                                         "committed_trajectory/viz",
                                         10)
    pub_predTrajMainViz = node_.create_publisher(geometry_msgs.PoseArray, 
                                         "predicted_trajectory/main/viz",
                                         10)
    pub_predTrajBackupViz = node_.create_publisher(geometry_msgs.PoseArray, 
                                         "predicted_trajectory/backup/viz",
                                         10)

    # create nodeData struct
    nodeData = NodeData(
                        node_,
                        px4_msgs.VehicleLocalPosition(),
                        false,
                        dasc_msgs.DITrajectory(),
                        false,
                        pub_comTraj,
                        pub_comTrajViz,
                        pub_predTrajMainViz,
                        pub_predTrajBackupViz
                       )

    # subscribers
    node_.create_subscription(px4_msgs.VehicleLocalPosition, 
                             "px4_1/fmu/out/vehicle_local_position",
                             msg->stateCallback(nodeData, msg),
                             qos.qos_profile_sensor_data)

    node_.create_subscription(dasc_msgs.DITrajectory,
                             "nominal_trajectory", 
                             msg->nomTrajCallbackTimed(nodeData, msg),
                             1) 

    # timers


    # start the spinner
    rclpy.spin(node_)


end
  



## run everything
main()
