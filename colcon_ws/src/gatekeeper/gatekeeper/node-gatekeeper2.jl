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
import Dates


## RCLPY imports
rclpy = pyimport("rclpy")
node = pyimport("rclpy.node")
dasc_msgs = pyimport("dasc_msgs.msg")
px4_msgs = pyimport("px4_msgs.msg")
geometry_msgs = pyimport("geometry_msgs.msg")
decomp_ros_msgs = pyimport("decomp_ros_msgs.msg")
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

function toVector3(v)
    msg = geometry_msgs.Vector3()
    msg.x = v[1]
    msg.y = v[2]
    msg.z = v[3]
    return msg
end


# listen on nominal trajectory
# publish on committed trajectory

mutable struct NodeData
    node_ # node
    stateMsg # current robot state
    state_initialized
    goalMsg
    goalMsg_initialized
    pub_comTraj
    pub_comTrajViz
    pub_predTrajMainViz
    pub_predTrajBackupViz
    pub_sfcViz
    home_sfc
    sfcs
end



# use a offset to make numbers more reasonable unix time 1689163200 = July 12 2023 12 noon gmt
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
    # run_gatekeeper(nodeData)
end


function convertFromNominal(t0, goalMsg) # nominal trajectory message

    N = 20
    dt = 0.1

    xs = [pyconvert(Float64, goalMsg.pose.position.x) for i = 1:N]
    ys = [pyconvert(Float64, goalMsg.pose.position.y) for i = 1:N]
    zs = [pyconvert(Float64, goalMsg.pose.position.z) for i = 1:N]
    yaws = [getYaw(goalMsg.pose.orientation) for i=1:N]

    # t0 = from_ros_timestamp(goalMsg.header.stamp)

    nomTraj = GK.NominalTrajectory(
                                   t0,
                                   pyconvert(Float64, dt),
                                   xs, 
                                   ys,
                                   zs, 
                                   yaws
                                  )


    return nomTraj


end

function convertToPredicted(sol_main, sol_branch, dt=0.1, frame_id = "vicon/world")

    predTrajMain = geometry_msgs.PoseArray()
    predTrajMain.header.frame_id = frame_id

    t0_s, t0_ns = to_ros_timestamp(sol_main.t[1])
    predTrajMain.header.stamp.sec = t0_s
    predTrajMain.header.stamp.nanosec = t0_ns

    for t in range(start=sol_main.t[1], step=dt, stop=sol_branch.t[1])
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
    predTrajBackup.header.frame_id = frame_id

    tb_s, tb_ns = to_ros_timestamp(sol_branch.t[1])
    predTrajBackup.header.stamp.sec = tb_s
    predTrajBackup.header.stamp.nanosec = tb_ns
    
    for t in range(start=sol_branch.t[1], step=dt, stop=sol_branch.t[end])
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

function rotMatrix_to_rosQuat(R)

  q = RS.Rotations.QuatRotation(R)

  # create geometry_msgs q
  quat = geometry_msgs.Quaternion()

  # copy over data
  quat.x = q.x
  quat.y = q.y
  quat.z = q.z
  quat.w = q.w


  return quat

end



function convertToCommitted(sol_main, sol_branch, dt=0.1, frame_id = "vicon/world", shift=0.1)


    comTraj = dasc_msgs.DITrajectory()
    ros_t_s, ros_t_ns = to_ros_timestamp(sol_main.t[1] - shift)
    comTraj.header.stamp.sec = ros_t_s
    comTraj.header.stamp.nanosec = ros_t_ns
    comTraj.header.frame_id = frame_id
    comTraj.dt = dt

    ax_main(t) = ForwardDiff.derivative(s->sol_main(s; idxs = 4), t)
    ay_main(t) = ForwardDiff.derivative(s->sol_main(s; idxs = 5), t)
    az_main(t) = ForwardDiff.derivative(s->sol_main(s; idxs = 6), t)
    
    ax_branch(t) = ForwardDiff.derivative(s->sol_branch(s; idxs = 4), t)
    ay_branch(t) = ForwardDiff.derivative(s->sol_branch(s; idxs = 5), t)
    az_branch(t) = ForwardDiff.derivative(s->sol_branch(s; idxs = 6), t)

    for t in range(start=sol_main.t[1], step=dt, stop=sol_branch.t[1])
        state = sol_main(t)
        
        pose = geometry_msgs.Pose()
        pose.position.x = state.x[1]
        pose.position.y = state.x[2]
        pose.position.z = state.x[3]
        pose.orientation = rotMatrix_to_rosQuat(state.R)
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
    
    for t in range(start=sol_branch.t[1], step=dt, stop=sol_branch.t[end])
        state = sol_branch(t)
        
        pose = geometry_msgs.Pose()
        pose.position.x = state.x[1]
        pose.position.y = state.x[2]
        pose.position.z = state.x[3]
        pose.orientation = rotMatrix_to_rosQuat(state.R)
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

function convertToCommitted2(goalMsg, sol_main, sol_branch, dt=0.1)


    comTraj = dasc_msgs.DITrajectory()
    ros_t_s, ros_t_ns = to_ros_timestamp(sol_main.t[1])
    comTraj.header.stamp.sec = ros_t_s
    comTraj.header.stamp.nanosec = ros_t_ns
    comTraj.header.frame_id = goalMsg.header.frame_id
    comTraj.dt = dt
    
    #t0 = sol_main.t[1] # in julia time
    Ts = sol_branch.t[1]

    for t in range(start=sol_main.t[1], step=dt, stop=sol_branch.t[end])
          comTraj.poses.append(goalMsg.pose)
          comTraj.twists.append(geometry_msgs.Twist())
          comTraj.accelerations.append(geometry_msgs.Accel())
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
    for t in range(start=Ts, step=dt, stop=sol_branch.t[end])
        comTraj.poses.append(stop_pose)
        comTraj.twists.append(geometry_msgs.Twist())
        comTraj.accelerations.append(geometry_msgs.Accel())
    end

    # print some stats here
    # println("NomTraj: [$(t0_nomTraj)::$(t0_nomTraj + traj_nom.dt * (N-1))]  Backup: [$(Ts) :: $(sol_branch.t[end])")

    return comTraj


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


function convertFromSFC(poly)

   N = pylen(poly.ps)
   A = zeros(N, 3)
   b = zeros(N)

   for i=1:N
       py_p = poly.ps[i-1]
       py_n = poly.ns[i-1]

       p = [pyconvert(Float64,s) for s in (py_p.x, py_p.y, py_p.z)]
       n = [pyconvert(Float64,s) for s in (py_n.x, py_n.y, py_n.z)]

       A[i, :] .= n
       b[i] = dot(n, p)
   end

   return GK.SFC(A, b)
end

function sfcArrayCallback(nodeData, msg)

    if (pyconvert(String, msg.header.frame_id) != "vicon/world")
        println("sfc is not in world frame")
        return
    end

    # nodeData.sfcs = [nodeData.home_sfc]
    nodeData.sfcs = GK.SFC[]

    N = pylen(msg.polys)
    for i=1:N
        sfc = convertFromSFC(msg.polys[i-1].poly)
        push!(nodeData.sfcs, sfc)
    end

end

function sfcCallback(nodeData, msg)

    if (pyconvert(String, msg.header.frame_id) != "vicon/world")
        println("sfc is not in world frame")
        return
    end

    # nodeData.sfcs = GK.SFC[]
    sfc = convertFromSFC(msg.poly)
    nodeData.sfcs = [nodeData.home_sfc, sfc]
    # nodeData.sfcs = [ sfc]

    println("running sfc callback")

    return

end

function goalMsgCallbackTimed(nodeData, msg)
    goalMsgCallback(nodeData, msg)
end


function goalMsgCallback(nodeData, msg)

    world_frame = "vicon/world" 
    if pyconvert(String, msg.header.frame_id) != world_frame 
        nodeData.node_.get_logger().warn("goal_msg is not in $(world_frame), is in $(msg.header.frame_id). ignorning msg")
    end

    nodeData.goalMsg = msg
    nodeData.goalMsg_initialized = true

end


# function past_nomTraj(nodeData)
# 
#     t_now = from_ros_timestamp(nodeData.node_.get_clock().now().to_msg())
# 
#     t0_nomTraj = from_ros_timestamp(nodeData.nomTrajMsg.header.stamp)
# 
#     N = pylen(nodeData.nomTrajMsg.poses)
#     dt = pyconvert(Float64, nodeData.nomTrajMsg.dt)
# 
#     return t_now >= t0_nomTraj + dt * (N -1)
# end
        


function run_gatekeeper(nodeData)

    if ! nodeData.goalMsg_initialized 
        return
    end

    if ! nodeData.state_initialized 
        return
    end


    # convert ros messages to julia format
    t0, x0 = convertFromState(nodeData.stateMsg)
    nomTraj = convertFromNominal(t0, nodeData.goalMsg)
    
    # use the gatekeeper module to get the trajectory
    suc, sol_main, sol_branch  = GK.gatekeeper(
                                               t0,
                                               x0,
                                               nomTraj,
                                               nodeData.sfcs)


    # check if replanning was successful
    if (!suc) 
        # println("suc: $(suc)")
        return
    end


    # convert the message to ros message
    # comTraj = convertToCommitted2(nodeData.nomTrajMsg, sol_branch)
    # comTraj = convertToCommitted2(nodeData.goalMsg, sol_main, sol_branch)
    comTraj = convertToCommitted(sol_main, sol_branch)
    # comTraj = convertToCommitted(nodeData.nomTrajMsg, sol_main, sol_branch)
    predTrajs = convertToPredicted(sol_main, sol_branch)

    # publish
    publish_committed(nodeData, comTraj)
    # publish_predicted(nodeData, predTrajs)
    publish_sfcs_viz(nodeData, comTraj.header)
end

function sfc2polymsg(sfc, header)

    poly = decomp_ros_msgs.PolyhedronStamped()
    poly.header = header 

    N = size(sfc.A, 1)
    for i=1:N
        # convert each to a polyhedron
        n = toVector3(sfc.A[i, :])
        p = geometry_msgs.Point()

        if (sfc.A[i, 1] != 0)
          p.x = sfc.b[i]/sfc.A[i,1]
        elseif (sfc.A[i, 2] != 0)
            p.y = sfc.b[i]/sfc.A[i,2]
        elseif (sfc.A[i, 3] != 0)
            p.z = sfc.b[i] / sfc.A[i, 3]
        end

        poly.poly.ns.append(n)
        poly.poly.ps.append(p)
    end

    return poly

end

function publish_sfcs_viz(nodeData, header)

    msg = decomp_ros_msgs.PolyhedronArray()
    msg.header = header
    
    for sfc in nodeData.sfcs
        poly_msg = sfc2polymsg(sfc, header)
        msg.polys.append(poly_msg)
    end

    nodeData.pub_sfcViz.publish(msg)
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
    pub_sfcViz= node_.create_publisher(decomp_ros_msgs.PolyhedronArray, 
                                       "gatekeeper/sfc_debug",
                                         10)

    # create the home sfc
    sfc_A = [
             [1;; 0 ;; 0.0];
             [-1;; 0 ;; 0.0];
             [0;; 1 ;; 0.0];
             [0;; -1 ;; 0.0];
             [0;; 0 ;; 1];
             [0;; 0 ;; -1];
            ]
    sfc_b = [0.5, 0.5, 0.5, 0.5, 1.5, 0.5]  
    home_sfc = GK.SFC(sfc_A, sfc_b) # small cuboid in the middle that is safe

    # create nodeData struct
    nodeData = NodeData(
                        node_,
                        px4_msgs.VehicleLocalPosition(),
                        false,
                        # dasc_msgs.DITrajectory(),
                        geometry_msgs.PoseStamped(),
                        false,
                        pub_comTraj,
                        pub_comTrajViz,
                        pub_predTrajMainViz,
                        pub_predTrajBackupViz,
                        pub_sfcViz,
                        home_sfc,
                        [home_sfc]
                       )

    # subscribers
    node_.create_subscription(px4_msgs.VehicleLocalPosition, 
                             "px4_1/fmu/out/vehicle_local_position",
                             msg->stateCallback(nodeData, msg),
                             qos.qos_profile_sensor_data)

    # node_.create_subscription(dasc_msgs.DITrajectory,
    #                          "nominal_trajectory", 
    #                          msg->nomTrajCallbackTimed(nodeData, msg),
    #                          1) 

    node_.create_subscription(geometry_msgs.PoseStamped(),
                             "goal_pose", 
                             msg->goalMsgCallbackTimed(nodeData, msg),
                             1) 

    # node_.create_subscription(decomp_ros_msgs.PolyhedronArray(),
    #                         "/camera/sfc_array",
    #                         msg->sfcArrayCallback(nodeData, msg),
    #                         1)
    
    node_.create_subscription(decomp_ros_msgs.PolyhedronStamped(),
                            "/nvblox_node/sfc",
                            msg->sfcCallback(nodeData, msg),
                            1)
    # timers
    node_.create_timer(0.2, ()->run_gatekeeper(nodeData))

    # start the spinner
    rclpy.spin(node_)

    # run the shutdown
    # node_.destroy_node()
    # rclpy.shutdown()


end
  

## run everything
main()
