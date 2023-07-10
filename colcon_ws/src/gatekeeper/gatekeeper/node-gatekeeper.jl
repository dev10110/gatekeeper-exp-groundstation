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


## RCLPY imports
rclpy = pyimport("rclpy")
node = pyimport("rclpy.node")
# pyimport("dasc_msgs")
dasc_msgs = pyimport("dasc_msgs.msg")
px4_msgs = pyimport("px4_msgs.msg")
geometry_msgs = pyimport("geometry_msgs.msg")
qos = pyimport("rclpy.qos")



# listen on nominal trajectory
# publish on committed trajectory

mutable struct NodeData
    state # current robot state
    state_initialized
    pub_comTraj
    pub_comTrajViz
end

function stateCallback(nodeData, msg)
    # println("got a state msg")

    if !msg.xy_valid || !msg.z_valid || !msg.v_xy_valid || !msg.v_z_valid || !msg.heading_good_for_control
        return 
    end
    
    nodeData.state = msg
    nodeData.state_initialized = true
end


function convertFromNominal(msg) # nominal trajectory message

    N = pylen(msg.poses)

    xs = [pyconvert(Float64, p.position.x) for p in msg.poses]
    ys = [pyconvert(Float64, p.position.y) for p in msg.poses]
    zs = [pyconvert(Float64, p.position.z) for p in msg.poses]
    yaws = [0.0 for p in msg.poses] # TODO: fix

    nomTraj = GK.NominalTrajectory(
                                   0.0,
                                   pyconvert(Float64, msg.dt),
                                   xs, 
                                   ys,
                                   zs, 
                                   yaws
                                  )


    return nomTraj


end

function convertToCommitted(sol_main, sol_branch, dt)

    comTraj = dasc_msgs.DITrajectory()
    comTraj.dt = dt

    for t in range(start=0.0, step=pyconvert(Float64, dt), stop=sol_branch.t[1])
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
        comTraj.accelerations.append(acc)
    end
    
    for t in range(start=sol_branch.t[1], step=pyconvert(Float64, dt), stop=sol_branch.t[end])
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
        comTraj.accelerations.append(acc)
    end


    return comTraj


end

function nomTrajCallbackTimed(nodeData, msg)
    @time nomTrajCallback(nodeData, msg)
end

function convertFromState(state)

    # px4 state is in NED, I need it in ENU

    x0 = ComponentArray(
                        x = [state.y, state.x, -state.z],
                        v = [state.vy, state.vx, -state.vz],
                        R = RS.Rotations.RotZ(state.heading),
                        Ω = zeros(3),
                        ω = RS.hover_ω(GK.quad_params)
                       )

    return x0


function nomTrajCallback(nodeData, msg)

    println("got nomTraj callback")

    if nodeData.state_initialized == false
        return
    end

    comTraj = dasc_msgs.DITrajectory()
    comTraj.header = msg.header
    comTraj.dt = msg.dt 

    nomTraj = convertFromNominal(msg)

    println("coverted to nomTraj")

    # run gatekeeper
    x0 = convertFromState(nodeData.state)

    sfc_A = [
             [1;; 0 ;; 0.0];
             [-1;; 0 ;; 0.0];
             [0;; 1 ;; 0.0];
             [0;; -1 ;; 0.0];
             [0;; 0 ;; 1];
            ]
    sfc_b = [0.5, 0.5, 0.5, 0.5, 1.5]  

    home_sfc = GK.SFC(sfc_A, sfc_b) # small cuboid in the middle that is safe
    fov_sfc = GK.SFC(
                     [[1;; 0 ;; 0.0];
                      [-1;; 0 ;; 0.0];
                      [0;; 1 ;; 0.0];
                      [0;; -1 ;; 0.0];
                      [0;; 0 ;; 1];
                      [0;; 0 ;; -1];
                     ],
                     [1.5, 0.0, 0.5, 0.5, 1.5, 0.0]
                    ) # x <= 1.5

    sfcs = [
            fov_sfc,
            home_sfc
           ]

    suc, sol_main, sol_branch  = GK.gatekeeper(
                                               0.0,
                                               x0,
                                               nomTraj,
                                               nomTraj,
                                               sfcs)


    println("suc: $(suc)")

    if (!suc) 
        return
    end


    comTraj = convertToCommitted(sol_main, sol_branch, msg.dt)

    # N = pylen(msg.poses)
    # for i in 1:N
    #     if pyconvert(Float64, msg.poses[i].position.x) > 1.5
    #         println("ignoring data past pose $(i)")
    #         break
    #     end
    #     comTraj.poses.append(msg.poses[i])
    #     comTraj.twists.append(msg.twists[i])
    #     comTraj.accelerations.append(msg.accelerations[i])
    # end
    
    comTraj.header = msg.header
    
    publish_committed(nodeData, comTraj)
end

function publish_committed(nodeData, trajMsg)

    nodeData.pub_comTraj.publish(trajMsg)

    # constrct the vizualization msg
    vizMsg = geometry_msgs.PoseArray()
    vizMsg.header = trajMsg.header
    vizMsg.poses = trajMsg.poses
    
    nodeData.pub_comTrajViz.publish(vizMsg)

end


function main()

    println("node gatekeeper main()")

    rclpy.init()
    node_ = node.Node("gatekeeper")

    println("gatekeeper node started!!")

    # parameters 
    state = px4_msgs.VehicleLocalPosition() # TODO: initialize

    # publishers
    pub_comTraj = node_.create_publisher(dasc_msgs.DITrajectory, 
                                         "committed_trajectory",
                                         10)
    pub_comTrajViz = node_.create_publisher(geometry_msgs.PoseArray, 
                                         "committed_trajectory/viz",
                                         10)

    # create nodeData struct
    nodeData = NodeData(
                        state, 
                        false,
                        pub_comTraj,
                        pub_comTrajViz
                       )

    # subscribers
    node_.create_subscription(px4_msgs.VehicleLocalPosition, 
                             "px4_1/fmu/out/vehicle_local_position",
                             msg->stateCallback(nodeData, msg),
                             qos.qos_profile_sensor_data)

    node_.create_subscription(dasc_msgs.DITrajectory,
                             "nominal_trajectory", 
                             msg->nomTrajCallbackTimed(nodeData, msg),
                             10) 

    # timers


    # start the spinner
    rclpy.spin(node_)


end
  



## run everything
main()
