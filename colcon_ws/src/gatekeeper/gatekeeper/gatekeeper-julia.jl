module Gatekeeper

using LinearAlgebra
using ComponentArrays
using StaticArrays
using Interpolations
using ForwardDiff


using DifferentialEquations

using RoboticSystems
RS = RoboticSystems

const quad_params = ComponentArray(
  RS.quadrotor_parameters;
  mass=0.8,
  J = diagm([0.0025, 0.0025, 0.004]) |> collect, 
  k_f = 1.37e-6, # TODO(dev): check!
  k_μ = 1.37e-7,  # TODO(dev): check!
  ω_max = 2500.0,
) |> RS.initialize_quad_params

const cont_params = ComponentArray(
    kx = 11.29,
    kv = 6.97,
    kR = 1.2,
    kΩ = 0.1,
    m = quad_params.mass,
    J = quad_params.J,
    invG = quad_params.invG, 
    g = 9.81
)


struct NominalTrajectory
    t0
    dt
    xs
    ys
    zs
    yaws 
end


# function interpolate_flat_trajectory(time, traj)
# 
#     index = floor(Int, (time - traj.t0) /  traj.dt) + 1
# 
#     dt = time - (traj.t0 + (index - 1) * traj.dt)
# 
#     # construct matrix for forwardprop
#     A = [
#          [ 1  ;; dt ;; dt^2/2 ;; dt^3/6 ;; dt^4/24];
#          [ 0  ;; 1  ;; dt ;; dt^2/2 ;; dt^3/6 ];
#          [ 0  ;; 0  ;; 1  ;; dt ;; dt^2/2 ];
#          [ 0  ;; 0  ;; 0  ;; 1  ;; dt ];
#          [ 0  ;; 0  ;; 0  ;; 0  ;; 1  ];
#         ]
# 
#     S = [
#          [traj.xs..., traj.yaws];
#          [traj.vs..., traj.yawspeeds];
#          [traj.as..., traj.yawaccels];
#          [traj.js..., 0];
#          [traj.ss..., 0];
#         ]
# 
#     R = A * S
# 
# 
#     return R[1, 1:3], R[2, 1:3], R[3, 1:3], R[4, 1:3], R[5, 1:3], R[1, 4], R[2, 4], R[3, 4]
# 
# end

function construct_interpolants(traj)
    # assumes traj is in the form of traj.t0, traj.dt, traj.positions
    # fits a cubic spline to the data
    # returns the new flat state


    ts = range(;start = traj.t0, step = traj.dt, length=length(traj.xs))

    interp_x   = cubic_spline_interpolation(ts, traj.xs)
    interp_y   = cubic_spline_interpolation(ts, traj.ys)
    interp_z   = cubic_spline_interpolation(ts, traj.zs)
    interp_yaw = cubic_spline_interpolation(ts, traj.yaws)

    interp_vx(x) = ForwardDiff.derivative(interp_x, x)
    interp_ax(x) = ForwardDiff.derivative(interp_vx, x)

    interp_vy(y) = ForwardDiff.derivative(interp_y, y)
    interp_ay(y) = ForwardDiff.derivative(interp_vy, y)
    
    interp_vz(z) = ForwardDiff.derivative(interp_z, z)
    interp_az(z) = ForwardDiff.derivative(interp_vz, z)
    interp_yawspeed(yaw) = ForwardDiff.derivative(interp_yaw, yaw)

    szeros = @SVector zeros(3)
    
    target_state(t) = RS.flat_state_to_quad_state(
                                                  SA[interp_x(t), interp_y(t), interp_z(t)],
                                                  SA[interp_vx(t), interp_vy(t), interp_vz(t)],
                                                  SA[interp_ax(t), interp_ay(t), interp_az(t)],
                                                  szeros, # jerk
                                                  szeros, # snap
                                                  interp_yaw(t),
                                                  interp_yawspeed(t),
                                                  0.0 # yaw accel
                                                 )
    return target_state


end

function test_interpolants(traj)
    # assumes traj is in the form of traj.t0, traj.dt, traj.positions
    # fits a cubic spline to the data
    # returns the new flat state


    v = 1.0;

    interp_x(t) = v * t 
    interp_y(t) = 0.0
    interp_z(t)   = 1.0
    interp_yaw(t) = 0.0

    interp_vx(x) = v
    interp_ax(x) = 0.0

    interp_vy(y) = 0.0
    interp_ay(y) = 0.0

    interp_vz(z) = 0.0
    interp_az(z) = 0.0
    
    interp_yawspeed(yaw) = 0.0

    szeros = @SVector zeros(3)


    target_state(t) = RS.flat_state_to_quad_state(
                                                  SA[interp_x(t), interp_y(t), interp_z(t)],
                                                  SA[interp_vx(t), interp_vy(t), interp_vz(t)],
                                                  SA[interp_ax(t), interp_ay(t), interp_az(t)],
                                                  szeros, # jerk
                                                  szeros, # snap
                                                  interp_yaw(t),
                                                  interp_yawspeed(t),
                                                  0.0 # yaw accel
                                                 )
    return target_state


end


function closed_loop_tracking_nominal!(D, state, params, time)

    interpolant_fn = params
    
    target_state = interpolant_fn(time)

    fM = RS.geometric_controller(state, target_state..., cont_params)

    control = RS.fM_to_ω(fM, cont_params)

    RS.quadrotor!(D, state, quad_params, control, time)


end

function closed_loop_backup_stop!(D, state, target_state, time)

    fM = RS.geometric_controller(state, target_state..., cont_params)

    control = RS.fM_to_ω(fM, cont_params)

    RS.quadrotor!(D, state, quad_params, control, time)

end


"""


"""
function construct_candidate_stop(t0, x0, traj_nominal, Ts, Tb)


    # convert traj_nominal into trajectory function to track
    interpolant_fn = construct_interpolants(traj_nominal)
    # @time interpolant_fn = test_interpolants(traj_nominal)



    ## first simulate tracking nominal for Ts seconds
    quad_ic1 = ComponentArray(
                              x = zeros(3),
                              v = zeros(3),
                              R = 1.0(I(3)) |> collect,
                              Ω = zeros(3),
                              ω = RS.hover_ω(quad_params)
                             )# TODO(dev): update with the correct initial condition!!
    tspan1 = (t0, t0 + Ts)
    params1 = interpolant_fn
    prob1 = ODEProblem(closed_loop_tracking_nominal!, quad_ic1, tspan1, params1)
    sol1 = solve(prob1, Tsit5(), abstol = 1e-5, reltol=1e-2, dense=false)
    

    ## get the solution at Ts
    quad_ic2 = sol1(t0 + Ts)

    # construct the desired stopping state
    stop_pos = quad_ic2.x
    stop_yaw = RS.yaw(quad_ic2.R)
    szeros = @SVector zeros(3)
    stop_target = RS.flat_state_to_quad_state(
                                               SVector{3}(stop_pos),
                                               szeros, # vel
                                               szeros, # acc
                                               szeros, # jerk
                                               szeros, # snap
                                               stop_yaw, # yaw
                                               0.0, # yaw speed
                                               0.0  # yaw accel
                                              )
    params2 = stop_target


    ## create the backup trajectory
    tspan2 = (t0 + Ts, t0 + Ts + Tb)
    prob2 = ODEProblem(closed_loop_backup_stop!, quad_ic2, tspan2, params2)
    sol2 = solve(prob2, Tsit5(), abstol = 1e-4, reltol=1e-2, dense=false)

    return (sol1, sol2)

end


"""


"""
function construct_candidate_mpc(t0, x0, traj_nominal, Ts, sfc)

end


"""
  is_valid(traj_candidate, sfc)

returns true if the candidate trajectory is valid based on the provided safe flight corridor

"""
function is_valid(sol1, sol2, sfc)

    # println("testing solution with time: $(sol1.t[1]) :: $(sol2.t[end])")
    
    t0 = sol1.t[1]
    t1 = sol2.t[1]
    t2 = sol2.t[end] 

    # check if the x exceeds 1.5 m
    for (i, state) in enumerate(sol1.u)
        if sol1.t[i] < t1
            if state.x[1] > 1.5
                return false
            end
        end
    end
    
    for state in sol2.u
        if state.x[1] > 1.5
            return false
        end
    end




    return true 
    


end


function construct_main_branch(t0, x0, traj_nominal)
    # convert traj_nominal into trajectory function to track
    interpolant_fn = construct_interpolants(traj_nominal)
    # @time interpolant_fn = test_interpolants(traj_nominal)



    ## first simulate tracking nominal for Ts seconds
    quad_ic1 = ComponentArray(
                              x = zeros(3),
                              v = zeros(3),
                              R = 1.0(I(3)) |> collect,
                              Ω = zeros(3),
                              ω = RS.hover_ω(quad_params)
                             )# TODO(dev): update with the correct initial condition!!
    tspan1 = (traj_nominal.t0, traj_nominal.t0 + traj_nominal.dt * (length(traj_nominal.xs) -1 ))
    
    prob1 = ODEProblem(closed_loop_tracking_nominal!, quad_ic1, tspan1, interpolant_fn)
    sol1 = solve(prob1, Tsit5(), abstol = 1e-5, reltol=1e-2, dense=false)

    return sol1


end



"""

"""
function gatekeeper(t0, x0, traj_nominal, traj_committed, sfc; Tb = 3.0)

    # x0 is expected to be in format of quadrotor state: (x, v, R, \Omega, \omega)
  

    # first construct the main branch of the solution
    sol_main = construct_main_branch(t0, x0, traj_nominal)


    # now choose a bunch of Ts
    tf = traj_nominal.dt * (length(traj_nominal.xs) - 1)
    candidate_Ts = range(start=tf, stop=0.0, length=10)

    function prob_func(prob, i, repeat)
        Ts = candidate_Ts[i]
        ic = sol_main(t0 + Ts) 
        stop_pos = ic.x
        stop_yaw = RS.yaw(ic.R)
        szeros = @SVector zeros(3)
        stop_target = RS.flat_state_to_quad_state(
                                               SVector{3}(stop_pos),
                                               szeros, # vel
                                               szeros, # acc
                                               szeros, # jerk
                                               szeros, # snap
                                               stop_yaw, # yaw
                                               0.0, # yaw speed
                                               0.0  # yaw accel
                                              )
        remake(prob, u0 = ic, p = stop_target, tspan = (Ts, Ts + Tb) )
    end

    function reduction(sols, new_sols, I)

        # check validity - if it is valid, exit!
        for sol_branch in new_sols
            append!(sols, sol_branch)
            if is_valid(sol_main, sol_branch, sfc)
                return sols, true
            end
        end
        
        return sols, false
    
    end

    # now construct the branch problems
    prob_branch = ODEProblem(closed_loop_backup_stop!, x0, (0.0, Tb), x0) 

    # prob_ensemble = EnsembleProblem(prob_branch, prob_func = prob_func)
    prob_ensemble = EnsembleProblem(prob_branch, prob_func = prob_func, reduction = reduction)

    sim = solve(prob_ensemble, Tsit5(), EnsembleThreads(), trajectories = length(candidate_Ts), batch_size = 3)


    if is_valid(sol_main, sim[end], sfc)

      return true, sol_main, sim
    end
    
    return false, sol_main, sim

end






    
    # tf = traj_nominal.dt * (length(traj_nominal.xs) - 1)

    # candidate_Ts = range(start=tf, stop=0.0, length=5)

    # for Ts in candidate_Ts

    #     println("**SOLVING WITH $(Ts)**")
    # 
    #     sol1, sol2 = construct_candidate_stop(t0, x0, traj_nominal, Ts, Tb)

    #     if is_valid(sol1, sol2, sfc)

    #         # need to convert to traj_committed
    #         return true
    #     end

    # end

    # return false

# end

end


import .Gatekeeper
GK = Gatekeeper
using Plots
using RoboticSystems
RS = RoboticSystems


function animate_sol(Ts, Tb, sol1, sol2, xs, ys, zs)

    anim = @animate for tm = range(sol1.t[1], sol2.t[end], 200)

        if tm < sol1.t[end]

            plot()
            RS.plot_quad_traj!(sol1, GK.quad_params; tspan=(sol1.t[1], tm))

        else
            plot()
            RS.plot_quad_traj!(sol1, GK.quad_params; tspan=(sol1.t[1], sol2.t[2]))
            RS.plot_quad_traj!(sol2, GK.quad_params; tspan=(sol2.t[1], tm))

        end

        plot!(xs, ys, zs, label="target")
            plot!(camera = (360 * ((tm - sol1.t[1]) / (sol2.t[end] - sol1.t[1])), 30))
            plot!( xlabel="x", ylabel="y", zlabel="z")
            RS.plot_iso3d!()
            RS.plot_project3d!()
        end

end




function test_stuff()

    t0 = 0.0;
    Ts = 2.0;
    Tb = 3.0;

    x0 = [0.0] # TODO: fix

    ## define a traj_nominal
    # struct NominalTrajectory
    #     t0
    #     dt
    #     xs
    #     ys
    #     zs
    #     yaws 
    # end
  
    dt = 0.2
    N = ceil(Int, Ts/dt)

    xs = [1.0 * i * dt for i = 0:N]
    ys = [0.0 for x in xs]
    zs = [1.0 for x in xs]
    yaws = [0.0 for x in xs]

    traj_nominal = GK.NominalTrajectory(
                                     0.0,
                                     dt,
                                     xs, ys, zs, yaws)

    traj_committed = GK.NominalTrajectory(
                                     0.0,
                                     dt,
                                     xs, ys, zs, yaws)


    # precompile
    GK.construct_candidate_stop(t0, x0, traj_nominal, Ts, Tb)


    println("hi")

    sfc = 0

    @time  GK.gatekeeper(t0, x0, traj_nominal, traj_committed, sfc)
    @time res, sol_main, sols_branch = GK.gatekeeper(t0, x0, traj_nominal, traj_committed, sfc)



    # plot solutions
    plot()
    RS.plot_quad_traj!(sol_main, GK.quad_params)

    for sol_branch in sols_branch
      RS.plot_quad_traj!(sol_branch, GK.quad_params)
    end

    RS.plot_iso3d!()
            RS.plot_project3d!()





    # plot solution
    #  tspan1 = (t0, t0 + Ts)
    #  plot()
    #  RS.plot_quad_traj!(sol1, GK.quad_params; tspan=tspan1)


    #  tspan2 = (t0 + Ts, t0 + Ts + Tb)

    #  RS.plot_quad_traj!(sol2, GK.quad_params; tspan=tspan1)


    #  RS.plot_iso3d!()


    #  anim = animate_sol(Ts, Tb, sol1, sol2, xs, ys, zs)
    #  gif(anim)


end
