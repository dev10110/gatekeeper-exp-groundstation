module Gatekeeper

using LinearAlgebra
using ComponentArrays
using StaticArrays
using Interpolations
using ForwardDiff
using Parameters


using DifferentialEquations

using RoboticSystems
RS = RoboticSystems

const quad_params = ComponentArray(
  RS.quadrotor_parameters;
  mass=0.8,
  J = diagm([0.0025, 0.0025, 0.004]) |> collect, 
  k_f = 1.37e-6, 
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


struct NominalTrajectory{F, VF<:AbstractVector{F}}
    t0::F
    dt::F
    xs::VF
    ys::VF
    zs::VF
    yaws::VF
end

const SM3{F} = SMatrix{3,3,F,9}
const SV3{F} = SVector{3,F}

function vee(M::SM3)
    return @SVector [
        (M[3, 2] - M[2, 3]) / 2
        (M[1, 3] - M[3, 1]) / 2
        (M[2, 1] - M[1, 2]) / 2
    ]
end

function hat(v::SV3{F}) where {F}
    return @SMatrix [
        [zero(F);; -v[3];; v[2]]
        [v[3];; zero(F);; -v[1]]
        [-v[2];; v[1];; zero(F)]
    ]
end


function geometric_controller(state, xd, vd, ad, b1d, Ωd, αd, controller_params)

    x = SVector{3}(state.x)
    v = SVector{3}(state.v)
    R = SMatrix{3,3, Float64, 9}(state.R)
    Ω = SVector{3}(state.Ω)

    @unpack kx, kv, kR, kΩ, m, g = controller_params
    J = SM3(controller_params.J)

    e3 = SA[0, 0, 1.0]

    ex = x - SVector{3}(xd)
    ev = v - SVector{3}(vd)

    # construct desired rotation matrix
    b3d = (-kx * ex - kv * ev + m * g * e3 + m * ad) |> normalize
    b2d = cross(b3d, normalize(SVector{3}(b1d))) |> normalize
    b1d_n = cross(b2d, b3d) |> normalize

    Rd = [b1d_n b2d b3d]

    eR = 0.5 * vee(Rd' * R - R' * Rd)
    eΩ = Ω - R' * Rd * Ωd


    f = dot(-kx * ex - kv * ev + m * g * e3 + m * ad, R * e3)
    M = -kR * eR - kΩ * eΩ + cross(Ω, (J * Ω)) - J * (hat(Ω) * R' * Rd * Ωd - R' * Rd * αd)

    return SA[f, M[1], M[2], M[3]]

end

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

    interpolant_fn, quad_p, cont_p = params
    
    target_state = interpolant_fn(time)

    fM = geometric_controller(state, target_state..., cont_p)

    control = RS.fM_to_ω(fM, cont_p)

    RS.quadrotor!(D, state, quad_p, control, time)


end

function closed_loop_backup_stop!(D, state, params, time)

    target_state, quad_p, cont_p = params

    fM = geometric_controller(state, target_state..., cont_p)

    control = RS.fM_to_ω(fM, cont_p)

    RS.quadrotor!(D, state, quad_p, control, time)

end


"""


"""
function construct_candidate_stop(t0, x0, traj_nominal, Ts, Tb; quad_params =quad_params, cont_params=cont_params)


    # convert traj_nominal into trajectory function to track
    interpolant_fn = construct_interpolants(traj_nominal)
    # @time interpolant_fn = test_interpolants(traj_nominal)



    ## first simulate tracking nominal for Ts seconds
    tspan1 = (t0, t0 + Ts)
    params1 = (interpolant_fn, quad_params, cont_params)
    prob1 = ODEProblem(closed_loop_tracking_nominal!, x0, tspan1, params1)
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


    ## create the backup trajectory
    params2 = (stop_target, quad_params, cont_params)
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
    for state in reverse(sol2.u)
        if state.x[1] > 1.5
            return false
        end
    end
    
    for i in reverse(1:length(sol1.u))
        if sol1.t[i] < t1
            if sol1.u[i].x[1] > 1.5
                return false
            end
        end
    end
    


    return true 
    


end


function construct_main_branch(t0, x0, traj_nominal; quad_params=quad_params, cont_params=cont_params)
    # convert traj_nominal into trajectory function to track
    interpolant_fn = construct_interpolants(traj_nominal)

    params = (interpolant_fn, quad_params, cont_params)

    ## first simulate tracking nominal for Ts seconds
    tspan1 = (traj_nominal.t0, traj_nominal.t0 + traj_nominal.dt * (length(traj_nominal.xs) -1 ))
    
    prob1 = ODEProblem(closed_loop_tracking_nominal!, x0, tspan1, params) 
    sol1 = solve(prob1, Tsit5(), abstol = 1e-4, reltol=1e-2, dense = false)

    return sol1


end



"""

"""
function gatekeeper(t0, x0, traj_nominal, traj_committed, sfc; Tb = 3.0, quad_params=quad_params, cont_params=cont_params)

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
        remake(prob, u0 = ic, p = (stop_target, quad_params, cont_params), tspan = (Ts, Ts + Tb) )
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
    params_branch = (x0, quad_params, cont_params) 
    prob_branch = ODEProblem(closed_loop_backup_stop!, x0, (0.0, Tb), params_branch) 

    # prob_ensemble = EnsembleProblem(prob_branch, prob_func = prob_func)
    prob_ensemble = EnsembleProblem(prob_branch, prob_func = prob_func, reduction = reduction)

    sim = solve(prob_ensemble, Tsit5(), EnsembleThreads(), trajectories = length(candidate_Ts), batch_size = 1)


    if is_valid(sol_main, sim[end], sfc)

      return true, sol_main, sim
    end
    
    return false, sol_main, sim

end


end


