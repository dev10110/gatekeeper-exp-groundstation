__precompile__()

module Gatekeeper

println("gatekeeper imports")
@time using LinearAlgebra
@time using ComponentArrays
@time using StaticArrays
@time using Interpolations
@time using ForwardDiff
@time using Parameters


@time using Plots

@time using DifferentialEquations

@time using RoboticSystems
RS = RoboticSystems

println("imports done")

const quad_params =
  ComponentArray(
    RS.quadrotor_parameters;
    mass = 0.8,
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
  g = 9.81,
)


struct NominalTrajectory{F,VF<:AbstractVector{F}}
  t0::F # time in seconds 
  dt::F
  xs::VF
  ys::VF
  zs::VF
  yaws::VF
end


struct SFC{F, VF<:AbstractVector{F}, MF<:AbstractMatrix{F}}
    A::MF
    b::VF
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
  R = SMatrix{3,3,Float64,9}(state.R)
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


  ts = range(; start = traj.t0, step = traj.dt, length = length(traj.xs))

  interp_x = cubic_spline_interpolation(ts, traj.xs, extrapolation_bc=Flat())
  interp_y = cubic_spline_interpolation(ts, traj.ys, extrapolation_bc=Flat())
  interp_z = cubic_spline_interpolation(ts, traj.zs, extrapolation_bc=Flat())
  interp_yaw = cubic_spline_interpolation(ts, traj.yaws, extrapolation_bc=Flat())

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
    0.0, # yaw accel
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
  is_valid(traj_candidate, sfc)

returns true if the candidate trajectory is valid based on the provided safe flight corridor
"""
function is_valid(sol1, sol2, sfcs)

  # println("testing solution with time: $(sol1.t[1]) :: $(sol2.t[end])")

  t0 = sol1.t[1]
  t1 = sol2.t[1]
  t2 = sol2.t[end]

  # check that the branch comes to a stop
  if !is_stopped(sol2.u[end])
      return false
  end

  # check that branch lies in the sfc
  for state in reverse(sol2.u)
      if !state_in_sfcs(state.x, sfcs)
          return false
      end
  end

  # check that main lies in the sfc
  for i in reverse(1:length(sol1.u))
    if sol1.t[i] < t1 # only check those before the branching
        if !state_in_sfcs(sol1.u[i].x, sfcs)
        return false
      end
    end
  end

  return true

end



"""
check if it lies in any one of an array of sfcs
"""
function state_in_sfcs(state, sfcs)
    for (i, sfc)  in enumerate(sfcs)
        if all(sfc.A * state .<= sfc.b)
            return true
        end
    end
    return false
end

function is_stopped(state; eps= 0.05)
    return norm(state.v) <= eps # cm/s
end







function construct_main_branch(
  t0, # refers to the timestamp of x0
  x0,
  traj_nominal;
  Ts_max = 20.0, # max duration into the future to forecast
  quad_params = quad_params,
  cont_params = cont_params,
)
  # convert traj_nominal into trajectory function to track
  interpolant_fn = construct_interpolants(traj_nominal)

  params = (interpolant_fn, quad_params, cont_params)

  ## first simulate tracking nominal for Ts seconds
  tf_traj = traj_nominal.t0 + traj_nominal.dt * (length(traj_nominal.xs) - 1)
  @show tf_traj

  tf = min(t0 + Ts_max, tf_traj)
  
  @show tf
  tspan =(t0,  tf)

  @show tspan

  prob = ODEProblem(closed_loop_tracking_nominal!, x0, tspan, params)
  sol = solve(prob, Tsit5(), abstol = 1e-6, reltol = 1e-3)

  return sol


end

"""

"""
function gatekeeper(
  t0,
  x0,
  traj_nominal,
  sfcs;
  Tb = 3.0,
  quad_params = quad_params,
  cont_params = cont_params,
)

  # x0 is expected to be in format of quadrotor state: (x, v, R, \Omega, \omega)
  println("Running gatekeeper with t0: $(t0)")

  # first construct the main branch of the solution
  sol_main = construct_main_branch(t0, x0, traj_nominal)


  # return;

  # now choose a bunch of Ts
  candidate_Ts = range(start = sol_main.t[end] , stop = sol_main.t[1], length = 11)

  println("candidate Ts: $(candidate_Ts)")

  function prob_func(prob, i, repeat)
    Ts = candidate_Ts[i]
    ic = sol_main(Ts)
    stop_pos = SVector{3}(ic.x)
    stop_yaw = RS.yaw(ic.R)
    szeros = @SVector zeros(3)
    stop_target = RS.flat_state_to_quad_state(
      stop_pos,
      szeros, # vel
      szeros, # acc
      szeros, # jerk
      szeros, # snap
      stop_yaw, # yaw
      0.0, # yaw speed
      0.0,  # yaw accel
    )
    tspan = (Ts, Ts+Tb)
    println("remaking for i=$(i), using tspan $(tspan), with target: $(stop_pos)")
    remake(
      prob,
      u0 = ic,
      p = (stop_target, quad_params, cont_params),
      tspan = tspan,
    )
  end

  function reduction(sols, new_sols, I)

      println("reduction: length(new_sols) = $(length(new_sols))")

    # check validity - if it is valid, exit!
    for sol_branch in new_sols
        println("checking reduction for sol.tspan = $(sol_branch.t[1])::$(sol_branch.t[end])")
        push!(sols, sol_branch)
        
      println("reduction: length(sols) = $(length(sols))")

      if is_valid(sol_main, sol_branch, sfcs)
          println("FOUND VALID SOL! using sol.tspan = $(sol_branch.t[1])::$(sol_branch.t[end])")
        return sols, true # allows early termination
      end
    end

    return sols, false

  end

  # now construct the branch problems
  params_branch = (x0, quad_params, cont_params)
  prob_branch = ODEProblem(closed_loop_backup_stop!, x0, (0.0, Tb), params_branch)

  # prob_ensemble = EnsembleProblem(prob_branch, prob_func = prob_func)
  prob_ensemble = EnsembleProblem(prob_branch, prob_func = prob_func, reduction = reduction, safetycopy=false)

  println("constructed all the problems, solving...")
  sim = solve(
    prob_ensemble,
    Tsit5(),
    EnsembleThreads(),
    trajectories = length(candidate_Ts),
    batch_size = 1,
  )

  println("ensemble solve is successful!i, plotting stuff")

  ## plot stuff
  plot_p1 = begin plot()
  plot!(traj_nominal.xs, traj_nominal.ys, traj_nominal.zs; label="nominalTraj")
  RS.plot_quad_traj!(sol_main, quad_params; label="main")
  for (i, sol) in enumerate(sim)
      RS.plot_quad_traj!(sol, quad_params;  label="branch $(i)")
  end
  plot!()
  RS.plot_iso3d!()
  end

  plot_p2 = begin plot()
      ts = [traj_nominal.t0 + (i-1) * traj_nominal.dt for i in 1:length(traj_nominal.xs)]
      plot!(ts, traj_nominal.xs, label="nom x", marker=:dot)
      #plot!(ts, traj_nominal.ys, label="nom y")
      # plot!(ts, traj_nominal.zs, label="nom z", marker=:dot)

      plot!(t->sol_main(t)[1], sol_main.t[1], sol_main.t[end], label="main x")
      # plot!(t->sol_main(t)[2], sol_main.t[1], sol_main.t[end], label="main y")
      # plot!(t->sol_main(t)[3], sol_main.t[1], sol_main.t[end], label="main z")
      
      for (i, sol) in enumerate(sim)
          plot!(t->sol(t)[1], sol.t[1], sol.t[end], label="branch $(i) x")
          # plot!(t->sol(t)[2], sol.t[1], sol.t[end], label= "branch $(i) y")
          # plot!(t->sol(t)[3], sol.t[1], sol.t[end], label= "branch $(i) z")
      end
      plot!(legend=false)
  end

  plot(plot_p1, plot_p2, layout=(@layout [a b]))

  gui()

  # if length(sim) > 1
  #     println("wait for keyboard")
  #     readline()
  # end

  # println("waiting for keyboard input...")
  # readline()

  # do one more check for good measure
  if is_valid(sol_main, sim[end], sfcs)
      println("Using $(sim[end].t[1] - sol_main.t[1])s of the main branch")
      return true, sol_main, sim[end]
  end

  println("gatekeeper failed...")
  return false, sol_main, sim[end]

end



## precompile stuff
using PrecompileTools

@setup_workload begin


  @compile_workload begin
  
  t0 = 0.0
  Ts = 2.0
  Tb = 3.0

  x0 = ComponentArray(
    x = zeros(3),
    v = zeros(3),
    R = 1.0(I(3)) |> collect,
    Ω = zeros(3),
    ω = RS.hover_ω(GK.quad_params),
  )

  ## define a traj_nominal

  dt = 0.2
  N = ceil(Int, Ts / dt)

  xs = [1.0 * i * dt for i = 0:N]
  ys = [0.0 for x in xs]
  zs = [1.0 for x in xs]
  yaws = [0.0 for x in xs]


  traj_nominal = GK.NominalTrajectory(0.0, dt, xs, ys, zs, yaws)

  sfc_A = [[1;; 0 ;; 0.0];]
  sfc_b = [1.5];
  sfcs = [GK.SFC(sfc_A, sfc_b)]

  res, sol_main, sol_branch = GK.gatekeeper(t0, x0, traj_nominal, sfcs)
  end

end




end
