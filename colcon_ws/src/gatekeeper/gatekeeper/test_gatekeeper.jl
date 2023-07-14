include("gatekeeper-julia.jl")
import .Gatekeeper
GK = Gatekeeper
using Plots
using RoboticSystems
RS = RoboticSystems
using LinearAlgebra
using ComponentArrays


# function animate_sol(Ts, Tb, sol1, sol2, xs, ys, zs)
# 
#     anim = @animate for tm = range(sol1.t[1], sol2.t[end], 200)
# 
#         if tm < sol1.t[end]
# 
#             plot()
#             RS.plot_quad_traj!(sol1, GK.quad_params; tspan=(sol1.t[1], tm))
# 
#         else
#             plot()
#             RS.plot_quad_traj!(sol1, GK.quad_params; tspan=(sol1.t[1], sol2.t[2]))
#             RS.plot_quad_traj!(sol2, GK.quad_params; tspan=(sol2.t[1], tm))
# 
#         end
# 
#         plot!(xs, ys, zs, label="target")
#             plot!(camera = (360 * ((tm - sol1.t[1]) / (sol2.t[end] - sol1.t[1])), 30))
#             plot!( xlabel="x", ylabel="y", zlabel="z")
#             RS.plot_iso3d!()
#             RS.plot_project3d!()
#         end
# 
# end




function test_stuff()

  t0 = 0.5
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

  # precompile
  # GK.construct_candidate_stop(t0, x0, traj_nominal, Ts, Tb)


  println("hi")

  sfc_A = [[1;; 0 ;; 0.0];]
  sfc_b = [1.5];
  sfcs = [GK.SFC(sfc_A, sfc_b)]

  res, sol_main, sols_branch = GK.gatekeeper(t0, x0, traj_nominal, sfcs)
  # @time GK.gatekeeper(t0, x0, traj_nominal, sfcs)


  println(res)



  # # plot solutions
  # plot()
  # RS.plot_quad_traj!(sol_main, GK.quad_params)

  # for sol_branch in sols_branch
  #   RS.plot_quad_traj!(sol_branch, GK.quad_params)
  # end

  # RS.plot_iso3d!()
  # RS.plot_project3d!()





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


# using BenchmarkTools

function test_geo()

  x0 = ComponentArray(
    x = zeros(3),
    v = zeros(3),
    R = 1.0(I(3)) |> collect,
    Ω = zeros(3),
    ω = RS.hover_ω(GK.quad_params),
  )

  xd = ones(3)
  vd = zeros(3)
  ad = zeros(3)
  b1d = [1, 0, 0.0]
  Ωd = zeros(3)
  αd = zeros(3)

  @time res = GK.geometric_controller(x0, xd, vd, ad, b1d, Ωd, αd, GK.cont_params)
  # @time res = GK.geometric_controller(x0, xd, vd, ad, b1d, Ωd, αd, GK.cont_params)
  # @time res = RS.geometric_controller(x0, xd, vd, ad, b1d, Ωd, αd, GK.cont_params)

  @time control = RS.fM_to_ω(res, GK.cont_params)

  D = similar(x0)

  RS.quadrotor!(D, x0, GK.quad_params, control, 0.0)
  @time RS.quadrotor!(D, x0, GK.quad_params, control, 0.0)


end
