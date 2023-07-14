using Pkg;
Pkg.add("PackageCompiler")
Pkg.add("DifferentialEquations")


pkgs = [
        "DifferentialEquations",
       ]


using PackageCompiler
create_sysimage(pkgs, sysimage_path="/root/JuliaSysImage.so")
