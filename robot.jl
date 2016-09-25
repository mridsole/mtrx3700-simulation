# use tuple type for vector 2 (variable length array has overhead)
typealias Vec2 Tuple{Float64, Float64}
import Base.+, Base.-, Base.*, Base./, Base.norm, 
    Base.sign, Base.dot
-(l::Vec2) = (-l[1], -l[2])
+(l::Vec2, r::Vec2) = (l[1] + r[1], l[2] + r[2])
-(l::Vec2, r::Vec2) = +(l, -(r))
*(l::Vec2, r::Vec2) = (l[1] * r[1], l[2] * r[2])
/(l::Vec2, r::Vec2) = (l[1] / r[1], l[2] / r[2])
+(l::Vec2, r::Float64) = (l[1] + r, l[2] + r)
-(l::Vec2, r::Float64) = +(l, -r)
*(l::Vec2, r::Float64) = (l[1] * r, l[2] * r)
/(l::Vec2, r::Float64) = (l[1] / r, l[2] / r)
+(l::Float64, r::Vec2) = +(r, l)
-(l::Float64, r::Vec2) = +(l, -(r))
*(l::Float64, r::Vec2) = *(r, l)
/(l::Float64, r::Vec2) = (l / r[1], l / r[2])
norm(l::Vec2) = sqrt(l[1]^2 + l[2]^2)
sign(l::Vec2) = (sign(l[1]), sign(l[2]))
dot(l::Vec2, r::Vec2) = l[1] * r[1] + l[2] * r[2]

immutable RobotState
    
    # position of the robot in cartesian coords
    x::Vec2

    # velocity of the robot, in forward direction
    v::Vec2

    # angle measured counterclockwise from the x-axis
    θ::Float64

    # angle time derivative
    ω::Float64
end

type Robot

    # the state of the robot
    state::RobotState
    
    # distance from the wheel to the center
    d::Float64

    # mass of robot
    m::Float64

    # moment of inertia of robot
    I::Float64

    # constant friction force
    friction::Float64
    
    # max wheel speed
    vmax::Float64

    # porportional constant
    prop::Float64

    # max force that can be applied to wheel
    maxforce::Float64

    # function to be called that returns the left and right wheel forces
    # (acting on the robot - should return tuple (left, right))
    wheelSigFunc::Any

    function Robot(;d::Float64=0.1, m::Float64=2., I::Float64=0.02, 
        friction::Float64=0.5, vmax::Float64=0.5, prop::Float64=0.5, 
        maxforce::Float64=0.2, wsf=dt->(0., 0.))
        return new(RobotState((0., 0.), (0., 0.), 0., 0.), d, m, I, friction,
            vmax, prop, maxforce, wsf)
    end
end

t(rs::RobotState) = Vec2(-sin(θ), cos(θ))
n(rs::RobotState) = Vec2(cos(θ), sin(θ))

function tick(robot::Robot, dt::Float64)
    
    # get wheel forces
    wheelSig = robot.wheelSigFunc(dt)

    et = (-sin(robot.state.θ), cos(robot.state.θ))

    # proportional control to find out how much force to apply
    wheelForce = (
        robot.prop * (robot.vmax * wheelSig[1] - 
        (dot(et, robot.state.v) - robot.d * robot.state.ω)),
        robot.prop * (robot.vmax * wheelSig[2] - 
        (dot(et, robot.state.v) + robot.d * robot.state.ω))
    )

    #println("force: ", wheelForce)
    #println("target: ", robot.vmax * wheelSig)
    #println("actual: ", dot(et, robot.state.v) - robot.d * robot.state.ω,
    #    dot(et, robot.state.v) + robot.d * robot.state.ω)

    dt = dt / 20.
    for i = 1:20

        # compute the velocities of the wheels (either side)
        vL = norm(robot.state.v) - robot.d * robot.state.ω
        vR = norm(robot.state.v) + robot.d * robot.state.ω
        
        # need to apply friction at the wheels
        FL = 0. #robot.friction * vL
        FR = 0. #robot.friction * vL
    
        # compute new ω and θ
        ω = robot.state.ω + dt * robot.d * (wheelForce[2] - FR -
            (wheelForce[1] - FL)) / robot.I
        θ = (robot.state.θ + dt * ω + 2π) % 2π

        # compute new normal and tangent
        et = (-sin(robot.state.θ), cos(robot.state.θ))
        en = (cos(robot.state.θ), sin(robot.state.θ))

        # compute new v and x - include normal force!
        v = 0.9999 * robot.state.v + dt * (et * (wheelForce[1] - FL +
            wheelForce[2] - FR) / robot.m - en * dot(et, robot.state.v) * 
            robot.state.ω)
        x = robot.state.x + dt * v

        # update the state
        robot.state = RobotState(x, v, θ, ω)
    end

    nothing
end
