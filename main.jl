using SFML
using Reactive
include("robot.jl")

# redirect standard output to a terminal of choice
println("Enter redirect device (type n for no redirect): ")
tty_name = STDIN |> readline |> chomp
if tty_name != "n"
    tty = open(tty_name, "w")
    redirect_stdout(tty)
    redirect_stderr(tty)

    # flush every now and then
    @async begin
        while true
            sleep(0.2)
            flush(tty)
        end
    end
end

window = RenderWindow("Robot Simulation", 800, 600)
set_framerate_limit(window, 60)
event = Event()

# simulation object for the robot
robotSim = Robot(; d = 0.2, I = 0.05, prop = 2.0, vmax = 1., maxforce = 1.0)

# shape for the robot
robotRect = RectangleShape()
set_size(robotRect, Vector2(0.2, 0.3))
set_origin(robotRect, Vector2(0.1, 0.15))
set_position(robotRect, Vector2(200., 400.))
set_fillcolor(robotRect, SFML.cyan)

# left and right targets
sl = 0.
sr = 0.

# reset the state of the robot
function reset()
    global sl, sr
    sl = 0.; sr = 0.;
    robotSim.state = RobotState((0., 0.), (0., 0.), 0., 0.)
end

# generate the parrot distance signal, with some noise
noiseFactor = 0.1
function parrotSignal(dt)

    parrotPosView = pixel2coords(window, get_mousepos(window))
    parrotPos = (Float64(parrotPosView.x), -Float64(parrotPosView.y))
    distRaw = norm(robotSim.state.x - parrotPos)
    distRaw += noiseFactor * randn()

    return distRaw
end

# robot's reaction to the parrot signal
function parrotSignalResponse(distanceNoisy)
    
    println("distance measured: ", distanceNoisy)
end

# create the pipeline
_ = map(parrotSignal, fps(2))
_ = map(parrotSignalResponse, _)

robotSim.wheelSigFunc = dt -> (sl, sr)

view = View(Vector2f(0, 0), Vector2f(8, 6))

function main_loop(dt)

    global sl, sr

    # poll events and close window if necessary
    event = Event()
    while pollevent(window, event)

        if get_type(event) == EventType.CLOSED
            close(window)
        end

        if get_type(event) == EventType.KEY_PRESSED

            if get_key(event).key_code == KeyCode.A
                sl -= 0.05
                sr += 0.05
            elseif get_key(event).key_code == KeyCode.D
                sl += 0.05
                sr -= 0.05
            elseif get_key(event).key_code == KeyCode.W
                sl += 0.05
                sr += 0.05
            elseif get_key(event).key_code == KeyCode.S
                sl -= 0.05
                sr -= 0.05
            elseif get_key(event).key_code == KeyCode.SPACE
                sl = 0.
                sr = 0.
            end
        end
    end

    # tick the simulation
    tick(robotSim, dt)

    # update the position of the robot rectangle according to sim data
    set_position(robotRect, Vector2f(robotSim.state.x[1], -robotSim.state.x[2]))
    set_rotation(robotRect, -robotSim.state.θ * 360 / (2π))
    
    # might use this as a camera sort of thing
    set_view(window, view)
    
    # render the robot
    clear(window, SFML.black)
    draw(window, robotRect)
    display(window)
end

# run asynchronously
ticks = fps(60)
main_loop_sig = map(main_loop, ticks)
