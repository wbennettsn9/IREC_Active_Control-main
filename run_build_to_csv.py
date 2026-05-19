Import("env")
import os;

def post_program_action(source, target, env):
    print("Program has been built!")
    program_path = target[0].get_abspath()
    print("Running program from ", program_path)
    os.system(f"{program_path} > ./sitl/sim.csv")

    

env.AddPostAction("$PROGPATH", post_program_action)

#
# Upload actions
#