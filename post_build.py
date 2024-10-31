import os
from os.path import join
from SCons.Script import DefaultEnvironment

env = DefaultEnvironment()

def after_build(source, target, env):
    elf_file = str(target[0])
    hex_file = elf_file.replace(".elf", ".hex")
    
    command = f"arm-none-eabi-objcopy -O ihex {elf_file} {hex_file}"  # Make sure to use the correct objcopy command for your platform
    print(f"Running: {command}")
    os.system(command)

# Register the post-build action
env.AddPostAction("$BUILD_DIR/${PROGNAME}.elf", after_build)
