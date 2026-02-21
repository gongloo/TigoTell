from pathlib import Path

Import("env", "projenv")  # type: ignore

platform = env.PioPlatform() # type: ignore
FRAMEWORK_DIR = Path(platform.get_package_dir("framework-arduinoespressif32"))
TOOLCHAIN_DIR = Path(platform.get_package_dir("toolchain-xtensa-esp-elf"))
FRAMEWORK_LIBS_DIR = Path(platform.get_package_dir("framework-arduinoespressif32-libs"))
framework_includes = list()
filtered_cpppath = list()
# apply these changes to current working env, the project env and the global env
for e in (env, projenv, DefaultEnvironment()):  # type: ignore
    for p in e["CPPPATH"]:
        # is the current include path inside the framework directory?
        p_parents = Path(p).parents
        if FRAMEWORK_DIR in p_parents or TOOLCHAIN_DIR in p_parents or FRAMEWORK_LIBS_DIR in p_parents:
            framework_includes.append(p)
        else:
            filtered_cpppath.append(p)
    e.Replace(CPPPATH=filtered_cpppath)
    e.Append(CCFLAGS=[("-isystem", p) for p in framework_includes])
