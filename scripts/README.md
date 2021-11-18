# Available Scripts

### Run Script

Starts CARLA and the main launcher.

```
bash scripts/runscript.sh [--build] [--skip-carla-restart]
```

### Build Script

Builds the whole project (optional rosdep update and clean build)

```
bash scripts/build_ros.sh [--rosdep] [--clean] [--no-build]
```

### Install Script

Installs CARLA, ROS and project dependencies.

```
bash scripts/setup.sh
```

To install different parts separately, run one of the following commands:

```
bash scripts/subscripts/_setup_carla.sh   # carla install
bash scripts/subscripts/_setup_ros.sh     # ros install
bash scripts/subscripts/_setup_paf.sh     # dependency install
```

### Pre-Commit

For commiting, pre-commit is installed. Python, C++ and other Code is formatted and verified automatically before each
commit and the project is build. The following commands are available.

```
# install pre-commit with the install script
bash scripts/subscripts/_setup_paf.sh

# optional run over all files
pre-commit run --all-files
# or
pre-commit run -a

# commit without pre-commit checks
git commit --no-verify -m "commit message"
# or
git commit -n -m "commit message"
```

Committing always fails on automatically modified files, therefore try again if this happens.
