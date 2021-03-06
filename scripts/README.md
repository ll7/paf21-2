# Available Scripts

### Run Script

Starts CARLA and the main launcher in two new terminals.
```
bash scripts/runscript.sh TownXX [--build/-b] [--skip-carla-restart/-scr] [--npcs/-n] [--low-quality/-lq] [--manual-control/-mc] [--no-rules/-nr] [--validation/-val]
```

Tensorboard kann mit `tensorboard.sh` gestartet werden.

```
Argumente:
--skip-carla-restart/-scr : die Carla Simulation wird nicht neugestartet
--build/-b		  : erneutes builden des Projekts, wenn man Änderungen im Code vorgenommen hat
--npcs/-n		  : Starten mit npcs
--low-quality/-lq	  : Starten in niedrigerer Qualität für leistungsschwächere Geräte
--manual-control/-mc	  : Starten mit Öffnen eines manual-control Fensters
--no-rules/-nr (rules enabled by default): Unterscheidung ob man mit oder ohne Regeln Fahren möchte
TownXX                    : Town auf der gefahren werden soll, Town01-07 oder Town10HD
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
bash scripts/subscripts/_setup_carla.sh      # carla install
bash scripts/subscripts/_setup_ros.sh        # ros install
bash scripts/subscripts/_setup_paf.sh        # dependency install
bash scripts/subscripts/_setup_commonroad.sh # commonroad extended install
```

To install or run the (optional) Commonroad-Designer, execute the following command (this may take more than 30 minutes the first time):

```
bash scripts/commonroad_designer.sh
```

### Pre-Commit

For commiting, pre-commit is installed. Python, C++ and other Code is formatted and verified automatically before each
commit and the project is build. Additionally, a ```cmake clean``` command is executed after each checkout automatically.

The following commands are available:

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
