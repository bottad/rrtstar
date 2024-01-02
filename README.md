# RRT* Implementation

This are a few simple implementations of the RRT* algorithm.
- Starting the tree from the start trying to find a path to the goal (rrtstar.py)
- Starting the tree from the goal, to find paths from any start location (reverse_rrtstar)
- Using Scipy cKDtrees to speed up the nearest neighbor search (_kd)

Pygame is used for visualization purposes.

## Installation

Create a virtual environnement for python and install the dependencies listet in the requirements.txt file.

### Commands for cmd:

```
python -m venv .venv

.venv\Scripts\activate.bat

pip install -r requirements.txt
```

## Use

Define your map configuration in a config.yaml file in the config folder and choose your config file in the main file code. To deside which rrt version should be run, start the main program with eigther one or both of the following flags:
- kd -> using kd-tree datastructure
- r  -> reverse mode

The order of the Flags does not matter.

### Example:

```
python src/main.py kd r
```
