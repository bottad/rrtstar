# RRT* Implementation

This is a simple implementation of the RRT* algorithm. As it does not use any optimized tree structure it is pretty slow.
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

Define your map configuration in a config.yaml file in the config folder and choose your config file in the main file code, then run the main file.

```
python src/main.py
```