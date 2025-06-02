# MSTC-Star
> This project builds upon the codebase from the ICRA'21 paper:
MSTC* – Multi-Robot Coverage Path Planning under Physical Constraints. [[paper]](https://arxiv.org/abs/2108.04632), [[video]](https://vimeo.com/535512748).

## Description
We have extended the original implementation to explore and evaluate TMSTC*, a turn-aware variant of MSTC*, designed to further minimize coverage cost by explicitly considering turning penalties in addition to travel distances.

## Overview
This repository includes:
- Original MSTC* and Balanced-MSTC* implementations.
- TMSTC*: Our extension that incorporates turning costs into the robot path planning logic.

## Requirments
- python 3.6 +
- matplotlib
- numpy
- scipy
- skimage
- pytorch (tested on 1.7.0)
- opencv-python (tested on v.4.4.0.46)
- rasterio (tested on v.1.1.8)
- networkx (tested on v.2.5)
  
## Getting Started
- 1) Clone this repository: ```bash git clone https://github.com/a-runebou/MSTC_Star.git```
  2) Run `main.py`
 
## Acknowledgements
We thank Jingtao Tang for making the original MSTC* code publicly available. Our work builds directly upon this foundation.
