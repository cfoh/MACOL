[[Introduction]](#introduction) &nbsp;&nbsp;&nbsp; 
[[Numerical Validation]](#numerical-validation) &nbsp;&nbsp;&nbsp;
[[Simulation Code]](#simulation-code) &nbsp;&nbsp;&nbsp;
[[Simple Toy Example]](#simple-toy-example)

---

# Introduction

In this repo, we provide the numarical computation of beam analysis we presented in our paper:
- [![PDF](https://github.com/cfoh/MACOL/blob/main/PDF_file_icon.svg)](https://arxiv.org/abs/2401.02323)
  A. Kose, H. Lee, C. H. Foh, M. Shojafar, "Multi-Agent Context Learning Strategy for Interference-Aware Beam Allocation in
  mmWave Vehicular Communications," vol. 25, no. 7, pp. 7477-7493, July 2024, doi: 10.1109/TITS.2024.3351488.
  [[arXiv]](https://arxiv.org/abs/2401.02323)

## Citation
```
@ARTICLE{10433881,
  author={Kose, Abdulkadir and Lee, Haeyoung and Foh, Chuan Heng and Shojafar, Mohammad},
  journal={IEEE Transactions on Intelligent Transportation Systems}, 
  title={Multi-Agent Context Learning Strategy for Interference-Aware Beam Allocation in mmWave Vehicular Communications}, 
  year={2024},
  volume={25},
  number={7},
  pages={7477-7493},
  keywords={Interference;Millimeter wave communication;Array signal processing;Switches;Vehicle-to-everything;Throughput;Receivers;Vehicular networks;mmWave;beam management;machine learning;multi-armed bandit},
  doi={10.1109/TITS.2024.3351488}}
```
 
# Numerical Validation

We share our numerical computation unit test source code runnable online:
[Beam Analysis.ipynb](https://colab.research.google.com/drive/1Coy1dabcAHLq0Dn0PEpHJJAWnmnauhWd?usp=sharing)
on [![Google Colab](https://img.shields.io/badge/Google-Colab-yellow)](https://colab.research.google.com/drive/1Coy1dabcAHLq0Dn0PEpHJJAWnmnauhWd?usp=sharing)

# Simulation Code

<img src="https://github.com/cfoh/beam-analysis/blob/main/sim-demo.gif" width="300">

The user simulation code is also available in this repo [[click here to see the code]](https://github.com/cfoh/MACOL/blob/main/test-macol.py).

The simulation code requires 
[![Pymosim v0.8.8](https://img.shields.io/badge/Pymosim-v0.8.8-brightgreen)](https://cfoh.github.io/pymosim-doc/start.html) 
platform to run. We plan to open the source of Pymosim Platform soon.
Meanwhile, you can inspect the simulation code and understand the simulation setup.

# Simple Toy Example

You can find a simple toy example implementing MACOL [here](https://github.com/cfoh/MACOL/tree/main/toy-example). 
The toy example is self-contained and only requires pygame package to run.

The following is a comparison between greedy (top) and MACOL (bottom) approaches. As can be seen, MACOL reduces the aggressiveness of the transmissions, leading to less intensive interference and more successful transmissions.

<img src="https://github.com/cfoh/MACOL/blob/main/toy-example/demo.gif" width="300">
