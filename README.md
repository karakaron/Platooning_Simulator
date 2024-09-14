# Platooning Simulator

This project is a small driving simulator built on top of Carla. It aims to provide a simple way to validate control strategies of autonomous vehicle platoons in highly detailed simulations. It has been developed by Aron Karakai at the University of Groningen (Groningen, The Netherlands), supervised by Prof. dr. ir. Bart Besselink (Bernoulli Institute for Mathematics, Computer Science and Artificial Intelligence, Groningen, The Netherlands).

The accompanying research paper is available [here](http://dx.doi.org/10.13140/RG.2.2.31833.08802/1).

For questions, suggestions, or feature requests, please send an email to akarakai [at] ethz.ch.

## Getting Started

First, you need to set up Carla, following https://carla.readthedocs.io/en/latest/start_quickstart/. On Windows, it may be necessary to add "[carla_root]/PythonAPI/carla", where "[carla_root]" is your Carla root directory, to the PYTHONPATH environment variable.

Second, you need to either install the present Platooning Simulator with
````terminal
pip install PlatooningSimulator
````
or clone this repository and use the code directly. The former also installs numpy and the Carla Python API in case they are not already installed.

With that, the modules can be imported as
````Python
from PlatooningSimulator import Core, PlatooningControllers
````

See the examples library on how to run a simulation, and see the [Documentation](https://github.com/karakaron/Platooning_Simulator/wiki/Documentation) for more information.
