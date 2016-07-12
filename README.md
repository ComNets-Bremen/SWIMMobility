# SWIMMobility

SWIMMobility is an implementation of the Small Worlds in Motion (SWIM) [[1](https://arxiv.org/pdf/0809.2730.pdf)] mobility model in the OMNeT++ Simulator [[2](https://omnetpp.org)]. This implementation work has been done as part of the student projects at `Sustainable Communication Networks Group, University of Bremen, Germany`.

Introduction
============

SWIM is a mobility model that mathematically characterises the movement patterns of humans. It uses 2 intuitions of human mobility, viz., people usually visit mostly locations close to their home location and if they visit a location far away from home, it is due to its popularity. 


The mathematical formulations of these intuitions is based on an equation that considers the distance to a location and the popularity of that location. This code is an implementation of the SWIM mobility model in OMNeT++ Simulator for the INET framework [[3](https://inet.omnetpp.org)].


Mobility Models in OMNeT++
==========================

OMNeT++ provides an interface (i.e., Abstract Class) that any mobility mechanism must implement to enable mobility in a node. This interface, called IMobility provides a set of methods that is invoked from other models that require mobility related information. An example is the wireless propagation models which require to know current coordinates, speed of mobility, etc. to determine the connectivity to other nodes.

To simplify the implementation of mobility, a set of base implementations are provided in OMNeT++ that implements some of the abstract methods of IMobility. These base implementations focus on implementing the basic functionality required in mobility patterns that could be generalised into categories.

This SWIM implementation extends the LineSegmentsMobilityBase class.


Compiling the SWIM Mobility Model
=================================

Place the following 3 files in the mobility model folder of the INET framework and rebuild.

- `SWIMMobility.ned`
- `SWIMMobility.h`
- `SWIMMobility.cc`


Using the SWIM Mobility Model
=============================

Check the examples provided by the INET Framework in the 'examples/mobility' folder to see how a mobility model is configured and used in a simulation.

The SWIM model has a number of configurable parameters that are defined in the 'SWIMMobility.ned'. Each of these parameters has default values and if they are required to be changed, use the omnetpp.ini file to set these changed values. 

- `alpha` is the alpha value used in the SWIM model (default is 0.5)
- TODO



Support
=======

If you have any questions or comments regarding this code, please write to,

- Anas bin Muslim (anas1@uni-bremen.de),
- Asanga Udugama (adu@comnets.uni-bremen.de) or 
- Anna Foerster (anna.foerster@comnets.uni-bremen.de)

