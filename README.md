# 🛰 tudat-matlab-thrust-feedback

## 🎯 Features
- **Thrust feedback** of [TUDAT](https://tudat-space.readthedocs.io/en/latest/) application using **MATLAB**
- Assess the performance of **novel control solution** developed in MATLAB to a single spacecraft / constellation of satellites
- **High-fidelity** numerical propagation 
- **Easy** to implement control strategy in **high-level** MATLAB language
- **Seamless** transition between high-fidelity simulation using TUDAT and **simple propagation** in MATLAB for **debug** puposes
- Custom **makefile** to automate **compilation**, **simulation**, and **post-processing**

***
## 🚀 Index

- [Description](#-description)
- [Authors](#-authors)
- [Contact](#-contact)
- [Installation](#-installation)
- [Requisites](#-requisites)
- [Documentation](#-documentation)
- [Example](#-example)
- [Contributing](#-contributing)
- [Lincense](#-license)
- [References](#-references)

***

## 💡 Description

The **tudat-matlab-thrust-feedback** toolbox provides an **interface** between a C++ [TUDAT](https://tudat-space.readthedocs.io/en/latest/) application and **MATLAB** to define a **thrust feedback control law**. The architecture of the overall environment is shown below.

![simulation_scheme drawio-2-2](https://user-images.githubusercontent.com/40807922/177381510-b3d2b191-8a1d-427d-a67c-f419e7efef95.svg)

The **tudat-matlab-thrust-feedback** toolbox supports the simulation of **multiple spacecrafts**, each with its own feedback control law. Thus, it is suitable to simulate and control **large constellations** of satellites, for instance.

Advantages of **tudat-matlab-thrust-feedback** over implementing a thrust feedback control law directly in the TUDAT application:
- **Faster** and **easier** to **debug** and **implement** control laws in MATLAB
- **Fast** and **easy** to implement the distributed computation of control laws for a large-number of spacecrafts (using the [Parallel Computing Toolbox](https://www.mathworks.com/products/parallel-computing.html))
- Run TUDAT and MATLAB application in **separate machines** (e.g. a **computing server** if the control-law evaluation is very intensive)
- **Novel** control strategies are often **developed in MATLAB**

In few words:
> Combine the high-fidelity fast propagation of TUDAT with the practicality of MATLAB

***

## ✍🏼 Authors 
Leonardo Pedroso<sup>1</sup> <a href="https://scholar.google.com/citations?user=W7_Gq-0AAAAJ"><img src="https://cdn.icon-icons.com/icons2/2108/PNG/512/google_scholar_icon_130918.png" style="width:1em;margin-right:.5em;"></a> <a href="https://orcid.org/0000-0002-1508-496X"><img src="https://orcid.org/sites/default/files/images/orcid_16x16.png" style="width:1em;margin-right:.5em;" alt="ORCID iD icon"></a> <a href="https://github.com/leonardopedroso"><img src="https://github.githubassets.com/images/modules/logos_page/GitHub-Mark.png" style="width:1em;margin-right:.5em;" alt="ORCID iD icon"></a><br>
Pedro Batista<sup>1</sup> <a href="https://scholar.google.com/citations?user=6eon48IAAAAJ"><img src="https://cdn.icon-icons.com/icons2/2108/PNG/512/google_scholar_icon_130918.png" style="width:1em;margin-right:.5em;"></a> <a href="https://orcid.org/0000-0001-6079-0436"><img src="https://orcid.org/sites/default/files/images/orcid_16x16.png" style="width:1em;margin-right:.5em;" alt="ORCID iD icon"></a><br>
<sub>*<sup>1</sup>Institute for Systems and Robotics, Instituto Superior Técnico, Universidade de Lisboa, Portugal<br>
 
***

## 📞 Contact
**tudat-matlab-thrust-feedback** toolbox is currently maintained by Leonardo Pedroso (<a href="mailto:leonardo.pedroso@tecnico.ulisboa.pt">leonardo.pedroso@tecnico.ulisboa.pt</a>).

***
  
## 💿 Installation
  
To install **tudat-matlab-thrust-feedback**:
  
- **Clone** or **download** the repository to the desired installation directory
- Add folder <tt>src-tudat-matlab-thrust-feedback</tt> to the MATLAB path
```m
addpath('[installation directory]/tudat-matlab-thrust-feedback/src-tudat-matlab-thrust-feedback');
```
 
***  

## 🤝 Requisites  

To use **tudat-matlab-thrust-feedback**:
 
- you must have [tudat-bundle](https://github.com/tudat-team/tudat-bundle) installed. See [tudat-bundle's README](https://github.com/tudat-team/tudat-bundle#readme) for installation details.

- if you intent on applying a feedback control law on **orbital elements** rather than on **Cartesian coordinates**, you may consider using [osculating2mean toolbox](https://github.com/decenter2021/osculating2mean) for that conversion.
 
*** 

## 📖 Documentation
The documentation is divided into the following categories:
- [Setup thrust feedback in TUDAT](#setup-thrust-feedback-in-tudat)
- [Setup thrust feedback in MATLAB](#setup-thrust-feedback-in-matlab)
- [Run a simulation](#run-a-simulation)

### Setup thrust feedback in TUDAT

**1.** To include the **tudat-matlab-thrust-feedback** toolbox in the source code use:
```c++
#include "tudatThrustFeedbackMatlab.h"
```
 
**2.** Write the C++ TUDAT source for your application according to the [documentation](https://tudat-space.readthedocs.io/en/latest/) available. 
 
**3.** To add thrust acceleration with feedback from MATLAB create a <tt>tudatThrustFeedbackMatlab</tt> object. The constructor follows

```c++
tudatThrustFeedbackMatlab( const unsigned int port, 
                           const unsigned int numberOfSatellites, 
                           const double epochControlUpdatePeriod,
                           const int evaluationsPerUpdatePeriod, 
                           const double isp_times_g0, 
                           double simulationEndEpoch = -1)
```

where
 - <tt>port</tt>: port of the local MATLAB server
 - <tt>numberOfSatellites</tt>: number of spacecrafts to provide thrust feedback
 - <tt>epochControlUpdatePeriod</tt>: interval of time between thrust feedback updates
 - <tt>evaluationsPerUpdatePeriod</tt>: number of numeric solver queries of the thrust vector per control sampling period
 - <tt>isp_times_g0</tt>: thruster specific impulse time standard gravity
 - <tt>simulationEndEpoch</tt>: epoch of simulation end (to display a progress bar)
 
> Example: <i>Create a <tt>tudatThrustFeedbackMatlab</tt> object</i>
> 
> Macros:
> ```c++
> // MATLAB server port
> #define SERVER_PORT 6013
> // Simulation end epoch (s)
> #define EPOCH_END 1000
> // Solver integration step (s)
> #define EPOCH_SAMPLE 10
> // Control update period (s)
> #define EPOCH_CONTROL_UPDATE 10
> // Thruster characteristics: Isp (s) and g0 (m/s^2)
> #define SAT_ISP 1640
> #define SAT_g0 9.81
> ```
> Create <tt>tudatThrustFeedbackMatlab</tt> object using a fixed-step RK4 solver (5 queries per integration step)
> ```c++
> // Select number of spacecraft
> unsigned int numberOfSatellites = 1584;
> // Create tudatThrustFeedbackMatlab object
> std::shared_ptr <tudatThrustFeedbackMatlab> ttfm = 
>     std::make_shared <tudatThrustFeedbackMatlab(SERVER_PORT,
>                                                 numberOfSatellites,
>                                                 EPOCH_CONTROL_UPDATE,
>                                                 5*EPOCH_CONTROL_UPDATE/EPOCH_SAMPLE,
>                                                 SAT_ISP*SAT_g0,
>                                                 EPOCH_END);
>```

**4.** To setup the acceleration model for each spacecraft:
First, assign sequential numeric ids to each spacecraft (starting at 0).

Second, bind a function for spacecraft <tt>i</tt> to <tt>thrustFeedbackWrapper</tt> method:
```c++
// Bind thrust feedback wrapper for each spacecraft
std::function <Eigen::Vector3d(const double)> thrustFeedbackSati =
    std::bind(&tudatThrustFeedbackMatlab::thrustFeedbackWrapper, 
              ttfm, 
              std::placeholders::_1,
              i,
              bodies.getMap());
```
where 
- <tt>bodies</tt> is the <tt>SystemOfBodies</tt> object of the simulation environment

Third, create a <t>ThrustAccelerationSettings</t> for each spacecraft with <tt>thrustFeedbackSati</tt> and add it to the acceleration map.

> Example: <i>Setup the acceleration models for a constellation of <tt>numberOfSatellites</tt>satellites</i>.
> Thrust defined in TNW frame relative to the Earth
> ```c++
> // ---------- Create planets objects ----------
> SystemOfBodies bodies = (...)
> 
> // ---------- Create vehicle object ----------
> std::string currentSatelliteName;
> for (unsigned int i = 0; i < numberOfSatellites; i++){
>     currentSatelliteName =  "sat" + boost::lexical_cast< std::string >(i);
>     bodies.createEmptyBody(currentSatelliteName);
>     bodies.at(currentSatelliteName) = std::make_shared< simulation_setup::Body >( );
> }
>
> // ---------- Setup acceleration models ----------
> SelectedAccelerationMap accelerationMap;
> std::vector <std::string> bodiesToPropagate;
> std::vector <std::string> centralBodies;
> 
> // Set thrust accelerations for each satellite.
> for ( unsigned int i = 0; i < numberOfSatellites; i++ ){
>     currentSatelliteName = "sat" + boost::lexical_cast< std::string >( i );
>     std::map<std::string, std::vector<std::shared_ptr<AccelerationSettings>>> accelerationsOfCurrentSatellite;
>     // Set acceleration of gravity, solar radiation pressure, atmospheric drag, third body, ...
>     // (...)
>     std::function <Eigen::Vector3d(const double)> thrustFeedback = 
>         std::bind(&tudatThrustFeedbackMatlab::thrustFeedbackWrapper, 
>                   ttfm, 
>                   std::placeholders::_1,
>                   i,
>                   bodies.getMap());
>     accelerationsOfCurrentSatellite[currentSatelliteName] = 
>         {std::make_shared<ThrustAccelerationSettings>(thrustFeedback,
>                                                       SAT_ISP,
>                                                       tnw_thrust_frame,
>                                                       "Earth")};
>     // Push satellite acceleration map
>     accelerationMap[currentSatelliteName] = accelerationsOfCurrentSatellite;
>     // Define body to propagate
>     bodiesToPropagate.push_back(currentSatelliteName);
>     // Define central body
>     centralBodies.push_back("Earth");
> }
> // Create acceleration models and propagation settings
> basic_astrodynamics::AccelerationMap accelerationModelMap = 
>     createAccelerationModelsMap(bodies,accelerationMap,bodiesToPropagate,centralBodies);
>```
 
### Setup thrust feedback in MATLAB


 
### Run a simulation

*** 
 
## 🦆 Example
 

***
  
## ✨ Contributing

The community is encouraged to contribute with 
- Suggestions
- Addition of tools

To contribute to **tudat-matlab-thrust-feedback**

- Open an issue ([tutorial on how to create an issue](https://docs.github.com/en/issues/tracking-your-work-with-issues/creating-an-issue))
- Make a pull request ([tutorial on how to contribute to GitHub projects](https://docs.github.com/en/get-started/quickstart/contributing-to-projects))
- Or, if you are not familiar with GitHub, [contact the authors](#-contact) 

***

## 📄 License
[MIT License](https://github.com/decenter2021/tudat-matlab-thrust-feedback/blob/master/LICENSE)

***

## 💥 References 
<p align="justify">


</p> 
