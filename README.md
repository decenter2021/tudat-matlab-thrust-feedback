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

- 💡 [Description](#-description)
- ✍🏼 [Authors](#-authors)
- ✨ [Contributors](#-contributors)
- 📞 [Contact](#-contact)
- 💿 [Installation](#-installation)
- 🤝 [Requisites](#-requisites)
- 📖 [Documentation](#-documentation)
-  [`macOS` Dedicated Section](#-macos-dedicated-section)
- 🦆 [Example](#-example)
- ✨ [Contributing](#-contributing)
- 📄 [License](#-license)
- 💥 [References](#-references)

***

## 💡 Description

The **tudat-matlab-thrust-feedback** toolbox provides an **interface** between a C++ [TUDAT](https://tudat-space.readthedocs.io/en/latest/) application and **MATLAB** to define a **thrust feedback control law**. The architecture of the overall environment is shown below.

<p align="center">
  <img src="https://user-images.githubusercontent.com/40807922/177381510-b3d2b191-8a1d-427d-a67c-f419e7efef95.svg">
</p>

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
<sub><sup>1</sup>Institute for Systems and Robotics, Instituto Superior Técnico, Universidade de Lisboa, Portugal<br>
 
***

## ✨ Contributors  

Bruno Midões <a href="https://github.com/BrunoMid"><img src="https://github.githubassets.com/images/modules/logos_page/GitHub-Mark.png" style="width:1em;margin-right:.5em;" alt="github icon"></a><br>
João Marafuz Gaspar <a href="https://github.com/joaomarafuzgaspar"><img src="https://github.githubassets.com/images/modules/logos_page/GitHub-Mark.png" style="width:1em;margin-right:.5em;" alt="github"></a> <a href="https://www.linkedin.com/in/joaomarafuzgaspar/"><img src="https://i.stack.imgur.com/gVE0j.png" style="width:1em;margin-right:.5em;" alt="linkedin"></a>

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

- [**Windows users**] all procedures for the instalation of [tudat-bundle](https://github.com/tudat-team/tudat-bundle) assume the use of WSL. You must install the WSL1 version. See [Linux on Windows with WSL](https://learn.microsoft.com/en-us/windows/wsl/install) for installation details.

- if you intent on applying a feedback control law on **orbital elements** rather than on **Cartesian coordinates**, you may consider using [osculating2mean toolbox](https://github.com/decenter2021/osculating2mean) for that conversion.
 
*** 

## 📖 Documentation
The documentation is divided into the following categories:
- [Setup thrust feedback in TUDAT](#setup-thrust-feedback-in-tudat)
- [Setup thrust feedback in MATLAB](#setup-thrust-feedback-in-matlab)
- [Run a simulation](#run-a-simulation)

### Setup thrust feedback in TUDAT
 
It is recommended to setup the thrust feedback in **TUDAT** from the **template** available in [Example](#-example). 

This template allows:
- to define **common MACROS** between **TUDAT** and **MATLAB** in C header file
- save the thrust data and spacecraft state automatically to a <tt>.mat</tt> file
 
For simulation data post-processing see [Run a simulation](#run-a-simulation).
 
The main steps for seting up the thrust feedback in TUDAT **from sractch** are shown below.

**1.** To include the **tudat-matlab-thrust-feedback** toolbox in the source code use:
```cpp
#include "tudatThrustFeedbackMatlab.h"
```
 
**2.** Write the C++ TUDAT source for your application according to the [documentation](https://tudat-space.readthedocs.io/en/latest/) available. 
 
**3.** To add thrust acceleration with feedback from MATLAB create a <tt>tudatThrustFeedbackMatlab</tt> object. The constructor follows

```cpp
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
> ```cpp
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
> ```cpp
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
```cpp
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
> Thrust defined in TNW frame relative to the Earth.
> ```cpp
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
 
See [Example](#-example) for a complete example. 

***
 
### Setup thrust feedback in MATLAB
 
It is recommended to setup the thrust feedback in MATLAB, from the template available in [Example](#-example). 

This template allows:
- to define **common MACROS** between **TUDAT** and **MATLAB** in C header file 
- transition seamlessly between TUDAT simulation and a simplistic matlab simulation (for debug purposes)
 
The main steps for seting up the thrust feedback in MATLAB **from sractch** are shown below.

**1.** Setup MATLAB UDP server to listen for feedback requests
 
```m
% Setup server
% Output position-velocity-mass as vector: flag = 0 | Output as matrix: flag = 1
tudat = tudatMatlabServer(port,addr,N,flag);
% Setup cleanup
tudatCleanUp = onCleanup(@()tudat.delete());
% Wait for tudat-app
tudat.waitForClient();
```
where 
 - <tt>port</tt>: server port
 - <tt>addr</tt>: server address
 - <tt>N</tt>: number of spacecrafts
 - <tt>flag = 0</tt>: position-velocity-mass of all spacecraft is concatenated in a single vector for state feedback
 - <tt>flag = 1</tt>: position-velocity-mass of all spacecraft are the columns of matrix for state feedback
 
> Example: *Setup MATLAB UDP server* 
> ```m
> N = 1584;
> port = 6013;
> addr = '127.0.0.1';
> % Setup server
> tudat = tudatMatlabServer(port,addr,N,flag);
> % Setup cleanup
> tudatCleanUp = onCleanup(@()tudat.delete());
> % Wait for tudat-app
> tudat.waitForClient();
> ```

**2.** Wait for to feedback requests with 
```m 
[t,x_t,state] = tudat.getRequest();
```
where
- <tt>t</tt>: epoch of feedback request
- <tt>x_t</tt>: state for feedback of all satellites (output defined by <tt>falg</tt> in step 1)
- <tt>state</tt>: if $\neq 0$ TUDAT simulation is finished and the server may be terminated

**3.** Compute the actuation and send it to TUDAT with 
```m 
% Implement control law
%u_t = f(t,x_t)  
% Send feedback
tudat.sendResponse(u_t); 
```
where
 - <tt>u_t</tt>: concatenation of the thrust vectors of all spacecrafts 
 
**4.** Terminate the server using 
```m
if state 
    clear tudatCleanUp;
end
```

***
 
### Run a simulation

To run a simulation, we simply have to:
 - compile TUDAT C++ app using the [tudat-bundle](#-requisites)
 - run the MATLAB script and setup the feedback server  
 - afterwards, run the TUDAT executable

To automate this process a template of a `makefile` is available. To setup this `makefile` for your needs you have to set some variables in the beginning of the `makefile`:
```make
# -------------- Set paths -----------------------------------------------------
# Matlab dirs
matlab-exe := /usr/local/MATLAB/R2021b/bin/matlab
# Tudat installation specific variables
tudat-bundle-dir := /mnt/nfs/home/lpedroso/tudat-bundle
# Tudat app specific variables
tudat-matlab-feedback-dir := ../src-tudat-matlab-thrust-feedback
cpp-src := tudat-app.cpp
matlab-feedback-src := matlab_app
matlab-check-control-cycles := 5
# ------------------------------------------------------------------------------
```
where 
 - `matlab-exe`: path of MATLAB app
 - `tudat-bundle-dir`: path of tudat-bundle installation directory
 - `tudat-matlab-feedback-dir`: path of tudat-matlab-thrust-feedback toolbox source code
 - `cpp-src`: `*.cpp`tudat app source code
 - `matlab-feedback-src`: `*.m` MATLAB script
 - `matlab-check-control-cycles`: number of control cycles to be run when testing `matlab-feedback-src` for errors
*** 
 
The available targets and their descriptions can be checked using
 ```make
$ make help
all: 		 Compile tudat app
tudat:		 Compile tudat app
matlab-check: Perform matlab check
run: 		 Run tudat app and matlab server
run-tudat: 	 Run tudat app
output: 		 Save output results to .mat file and zip raw output files alongside with the source code and logs
clean: 		 Clean output results, logs, and compiled objects (.zip of past simualtions are not deleted)
```
 
To run a simulation makinh use of this `makefile`:

**1.** **Compile** TUDAT app with 
```make
$ make tudat
(...)
[100%] Linking CXX executable ../../bin/tudat-app
make[3]: Leaving directory '/mnt/nfs/home/lpedroso/tudat-bundle/build'
[100%] Built target tudat-app
make[2]: Leaving directory '/mnt/nfs/home/lpedroso/tudat-bundle/build'
make[1]: Leaving directory '/mnt/nfs/home/lpedroso/tudat-bundle/build'
cp /mnt/nfs/home/lpedroso/tudat-bundle/build/tudat/bin/tudat-app ./ # Retrieve app
 ```
 
> **Warning**: Ignore the massive dump of warnings concerning the *use of old-style cast* during compilation
 
**2.** *(Optional)* Check matlab sript for errors:

```make
$ make matlab-check 
@MATLAB server: Defining nominal constellation.
@MATLAB server: Retrieving simulation parameters.
@MATLAB server: Initializing thrust feedback controller.
@MATLAB server: Thrust feedback controller waiting for requests.
[======================================================================] 100.00 %
Elapsed time is 7.017307 seconds.
```
 
It runs a simulation propagated in MATLAB for `$(matlab-check-control-cycles)` as defined in the `makefile` to catch any errors in the MATLAB script. 
 
**3.** Run the simulation:
 
```make 
$ make run
@Tudat: Creating MATLAB thrust feedback object for 30 satellites.
@Tudat: Entered establish connection UDP.
@Tudat: Sent ping.
@Tudat: MATLAB UDP connection established.
Started simulation.
Dependent variables being saved, output vector contains: 
Vector entry, Vector contents
[======================================================================] 100 %
Saving results.
Simulation complete.
```

A new terminal window is opens to run the MATLAB script, which outputs:
 
```
@MATLAB server: Setting up feedback server.
@MATLAB server: Starting MATLAB server.
@MATLAB server: Hosted at 127.0.0.1:6013.
@MATLAB server: Waiting for Tudat client.
@MATLAB server: Client connected.
@MATLAB server: Defining nominal constellation.
@MATLAB server: Retrieving simulation parameters.
@MATLAB server: Initializing thrust feedback controller.
@MATLAB server: Thrust feedback controller waiting for requests.
Elapsed time is 85.911747 seconds.
@MATLAB server: Thrust feedback controller has been terminated.
```

If the simulation is successfull:
- status logs are available at `/logs`
- the thrust force and state of each spacecraft for each interation step are saved as `.dat` files in `/output`

**4.** Process simulation results

To atomaticaly process the simulation results run 
```make
$ make output
(...)
adding: output/stateSat9.dat (deflated 56%)
adding: output/output.mat (deflated 0%)
adding: tudat-app.cpp (deflated 78%)
adding: tudat-matlab-parameters.h (deflated 60%)
adding: matlab_app.m (deflated 65%) 
adding: matlab_feedback_routine.m (deflated 33%)
adding: plot_simulation.m (deflated 67%) 
adding: makefile (deflated 63%)
adding: logs/cmake-tudat-app.log (deflated 85%)
adding: logs/get-tudat-output.log (deflated 69%)
adding: logs/matlab-check.log (deflated 71%)
adding: logs/run-matlab-server.log (deflated 70%)
adding: logs/run-tudat-app.log (deflated 99%)
```

The simulation results become available as
 - `/output/output.mat` with state and thrust evolution throughout the simulation
 - zip file in `/output` with all the raw simulation results and source code
 
**5.** *(Optional)* Clean simulation results

After having archived all the simulation results, you can delete
- the TUDAT executable
- the logs in  `/logs/` 
- output results in `/output`

with 
 
```make
$ make clean
Do you really want to delete all output results, logs, and executables? [y/n]
y
rm -rf /mnt/nfs/home/lpedroso/tudat-bundle/tudat/tudat-applications/tudat-app # Clean build directory
rm -f tudat-app # Clean tudat-app
rm -f ./output/*.dat
rm -f ./output/*.mat
rm -f ./output/*.txt
rm -rf ./logs
rm -f *.asv
```
 
> **Warning**: The `.zip` archives in `/output` are not deleted during `make clean`
*** 

##  `macOS` Dedicated Section
In this section is described the tested and validated procedure (in May 2024 by [@joaomarafuzgaspar](https://github.com/joaomarafuzgaspar)) for macOS users to be able to run this project without wasting a substancial amount of time. 

### 1. Be sure of some prerequisites
#### 1.1 `MATLAB` and its required add-ons
Install `MATLAB`. After this, open `MATLAB` and verify which add-ons you have installed.
```matlab
ver
```
You'll need the **Instrument Control Toolbox** and the **Signal Processing Toolbox**. You can install them through `MATLAB` Add-On Explorer.
#### 1.2 Make sure you have `boost` installed
```bash
$ brew install boost
```

### 2. Install requisite `tudat-bundle` and `tudat` at their stable tested commits for macOS
#### 2.1 `tudat-bundle` at commit `508ae48`
```bash
$ git clone git@github.com:tudat-team/tudat-bundle.git
$ cd tudat-bundle
$ git checkout 508ae48
```

#### 2.2 `tudat` at version `v2.10.6.dev13`
```bash
$ git submodule update --init --recursive
$ cd tudat
$ git checkout tags/v2.10.6.dev13
```

### 3. Comment lines `tudat-bundle/CMakeLists.txt:38-44` to speed up the build
This way `tudatpy`, which is not used, won't be built.
```cpp
37: # TudatPy project.
38: # if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/.tudatpy-documented)
39: #     message(STATUS "Using DOCUMENTED tudatpy")
40: #     add_subdirectory(".tudatpy-documented")
41: # else ()
42: #     message(STATUS "Using UNDOCUMENTED tudatpy")
43: #     add_subdirectory(tudatpy)
44: # endif ()
```

### 4. Update `tudat-bundle/tudat/CMakeLists.txt` to speed up the build and fix errors
Test suite is not needed.
```cpp
49: option(TUDAT_BUILD_TESTS "Build the test suite." OFF)
```
> [!CAUTION]
> For this update to work, `tudat-bundle/build.sh:49` needs to be replaced by `BUILD_TESTS="${build_tests:-0}"`.

We only want to propagate the dynamics as precise as possible, while making use of [NASA's `NRLMSISE-00` atmospheric model](https://ccmc.gsfc.nasa.gov/models/NRLMSIS~00/).
```cpp
60: option(TUDAT_BUILD_WITH_ESTIMATION_TOOLS "Build tudat with estimation tools." OFF)

78: option(TUDAT_BUILD_WITH_NRLMSISE00 "Build with nrlmsise-00 atmosphere model." ON)

84: option(TUDAT_BUILD_WITH_EXTENDED_PRECISION_PROPAGATION_TOOLS "Build tudat with extended precision propagation tools." ON)
```
Add the missing `boost` component - `unit_test_framework`.
```diff
- 149: set(_TUDAT_REQUIRED_BOOST_LIBS filesystem system regex date_time thread
- 150:        chrono atomic)
+ 149: set(_TUDAT_REQUIRED_BOOST_LIBS filesystem system regex date_time thread chrono atomic unit_test_framework)
```
Add the application directory to the build for the executable to be generated.
```cpp
347: add_subdirectory(tudat-applications/tudat-app)
```

### 5. Git clone `tudat-matlab-thrust-feedback` to `tudat-bundle`'s directory
```bash
$ git clone git@github.com:decenter2021/tudat-matlab-thrust-feedback.git
$ cd tudat-bundle/example
```

### 6. Go to *[Run a simulation](#run-a-simulation)* to continue with the setup!
> [!NOTE]
> You don't need to worry about building `tudat`, it will be built automatically with:
> ```bash
> $ make tudat
> ```
***

## 🦆 Example
 
The example in `/example` can be used a template to use the **tudat-matlab-thrust-feedback** toolbox. All the source files are **thoroughly commented**.

The C header `/example/tudat-matlab-parameters.cpp` is used to define a set of common macros used by TUDAT and MATLAB
 
The TUDAT source code `/example/tudat-app.cpp` is written according to [Setup thrust feedback in TUDAT](#setup-thrust-feedback-in-tudat)
 
The MATLAB feedback script is dived into two parts:
 - `/example/matlab_app.m`: setup server, simulation environment, and define controller parameters according to [Setup thrust feedback in MATLAB](#setup-thrust-feedback-in-matlab)
 - `/example/matlab_feedback_routine.m`: implement the control policy, i.e., compute the actuation of every spacecraft acording to the position-velocity-mass vectors and time-intant 
 
For debug puposes, you can run the simulation directly in MATLAB with a simplistic propagation taking into account:
- Point-mass gravity of the Earth
- Effect of $J_2$
- Linearized atmospheric drag 
 
To run it:
 - set the variable `tudatSimulation` to `false` in `/example/matlab_app.m`
 - run script `/example/matlab_app.m` dirctely in MATLAB
 
In this example, a LEO constellation of 30 satellites is simulated. A dummy feedback law is used, which after half of an orbital period sets a constant thrust along the velocity vector. 
 
The example can be run according to [Run a simulation](#run-a-simulation). The simulation results for satellite 13 are shown below 

<p align="center">
  <img src="https://user-images.githubusercontent.com/40807922/177766203-c820703e-605b-4e9b-a949-e8d24c2cb0a0.svg">
</p>
<p align="center">
  <img src="https://user-images.githubusercontent.com/40807922/177766208-d3820fce-f38f-4c9f-a795-32a3957d48bf.svg">
</p>
<p align="center">
  <img src="https://user-images.githubusercontent.com/40807922/177766209-bcc13682-6352-469f-bb7b-b51003d508de.svg">
</p>
<p align="center">
  <img src="https://user-images.githubusercontent.com/40807922/177766211-c6006350-8149-4ab8-9e31-57c783773112.svg">
</p>
<p align="center">
  <img src="https://user-images.githubusercontent.com/40807922/177766214-e7bad7b6-d9e3-4af2-911d-415ba9132623.svg">
</p>
<p align="center">
  <img src="https://user-images.githubusercontent.com/40807922/177766212-3c1fec2e-9d91-4b33-aa25-548315d579ff.svg">
</p>

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

<a href="https://www.sciencedirect.com/science/article/pii/S0967066123002976">L. Pedroso and P. Batista, *Distributed decentralized EKF for very large-scale networks with application to satellite mega-constellations navigation*, Control Engineering Practice, vol. 135, pp. 105509, 2023. doi: 10.1016/j.conengprac.2023.105509.</a>
 
