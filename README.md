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
> Combine the high-fidelity propagation of TUDAT with the practicality of MATLAB

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

## 📖 Documentation
The documentation is divided into the following categories:
- [Setup thrust feedback in TUDAT](#setup-trust-feedback-in-tudat)
- [Setup thrust feedback in MATLAB](#setup-trust-feedback-in-matlab)
- [Run a simulation](#run-a-simulation)

### Setup thrust feedback in TUDAT
 
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
