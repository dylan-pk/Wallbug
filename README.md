# 🌱 Wallbot System Development  

---

![Overview](https://img.shields.io/badge/Overview-%23184E77?style=for-the-badge&logoColor=white)

The **Wallbot System Development** project represents the finalised redevelopment and delivery of the _Wallbot_ platform by the **Wallbug Team** under the DMMS subject at the University of Technology Sydney.  
The system is designed to traverse vertical surfaces and function as a **plant‑surveying robot**, equipped with sensors to monitor plant health and density.  

This project delivers an **investor‑ready demonstration platform**, fulfilling requirements outlined in the _Project Scope and Specifications (Rev E)_ and the _Project Handover Plan (Rev B)_.  
It demonstrates the integration of mechanical, electrical, and software subsystems, supported by a refined user interface and visual branding package.  

---

![Objectives](https://img.shields.io/badge/Objectives-%233A5A40?style=for-the-badge&logoColor=white)

- Develop a safe, reliable, and visually engaging **Wallbot demonstration platform**.  
- Enable **vertical traversal** and plant health data collection via onboard sensors.  
- Ensure **stable communication** between winches, motors, and control UI.  
- Integrate subsystems into a cohesive, modular architecture for ease of testing and reuse.  
- Deliver **complete handover documentation**, CAD, and functional demonstration video package.  

---

<h2 id="scope">&#8203;</h2>
<img src="https://img.shields.io/badge/Scope-%23669EBC?style=for-the-badge&logoColor=white" alt="Scope">

**In‑Scope**  
- Robot movement  
- Motor control  
- Visual output  
- Wireless control and communication  
- Redeveloped shell with improved cable design  

**Out‑of‑Scope**  
- Weather proofing for winch  
- Z-axis movement  
- Accounting for wind speed at altitude  
- Complex manipulation (plant pruning etc.)  
- Renewable integration for charging
- Redesign component  

---

<h2 id="requirements">&#8203;</h2>
<img src="https://img.shields.io/badge/Requirements-%23A3B18A?style=for-the-badge&logoColor=black" alt="Requirements">

**Minimum Viable Product (MVP)**  
- Structured and secure cable management system withput interfering with current sensors   
- Pre‑programmed demonstration path for reliable motion  
- Wireless communication between UI and motor controllers  
- Visual indicator of plant health via colour segmentation (3 stages)  
- Full documentation and build reproducibility  

**Stretch Goals**  
- Enhanced LED feedback and display integration  
- Wireless recharging station  
- Autonamous navigation and movement   
- Single press complete demo of all elemnts of the Wallbot System  

---

![Deliverables](https://img.shields.io/badge/Deliverables-%23184E77?style=for-the-badge&logoColor=white)

**As defined in Project Handover Plan (Rev B)**  

| Deliverable ID | Description | Status |
|:--------------:|:--------------|:-------:|
| **D.1b** | Framework for the UI displaying on-board mock-up of sensors | ☐ |
| **D.2b** | A simulation demonstrating the Wallbot’s movement along a pre-planned path and a code framework for wireless communication | ☐ |
| **D.3b** | Upgraded mechanical shell to facilitate nicer wire management and durability | ☐ |
| **D.4** | Supplying all documentation files to Client (Marc) in the handover | ☐ |
| **D.5** | Supplying all CAD files to Client (Marc) in the handover | ☐ |
| **D.6** | Supplying all code files to Client (Marc) in the handover | ☐ |

  "✅Completed" _will populate the "Status" column upon handover_  
  
---

![Work Breakdown](https://img.shields.io/badge/Work_Breakdown_Structure-%233A5A40?style=for-the-badge&logoColor=white)

![Work Breakdown Structure](https://docs.google.com/drawings/d/e/2PACX-1vQRKzZUNqPZjXplz8AnqKoSKiiIOJTEvibl5ZlPDjqprP8tEGBfvGFfV0hOhyBmGjDRLVORC9NoSYw-/pub?w=3089&h=1033)


> _Figure 1. High-Level Work Breakdown Structure for Wallbot System Development – Wallbug Iteration_  
> 📎 [View WBS (Google Drawings)](https://docs.google.com/drawings/d/1HzqvC_d6uhQx4mHRx4d34mvmRflEmt4HzOK0eZC4jhY/edit?usp=sharing)


### _WBS Description Summary_ 
 
- **Mechanical**
  - **Shell Redevelopment – Fadi Alameddine**  
    Development of a new ladybug-like shell.  
  -  **_Actuator Development – Outsourced_**   
    _Development of casing and attachment of motors to wall._  

- **Electrical**
  - **Wire Management – Anika Roth**  
    Implementing a wire management tactic within the body of the Wallbot.  
  - **Wireless Control – Connor Williams**  
    Replace wired connections with wireless control and communication.  
    _In reference to hardware, protocols and ROS packages._  
  - **Charging Station – Anika Roth**  
    Designing and implementing an onboard power source and a way to charge it.  

- **Mechatronics**
  - **Motor Communication – Connor Williams**  
    Handling of motor driver to motor communication using CANBus.  
    *In reference to CANBus protocol and hardware requirements.*  
  - **Motor Control – Benjamin Cooper**  
    Handling of motor speeds and motion of Wallbot.  
    _In reference to hardware limitations._  
  - **Path Planning – Benjamin Cooper**  
    Predefined path for robot to follow during demonstration.  

- **Software**
  - **UI – Anton Cecire**  
    UI to visually show the on-board sensors on the Wallbot.  

- **Project Management / Oversight**
  - **Team Lead – Dylan Purbrick**  
    Responsible for overall project coordination, documentation, and client liaison.  
> _The above can be found within Project Specification (Rev E)_

---

![Visual Identity](https://img.shields.io/badge/Visual_Identity-%23669EBC?style=for-the-badge&logoColor=white)

**Wallbug Colour Palette**  

| Colour Name   | Hex Code   | Swatch |
|:---------------|:----------:|:------:|
| Dark Green    | `#3A5A40`  | ![Dark Green](https://img.shields.io/badge/-----------?style=flat-square&labelColor=3A5A40&color=3A5A40&logoColor=3A5A40) |
| Sage Green    | `#A3B18A`  | ![Sage Green](https://img.shields.io/badge/-----------?style=flat-square&labelColor=A3B18A&color=A3B18A&logoColor=A3B18A) |
| Light Neutral | `#EAE8E2`  | ![Light Neutral](https://img.shields.io/badge/-----------?style=flat-square&labelColor=EAE8E2&color=EAE8E2&logoColor=EAE8E2) |
| Blue          | `#669EBC`  | ![Blue](https://img.shields.io/badge/-----------?style=flat-square&labelColor=669EBC&color=669EBC&logoColor=669EBC) |
| Deep Blue     | `#184E77`  | ![Deep Blue](https://img.shields.io/badge/-----------?style=flat-square&labelColor=184E77&color=184E77&logoColor=184E77) |

---

![Getting Started](https://img.shields.io/badge/Getting_Started-%23A3B18A?style=for-the-badge&logoColor=black)

1. Clone the repository:  
   ```bash
   git clone https://github.com/dylan-pk/Wallbug.git
   cd Wallbug
   ```

2. Install dependencies:  
   ```bash
   pip install -r requirements.txt
   ```
   
3. Launch Gazebo and GUI:
   ```bash
   ros2 launch Wallbot_tl_root bringup.launch.py
   ```

4. Run the Wallbot UI demo:  
   ```bash
   python ui/main.py
   ```
5. Run the Wallbot demo:  
   ```bash
   colcon build --packages-select wallbot
   source install/setup.bash
   python3 src/Wallbug/src/demo.py
   ```

  
---

![Team](https://img.shields.io/badge/Team_%26_Contributors-%23184E77?style=for-the-badge&logoColor=white)

### The Wallbug Team  

| Name               | Role                    | Responsibilities                                                |
|:-------------------|:-----------------------:|:----------------------------------------------------------------|
| **Anika Roth**     | Electrical Lead         | Electrical design, embedded systems, wiring                     |
| **Anton Cecire**   | Interface Lead          | UI design, interface integration                                |
| **Benjamin Cooper**| Movement Lead           | Wallbot movement, subsystem integration                         |
| **Connor Williams**| Industrial Comms Lead   | Motor communication (CANBus), firmware/software comms           |
| **Dylan Purbrick** | Team Lead               | Organisation, documentation, client liaison                     |
| **Fadi Alameddine**| Mechanical Design Lead  | Mechanical design, CAD, prototyping                             |

### Extended Contributors  
- **Dana** – Winches & Mechanical Actuation  
- **Adithya** – External Student, Wider Wallbot Project Management  

### Client and Coach  
- **Marc Carmichael** – Client (UTS Robotics Institute, Head Tutor DMMS)  
- **Roy Wang** – Project Coach  

---

![Documentation](https://img.shields.io/badge/Documentation-%23669EBC?style=for-the-badge&logoColor=white)

All supporting materials, including CAD files, technical documentation, and meeting records, are stored within:  
- The **DMMS MS Teams → Wallbot System Development Channel**, and  
- The **GitHub repository** under `/docs` and `/assets` for final code, CAD, and documentation deliverables.  

---

![Handover](https://img.shields.io/badge/Handover_Completion-%233A5A40?style=for-the-badge&logoColor=white)

~~The **Wallbot System Development** project was formally handed over to the client, **Marc Carmichael** (UTS Robotics Institute), at **1:00 PM AEST on Monday, 13 Novemeber 2025** by **The Wallbug Team**.~~  
This delivery marks the completion of work as specified in:  
- _Project Scope and Specifications – Rev E (Signed)_  
- _Project Handover Plan – Rev B (Signed)_  

~~All deliverables (D.1b - D.3b & D.4 – D.6) were demonstrated, verified, and accepted by the client in accordance with the final handover criteria.~~  

---

![Version](https://img.shields.io/badge/Version_History-%23669EBC?style=for-the-badge&logoColor=white)

| Document | Version | Date | Description |
|:-----------|:--------:|:----:|:--------------|
| Project Scope & Specifications | Rev E | 11 Oct 2025 | Final signed version by Client |
| Project Handover Plan | Rev B | 3 Nov 2025 | Accepted handover deliverables & completion record |


---

![License](https://img.shields.io/badge/License-%233A5A40?style=for-the-badge&logoColor=white)

This project is currently unlicensed. All rights reserved to the Wallbug Team and collaborators.  
