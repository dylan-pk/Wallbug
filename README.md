# 🌱 Wallbot System Development  

---
*Wallbug Landing Page*.
https://dylan-pk.github.io/Wallbug/

![Overview](https://img.shields.io/badge/Overview-%23184E77?style=for-the-badge&logoColor=white)

The **Wallbot System Development** project aims to further develop and improve the current Wallbot into a fully functional **demonstration platform**.  
The **Wallbug Team** is responsible for the design and implementation of Wallbot.  

The system is designed to traverse vertical surfaces and demonstrate its role as a **plant surveying robot**, equipped with on-board sensors to monitor plant health and density.  

The end goal is to deliver an **investor-ready demo unit** that showcases technical capabilities, safety, and public appeal, while also serving as a stepping stone for **smart city green wall technologies**.  

---

![Objectives](https://img.shields.io/badge/Objectives-%233A5A40?style=for-the-badge&logoColor=white)

- Refurbish and integrate Wallbot components into a reliable demonstration unit.  
- Enable **vertical traversal** with wireless control and safe actuation.  
- Collect and display **plant health data** through onboard camera systems.  
- Improve **aesthetics, safety, and UI** for engaging demonstrations.  
- Supply **handover packages** (CAD, code, documentation) for scalability.  

---

![Scope](https://img.shields.io/badge/Scope-%23669EBC?style=for-the-badge&logoColor=white)

**In-Scope**  
- Wallbot movement and motor control  
- Wireless communication and control  
- Sensor integration and visual output  
- Mechanical shell redesign  

**Out-of-Scope**  
- Weather proofing for winch system  
- Z-axis climbing  
- Wind resistance at high altitude  
- Complex manipulation (e.g., pruning)  
- Renewable charging integration  

---

![Requirements](https://img.shields.io/badge/Requirements-%23A3B18A?style=for-the-badge&logoColor=black)

**Minimum Viable Product (MVP)**  
- Cable management system  
- Pre-planned movement demonstration  
- Modular, reusable subsystems  
- Camera data displayed on external monitor  
- Wireless communication  
- Plant health detection (3 colour stages)  

**Stretch Goals**  
- LED-based visual feedback  
- Autonomous docking/recharging  
- Autonomous movement  
- Single-press full demo sequence  

---

![Deliverables](https://img.shields.io/badge/Deliverables-%23184E77?style=for-the-badge&logoColor=white)

- **UI (D.1)**: Displays onboard sensors  
- **Functioning Demo (D.2)**: Pre-planned movement + wireless comms  
- **Mechanical Shell (D.3)**: Improved cable management + durability  
- **Documentation (D.4)**: Full technical + handover docs  
- **CAD (D.5) & Code Files (D.6)**: Provided at handover  

---

![Work Breakdown](https://img.shields.io/badge/Work_Breakdown_Structure-%233A5A40?style=for-the-badge&logoColor=white)

- **Mechanical**: Shell redevelopment, winches, housing, safety systems  
- **Electrical**: Motor communication (CANBus), wiring, load cells, control panel  
- **Mechatronics**: Motor control (speed/motion), path planning, stability  
- **UI**: Wireless control, real-time sensor display  

---

![Visual Identity](https://img.shields.io/badge/Visual_Identity-%23669EBC?style=for-the-badge&logoColor=white)

**Wallbug colour palette**  
![Dark Green](https://img.shields.io/badge/-%233A5A40?style=flat-square) ![Sage Green](https://img.shields.io/badge/-%23A3B18A?style=flat-square) ![Light Neutral](https://img.shields.io/badge/-%23EAE8E2?style=flat-square) ![Blue](https://img.shields.io/badge/-%23669EBC?style=flat-square) ![Deep Blue](https://img.shields.io/badge/-%23184E77?style=flat-square)  

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

3. Run the Wallbot UI demo:  
   ```bash
   python ui/main.py
   ```

---

![Team](https://img.shields.io/badge/Team_%26_Contributors-%23184E77?style=for-the-badge&logoColor=white)

### The Wallbug Team  

| Name               | Role                    | Responsibilities                                                |
|--------------------|-------------------------|----------------------------------------------------------------|
| **Anika Roth**     | Electrical Lead         | Electrical design, embedded systems, wiring                     |
| **Anton Cecire**   | Interface Lead          | UI design, interface integration                                |
| **Benjamin Cooper**| Movement Lead           | Wallbot movement, subsystem integration                         |
| **Connor Williams**| Industrial Comms Lead   | Motor communication (CANBus), firmware/software comms           |
| **Dylan Purbrick** | Team Lead               | Organisation, documentation, client liaison                     |
| **Fadi Alameddine**| Mechanical Design Lead  | Mechanical design, CAD, prototyping                             |

### Extended Contributors  

- **Dana** – Winches & Mechanical Actuation  
- **Adithya** – External Student, Wallbot Project Management Support  

### Client  

- **Marc Carmichael** – Client (UTS Robotics Institute, Head Tutor DMMS)  

### Project Coach  

- **Roy Wang** – Project Coach  

---

![Documentation](https://img.shields.io/badge/Documentation-%23669EBC?style=for-the-badge&logoColor=white)

All supporting materials, including CAD files, technical documentation, and meeting records, will be organised within the DMMS MS Teams dedicated Wallbot System Development Channel.  
Critical files such as ROS packages and further code elements will be stored within their respective folders.  

---

![License](https://img.shields.io/badge/License-%233A5A40?style=for-the-badge&logoColor=white)

This project is currently unlicensed. All rights reserved to the Wallbug team and collaborators.  
