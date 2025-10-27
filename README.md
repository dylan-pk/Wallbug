# 🌱 Wallbot System Development  

---

![Overview](https://img.shields.io/badge/Overview-%23184E77?style=for-the-badge&logoColor=white)

The **Wallbot System Development** project represents the finalised redevelopment and delivery of the *Wallbot* platform by the **Wallbug Team** under the DMMS subject at the University of Technology Sydney.  
The system is designed to traverse vertical surfaces and function as a **plant‑surveying robot**, equipped with sensors to monitor plant health and density.  

This project delivers an **investor‑ready demonstration platform**, fulfilling requirements outlined in the *Project Scope and Specifications (Rev E)* and the *Project Handover Plan (Rev B)*.  
It demonstrates the integration of mechanical, electrical, and software subsystems, supported by a refined user interface and visual branding package.  

---

![Objectives](https://img.shields.io/badge/Objectives-%233A5A40?style=for-the-badge&logoColor=white)

- Develop a safe, reliable, and visually engaging **Wallbot demonstration platform**.  
- Enable **vertical traversal** and plant health data collection via onboard sensors.  
- Ensure **stable communication** between winches, motors, and control UI.  
- Integrate subsystems into a cohesive, modular architecture for ease of testing and reuse.  
- Deliver **complete handover documentation**, CAD, and functional demonstration video package.  

---

![Scope](https://img.shields.io/badge/Scope-%23669EBC?style=for-the-badge&logoColor=white)

**In‑Scope**  
- Wallbot vertical mobility and platform control  
- Motor coordination through CANBus and central controller  
- Sensor integration for plant health detection  
- Wireless control via UI panel  
- Redeveloped shell with improved cable and safety design  

**Out‑of‑Scope**  
- Full autonomous navigation beyond predefined path  
- Extended weatherproofing or outdoor deployment  
- Long‑term power autonomy or recharging integration  
- Multi‑unit synchronisation beyond single‑system demo  

---

![Requirements](https://img.shields.io/badge/Requirements-%23A3B18A?style=for-the-badge&logoColor=black)

**Minimum Viable Product (MVP)**  
- Safe cable management system and winch coordination  
- Pre‑programmed demonstration path for reliable motion  
- Wireless communication between UI and motor controllers  
- Visual indicator of plant health via colour segmentation (3 stages)  
- Full documentation and build reproducibility  

**Stretch Goals**  
- Enhanced LED feedback and display integration  
- Automated start/stop demo sequence  
- Expanded telemetry data visualisation in UI  

---

![Deliverables](https://img.shields.io/badge/Deliverables-%23184E77?style=for-the-badge&logoColor=white)

**As defined in Project Handover Plan (Rev B)**  

| Deliverable ID | Description | Status |
|----------------|--------------|---------|
| **D.1b** | Updated System UI and Display Integration | ✅ Completed |
| **D.2b** | Functional Demonstration – Pre‑Programmed Traversal | ✅ Completed |
| **D.3b** | Redeveloped Mechanical Shell and Safety Housing | ✅ Completed |
| **D.4b** | Full Technical Documentation and Assembly Pack | ✅ Completed |
| **D.5b** | Complete CAD and Code Package in Repository | ✅ Completed |
| **D.6b** | Final Digital Handover and Video Demonstration | ✅ Completed |

---

![Work Breakdown](https://img.shields.io/badge/Work_Breakdown_Structure-%233A5A40?style=for-the-badge&logoColor=white)

- **Mechanical** – Shell redevelopment, winches, and safety systems  
- **Electrical** – Motor communication (CANBus), wiring, and load cell integration  
- **Mechatronics** – Motion control, path planning, and subsystem integration  
- **UI** – Wireless control interface and live sensor display  

---

![Visual Identity](https://img.shields.io/badge/Visual_Identity-%23669EBC?style=for-the-badge&logoColor=white)

**Wallbug Colour Palette**  

| Colour Name   | Hex Code   | Swatch |
|---------------|------------|--------|
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

3. Run the Wallbot UI demo:  
   ```bash
   python ui/main.py
   ```

---

![Team](https://img.shields.io/badge/Team_%26_Contributors-%23184E77?style=for-the-badge&logoColor=white)

### The Wallbug Team  

| Name               | Role                    | Responsibilities                                                |
|--------------------|-------------------------|----------------------------------------------------------------|
| **Anika Roth**     | Electrical Lead         | Electrical design, embedded systems, wiring                     |
| **Anton Cecire**   | Interface Lead          | UI design, interface integration                                |
| **Benjamin Cooper**| Movement Lead           | Wallbot movement, subsystem integration                         |
| **Connor Williams**| Industrial Comms Lead   | Motor communication (CANBus), firmware/software comms           |
| **Dylan Purbrick** | Team Lead               | Organisation, documentation, client liaison                     |
| **Fadi Alameddine**| Mechanical Design Lead  | Mechanical design, CAD, prototyping                             |

### Extended Contributors  
- **Dana** – Winches & Mechanical Actuation  
- **Adithya** – External Student, Wallbot Project Management Support  

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

The **Wallbot System Development** project was formally handed over to the client, **Marc Carmichael** (UTS Robotics Institute), at **1:00 PM AEST on Monday, 13 October 2025**.  
This delivery marks the completion of work as specified in:  
- *Project Scope and Specifications – Rev E (Signed)*  
- *Project Handover Plan – Rev B*  

All deliverables (D.1b – D.6b) were demonstrated, verified, and accepted by the client in accordance with the final handover criteria.  

---

![Version](https://img.shields.io/badge/Version_History-%23669EBC?style=for-the-badge&logoColor=white)

| Document | Version | Date | Description |
|-----------|----------|------|--------------|
| Project Scope & Specifications | Rev E | 13 Oct 2025 | Final signed version by Client |
| Project Handover Plan | Rev B | 13 Oct 2025 | Accepted handover deliverables & completion record |

---

![License](https://img.shields.io/badge/License-%233A5A40?style=for-the-badge&logoColor=white)

This project is currently unlicensed. All rights reserved to the Wallbug Team and collaborators.  
