# EcoDex

EcoDex is an innovative service designed to address the problems of garbage collection and segregation by automating the task and enabling tracking of waste collection based on location and type.

## Features
EcoDex consists of four main parts:

1. **Machine Learning Algorithm**
2. **ROS-Based Communication**
3. **Website-Based Monitoring**
4. **Manual Control for Debugging and Error Correction**

---

## Implementation

### **1. Machine Learning Algorithm**

#### Garbage Detection:
- Utilizes the **Ultralytics RT-DETR** model architecture.
- Fine-tuned on a modified version of the TACO dataset, where 60 original classes were aggregated into 25 target classes.
- The dataset was enhanced using extensive image augmentation techniques to address class imbalance and improve generalization.

#### Distance and Angle Calculation:
- Uses camera focal length and horizontal field of view (HFoV) properties.
- **Focal Length Calculation Formula:**

```
f = (w * d) / W
```

- `w`: Width in pixels  
- `W`: Actual width of the object  
- `d`: Distance from the camera

- **Distance Calculation Formula:**

```
d = (W * f) / w
```
![411771134-4730d42d-f5f9-43c2-82c5-b9e590202065](https://github.com/user-attachments/assets/60e1bed0-f32d-428e-96b9-7125f50ec08f)
  
- **Angle Calculation Formula:**

```
theta = (width_of_frame / Hfov) * (centre_of_screen - centre_of_bounding_box)
```

#### Optimized navigation to target object:
- Calculates delay for the bot to reach a particular distance, optimizing navigation based on detected objects.

---

### **2. ROS-Based Communication**
- Employs ROS2 as the primary tool for device communication.
- Tasks are modularized into multiple publisher and subscriber nodes:
  - Input from camera
  - Object detection
  - Distance and angle calculation
  - Data transmission to the microcontroller

- Final processed values are sent to an ESP microcontroller, which enables the bot to pick up and segregate garbage.

---

### **3. Website-Based Monitoring**
- A custom-built website provides user login and sign-in functionalities.

#### Key Features:
- **Data Storage:**
  - Tracks detected garbage data, categorized into biodegradable and non-biodegradable materials.

- **User-Based Monitoring:**
  - Each user or company can log in to access their bot's data.
  - Displays collected garbage details (e.g., weight in kilograms), bot location, collection points, and current bot status.

---

### **4. Manual Control-Based Debugging Method**

#### Purpose:
1. **Autonomous Functionality Backup:**
   - Ensures that if the bot's autonomous system fails, manual controls can take over seamlessly.
2. **Debugging:**
   - Facilitates debugging and correcting hardware errors.

---

## Additional Details from Presentation

### Autonomous System Overview
- The rover detects and navigates to garbage using onboard sensors and AI.
- Custom-built collection mechanism stores waste internally.
- Records weight, location, and estimated carbon footprint.

### Central Database and Web Dashboard
- Data from the bot is uploaded to a cloud database.
- Users can view stats, location tracking, and environmental impact from a centralized dashboard.
- Supports multi-bot monitoring.

### Design and Mechanism
- **Storage Unit:** Internal storage for collected waste.
- **Collection Mechanism:** Mechanized arm/tool to retrieve and hold waste.
- **Movement System:** Uses omni wheels for enhanced maneuverability.

![image](https://github.com/user-attachments/assets/30fd49c8-c8c9-457a-9916-933f1629ecaf)
![image](https://github.com/user-attachments/assets/112dd79c-94be-4664-adea-4bd9f5e40e10)

**Storage Unit**  
![image](https://github.com/user-attachments/assets/c0160ed6-7b6d-4233-9577-83c4674449b7)  
![image](https://github.com/user-attachments/assets/56bc6e16-7cc7-4855-b138-e9b64b12c1b3)

**Collection Mechanism**  
![image](https://github.com/user-attachments/assets/dee31dbd-eb06-45be-a0d0-fc205df8a9d0)  
![image](https://github.com/user-attachments/assets/ce277c17-4e39-4355-80aa-57cbf646ae41)

**Movement (Omni Wheels)**  
![image](https://github.com/user-attachments/assets/a638a27d-15b3-4d49-bea5-46da81f9a42b)  
![image](https://github.com/user-attachments/assets/93102298-a75f-4ba1-b17e-2edbd18976d9)

---

### Object Detection Using Ultralytics RTDETR
- Fast and accurate real-time waste detection in cluttered environments.
- Pretrained to recognize various waste categories.
- Optimized for low-power devices.
- Integrates tightly with the bot’s physical waste collection.

---

### Control System and Circuitry
- Dedicated control board integrated with sensors and actuators.
- Controlled using ROS nodes and publisher/subscriber architecture.

![image](https://github.com/user-attachments/assets/ee4c2595-c8b8-4260-912a-c7903c8c62a4)

---

### ROS Modularity
- Modular ROS node setup for image input, detection, path planning, motor control, and data logging.

![image](https://github.com/user-attachments/assets/163785d4-63c3-4eaa-8d6f-04204ab520ff)

---

### Website and Tech Stack
- Built using:
  - **Next.js**
  - **Tailwind CSS**
  - **Leaflet API** (for map/location)
  - **ChatBase API**
  - **MongoDB**

![image](https://github.com/user-attachments/assets/3ba092cf-7e95-47c2-89c0-c28cabb7df95)

---

### Flow Diagram
- Flowchart showing system-level interaction among detection, collection, control, and monitoring.

![image](https://github.com/user-attachments/assets/3bf39f34-2526-4fab-8549-8b031f5aa04d)  
![image](https://github.com/user-attachments/assets/bf6454ac-0b4e-4d57-8225-d30a4694f6ef)

---

## Innovation and Uniqueness
- Fully autonomous garbage navigation and collection.
- Real-time stats for location, type, and amount of waste.
- Web dashboard with bot status and stats.
- Location-aware with map-based tracking.

---

## Challenges
- Requires highly accurate detection under different lighting and environmental conditions.
- Orientation of bot and pickup arm must be precise.
- Obstacle avoidance and navigation require advanced spatial awareness and feedback mechanisms.

---

## Future Improvements
- Support for more waste categories.
- Add dumping station emptying mechanism.
- Enable multi-terrain navigation.

---

## Quick Start (Web Dashboard)

### Demo mode (no database — for recruiters / portfolio)

Deploy or run with sample data only. No MongoDB, login, or robot required.

```powershell
cd WebApp
npm install
$env:DEMO_MODE="true"; npm run dev
```

Open the URL shown in the terminal — you land directly on the dashboard with simulated waste collection data.

**Deploy to Vercel:**

1. Import the repo and set the root directory to `WebApp`
2. Add environment variable: `DEMO_MODE` = `true`
3. Deploy — no other env vars needed

### Full mode (with MongoDB)

```powershell
cd WebApp
npm install
npm run dev:local
```

Open `http://localhost:3000` (or the port shown in the terminal). Sign up with any username/email/password — sample waste data is seeded automatically for new users.

For production or a real MongoDB Atlas cluster, copy `WebApp/.env.example` to `WebApp/.env.local`, set `MONGODB_URI`, then:

```powershell
npm run build
npm start
```

Optional Flask override backend (for `/override`):

```powershell
pip install -r stone-security-backend/requirements.txt
python stone-security-backend/server.py
```

---

## Contributors
- [**Ayush Acharya**](https://github.com/ayushacharya27)  
- [**Joel Dantis**](https://github.com/joeldantis)  
- [**Gobi E**](https://github.com/Gobi05-exe)  
- [**Arjun Devraj**](https://github.com/Arjundevraj05)  

---

## License
This project is licensed under the [MIT License](LICENSE).
