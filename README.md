
# EcoDex - RAG-ED: Autonomous Waste Management System

EcoDex is an advanced waste management platform that integrates robotics, machine learning, and environmental tracking to offer seamless waste collection and segregation services. The platform is built with **Next.js** and styled using **Tailwind CSS** to ensure a responsive, user-friendly interface. EcoDex provides real-time data visualization, reporting, and autonomous waste management capabilities powered by RAG-ED robots.

## Services Offered

### 1. **Waste Collection Monitoring**
   - Track the number and weight of waste collected by RAG-ED robots over time. The platform provides an up-to-date count of waste categorized into various types, offering users clear insight into waste collection efforts.

### 2. **Autonomous Cleaning**
   - The RAG-ED robot operates autonomously to collect and categorize waste, sorting it into the following categories:
     - **Plastic**
     - **Metal**
     - **Paper**
     - **Biodegradable**
     - **Non-biodegradable**

### 3. **Carbon Emission Tracking**
   - Monitor carbon emissions reduced through waste collection efforts. EcoDex helps quantify the environmental benefits of using RAG-ED robots for waste management.

### 4. **Real-time Location Tracking**
   - View the live location of RAG-ED robots on an interactive map. EcoDex uses **Leaflet.js** to provide users with a detailed map interface to track robot movement in real-time.

### 5. **Waste Categorization**
   - RAG-ED robots are equipped with an object detection model that identifies waste and segregates it based on type, including:
     - **Plastic**
     - **Metal**
     - **Paper**
     - **Glass**
     - **Cardboard**
     - **Biodegradable**
     - **Non-biodegradable**

### 6. **Environmental Impact Reports**
   - EcoDex generates comprehensive reports on the waste management efforts performed by the RAG-ED robots. These reports provide detailed insights into environmental impact, including waste collected and carbon emissions reduced.

### 7. **Live Camera Feed**
   - Get a live video feed from the RAG-ED robot during its operation. Users can view the waste collection process in real-time, providing an added level of transparency.

## Technical Stack

### **Frontend**
- **Next.js**: EcoDex is built using the Next.js framework, which enables fast server-side rendering and excellent performance for user interactions.
- **Tailwind CSS**: The website is styled using Tailwind CSS to ensure a modern, responsive design that works across devices.

### **Backend**
- **MongoDB**: Waste collection data, including categorized waste types and environmental impact, is stored in a MongoDB database. The frontend fetches this data to display real-time statistics and reports.

### **Machine Learning**
- **YOLOv8 Object Detection Model**: The RAG-ED robot is integrated with the YOLOv8 model for accurate waste identification and classification. This model helps detect waste items such as plastic, metal, paper, glass, cardboard, and differentiates between biodegradable and non-biodegradable waste.

### **Real-time Features**
- **Leaflet API**: The map for real-time location tracking of RAG-ED robots is implemented using the Leaflet API. It allows users to track the robot's exact location during waste collection.
- **Chart.js**: Line and pie charts are generated using Chart.js to visualize trends, such as waste collection over time and the breakdown of waste categories.

### **Interactive Chatbot**
- **Chatbase.co**: An interactive chatbot is included in the platform, allowing users to ask questions and receive assistance about the system's features and operations.

## How it Works
1. **Autonomous Waste Collection**: The RAG-ED robot uses the YOLOv8 model to detect and categorize waste during its operation.
2. **Data Storage**: Collected data, such as waste types and quantities, is stored in a MongoDB database.
3. **Data Visualization**: The frontend fetches the stored data to display real-time stats using charts and tables. Users can view breakdowns of waste types, environmental impact reports, and more.
4. **Real-time Tracking**: The Leaflet.js-powered map provides a live view of the robot’s location as it moves through designated areas for waste collection.

## Getting Started

### Prerequisites
- **Node.js** (v14.x or higher)
- **MongoDB** (for backend database)
- **Yarn** or **npm** (for package management)

### Installation

1. Clone the repository:
   \`\`\`bash
   git clone https://github.com/yourusername/ecodex.git
   \`\`\`

2. Navigate to the project directory:
   \`\`\`bash
   cd ecodex
   \`\`\`

3. Install the required dependencies:
   \`\`\`bash
   yarn install
   # or
   npm install
   \`\`\`

4. Set up environment variables:
   Create a `.env.local` file in the root of the project and add your MongoDB connection string and any other necessary credentials.

   \`\`\`bash
   MONGODB_URI=<your_mongodb_uri>
   \`\`\`

5. Run the development server:
   \`\`\`bash
   yarn dev
   # or
   npm run dev
   \`\`\`

6. Open the app in your browser at [http://localhost:3000](http://localhost:3000).

### Deployment

EcoDex can be deployed to any platform that supports Next.js, such as **Vercel** or **Netlify**. Ensure your MongoDB database is accessible to your deployment environment.

## Future Enhancements
- **Improved AI Models**: Further training and refinement of the object detection model to increase waste categorization accuracy.
- **Expanded Environmental Reporting**: Additional metrics on environmental impact, such as water savings or air quality improvement through waste collection.


**Contact Information**
For further inquiries or collaboration, please contact us at [arjundevraj05@gmail.com].
