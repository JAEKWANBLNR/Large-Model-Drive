# Large-Model-Drive 

The Large-Model-Drive project is ROS2 humble project combined the on ROS-LLM (https://github.com/Auromix/ROS-LLM) and vlms_with_ros2_workshop (https://github.com/nilutpolkashyap/vlms_with_ros2_workshop) projects for mobile robot drive with natural language instruction and enhancing the situation awarness improvement for user. It enables understand the natural language instruction so that control the mobile robot and through the VLM (BLIP is used in this project) understand the scene and describe the situation. 

## Project Example Image 
![Untitled (4)](https://github.com/user-attachments/assets/18545761-3448-4516-878c-487a0f759e80)

## Project rqt_grpah
![Untitled (3)](https://github.com/user-attachments/assets/23dba483-5822-48f8-8e7c-72a9f3732093)

## Project Experiments Video
https://github.com/user-attachments/assets/73d7e666-e8c1-4aaf-941e-37244d6a7747


## Installation 
## 1. Clone the Repository:

`git clone https://github.com/JAEKWANBLNR/Large-Model-Drive.git`

## 2. Install Dependencies:

Navigate to the llm_install directory and execute the installation script.

`cd Large Model Drive/llm_install`

`bash dependencies_install.sh`

## 3. Configure OpenAI setting

`bash config_openai_api_key.sh`

## 4. Configuring AWS and OpenAI Whisper
 ### Audio input setting is denoted in the https://github.com/Auromix/ROS-LLM (ROS-LLM) Project
