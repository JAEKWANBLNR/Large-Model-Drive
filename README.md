# Large-Model-Drive 

The Large-Model-Drive project is ROS2 humble project combined the on ROS-LLM (https://github.com/Auromix/ROS-LLM) and vlms_with_ros2_workshop (https://github.com/nilutpolkashyap/vlms_with_ros2_workshop) projects for mobile robot drive with natural language instruction and enhancing the situation awarness improvement for user. It enables understand the natural language instruction so that control the mobile robot and through the VLM (BLIP is used in this project) understand the scene and describe the situation. 

### Object Dection Model 
YOLOv8.n 

### Language Model 
GPT 3.5

### Vision-Language Model
BLIP 


# Project Example Screenshot  
![Untitled (6)](https://github.com/user-attachments/assets/c68a3396-634a-42fa-b706-2baea118636f)
![Untitled (7)](https://github.com/user-attachments/assets/666a267c-b05e-4ea6-824a-dbcdcf2647fa)


# Project rqt_grpah
![Untitled (3)](https://github.com/user-attachments/assets/23dba483-5822-48f8-8e7c-72a9f3732093)

# Project Experiments Video
preparing 


# Installation 

## 설치는 ROS-LLM Github링크를 따라하는게 더 좋을것 같습니다! 

## Version 
Ubuntu 22.04 
ROS2 Humble

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
