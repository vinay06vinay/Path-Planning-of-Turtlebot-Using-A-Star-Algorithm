# Project Members:
1. Anuj Zore - zoreanuj - 119198957
2. Vinay Krishna Bukka - vinay06 - 118176680

# Github Link: 

https://github.com/vinay06vinay

## Part - 1
### Instructions to Run the Code : 

1. From the extracted zip file name "proj3p2_vinay_anuj.zip", take out the "Part1/vinay_anuj_part1.py" file from folder and open any Python IDE or terminal
2. If Python IDE is used, please click on the run button
Note : Before running please make sure whether these libraries are installed in the system : numpy,cv2,matplotlib,time.
Any Library not present can be by typing below command in terminal window using pip:
       pip install numpy
       
### Input Format for Two Test Cases : Click on enter after entering a value at each prompt

#### Test Case 1 : 
	Enter the clearance of the robot (0-50 mm)
	 50
	Enter the Left RPM of the motor
	 30
	Enter the Right RPM of the motor
	 60
	Enter the x-coordinate of start node
	 20
	Enter the y-coordinate of start node
	 20
	Initial Orientation of start node of robot(Angle in multiple of 30)
	 30
	Enter the x-coordinate of goal node
	 300
	Enter the y-coordinate of goal node
	 170
#### Test Case 2 : 
	Enter the clearance of the robot (0-50 mm)
	 50
	Enter the Left RPM of the motor
	 80
	Enter the Right RPM of the motor
	 80
	Enter the x-coordinate of start node
	 20
	Enter the y-coordinate of start node
	 20
	Initial Orientation of start node of robot(Angle in multiple of 30)
	 60
	Enter the x-coordinate of goal node
	 230
	Enter the y-coordinate of goal node
	 95



4. Once the code is run, a few information about the code is printed on the console and the cv2 visualisation video is recorded and saved
5. Video will be saved under the name "A-Star.avi". Alternatively, you can find the video in below links
    * TestCase1 Video : https://drive.google.com/file/d/1Y7eHnxFMTtI78IM5azrXAueo4jW7tZWt/view?usp=sharing
    * TestCase2 Video : https://drive.google.com/file/d/1exwrX8Vh8TrgUb0uQc2RJ2Rg4S5zTUjO/view?usp=sharing
6. If Using, VSCode or Spyder you will also see a final plot of the map using Opencv and Matplotlib


## Part - 2
### Instructions to Run the Code : 

#### Test Case

	Enter the clearance of the robot (0-50 mm)
	 50
	Enter the Left RPM of the motor
	 80
	Enter the Right RPM of the motor
	 80
	Enter the x-coordinate of start node
	 20
	Enter the y-coordinate of start node
	 20
	Initial Orientation of start node of robot(Angle in multiple of 30)
	 60
	Enter the x-coordinate of goal node
	 230
	Enter the y-coordinate of goal node
	 95
1. First, download the package from below path after extracting the zip
	* Part2/a_star
2. Keep this folder "a_star" inside the source folder of your respective ros workspace.
3. Perfrom below set of commands now 
	* cd ~/catkin_ws
	* catkin_make
	* source devel/setup.bash
	* roslaunch a_star astar.launch
4. A new terminal window will be opened where you need to enter the coordinates as in Test Case.
5. The turtlebot keeps moving to its goal point. The video link for the same is below:
	* Video Link : 
