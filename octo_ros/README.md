# OctoROS
This projects aims to be a bridge between 3D Printers and ROS. 
It uses OctoPrint to control the printer, and get the info that is sent to ROS topics. 


## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

## Prerequisites

First you'll need to install OctoPrint, the instructions are available at https://github.com/foosel/OctoPrint. But the easiest way that I see and in order to integrate the application with ROS is with the following instructions:

1. Checkout OctoPrint:  ```$ git clone https://github.com/foosel/OctoPrint.git```
2. Change into the OctoPrint folder: ```$ cd OctoPrint```
3. Request pip to run the setup file, i.e install all the necessary dependencies: ```$ sudo pip2.7 install .```
4. After installing OctoPrint, you should run it using: ```$ octoprint serve```   
5. Now you should verify your installation opening a web browser and going to http://localhost:5000
6. If everything went right you should see the OctoPrint home screen  

PS: In case you encountered a conflict error with PyYAML on step ```3.``` make sure you remove the previous installation from PyYAML from your computer by doing: 

 ```$ sudo rm -rf /usr/local/lib/python2.7/dist-packages/yaml ```

And then calling the setup file again to install all the dependencies but ignoring PyYAML (this is necessary because PyYAML is usually not completely deleted.)

 ```$ sudo pip2.7 install . --ignore-installed ${PyYAML} ``` 

By entering the OctoPrint home screen for the first time you should setup your 3D Printer. Make sure to include the baudrate from your 3D Printer and enable the API Key (respectively copying it).
In case the printer you're using is a MakerBot, then you need to install the GPX plug-in, which will enable us to send gcode to octoprint, which will take care of the conversion to x3g. In order to do that do the following:
1. In the octoPrint home screen go to configurations/Plugin manager/Get More
2. Search for GPX and click install 
3. After GPX is installed, make sure to check in the plugin's list if the GPX plugin is enabled
4. Choose your machine, gcode flavor and other settings

PS: In case you encountered the error that the octoprint could not open your port, do the following:

 ```$ sudo chmod +r /dev/ttyACM0 ``` 

## Installing

This project should be run from source, to do so just go to your ROS workspace (supposing is called ``` catkin_ws ```):

``` $ cd ~/catkin_ws/src ```

And clone it with: 

``` $ git clone https://github.com/ielson/octoROS.git```

Tether octoROS with your catkin workspace:

``` $ cd ~/catkin_ws ```

``` $ catkin_make ```

After that you can't forget to make ``` octoROS.py ``` executable, otherwise ROS won't find it.

## Usage
To use it, you need to have your octoprint server running, have a model uploaded to it, changing the file name in the 59th line to the uploaded file name and then run the octoROS.py file. 

So it will start printing the model and outputting the progress and some printer measurements to the ```/printer3d``` ROS topic until the printing is finished. When the printing is done, a flag will be set, i.e a boolean will be sent to ```printer3d/finishedPrinting```.

``` $ roslaunch octoROS connect_to_printer.launch ```

## Authors

* **Daniel Mascarenhas** - *Initial work* - [ielson](https://github.com/ielson)
* **Henrique Poleselo**

See also the list of [contributors](https://github.com/ielson/octoROS/contributors) who participated in this project.

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

Many thanks to the octoprint team, that made this awesome software
