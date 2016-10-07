#! /bin/bash
DATA_BAG_PATH=/home/lci/workspace/odometry/data_nn_pwc/bagfiles
SOURCEFILE_PATH=/home/lci/workspace/odometry/data_nn_pwc/src
NeuralNet_DATA_PATH=/home/lci/workspace/odometry/data_nn_pwc/src/correct_rnn/data/freq16
NeuralNet_SOURCE_PATH=/home/lci/workspace/odometry/data_nn_pwc/src/correct_rnn


rostopic echo -b $DATA_BAG_PATH/joyAndVOCorridor_2016-03-16-12-21-09.bag /chair_joy > $DATA_BAG_PATH/chairJoy1.txt

rostopic echo -b $DATA_BAG_PATH/joyAndVOCorridor_2016-03-16-12-21-09.bag /kinect_odometer/odometry > $DATA_BAG_PATH/visualOdometry1.txt


rostopic echo -b $DATA_BAG_PATH/joyAndVOElevator_2016-03-16-12-35-33.bag /chair_joy > $DATA_BAG_PATH/chairJoy2.txt

rostopic echo -b $DATA_BAG_PATH/joyAndVOElevator_2016-03-16-12-35-33.bag /kinect_odometer/odometry > $DATA_BAG_PATH/visualOdometry2.txt


rostopic echo -b $DATA_BAG_PATH/joyAndVOlab_2016-03-17-15-14-04.bag /chair_joy > $DATA_BAG_PATH/chairJoy3.txt

rostopic echo -b $DATA_BAG_PATH/joyAndVOlab_2016-03-17-15-14-04.bag /kinect_odometer/odometry > $DATA_BAG_PATH/visualOdometry3.txt


rostopic echo -b $DATA_BAG_PATH/joyAndVOlabCorridor2March18_2016-03-18-20-50-26.bag /chair_joy > $DATA_BAG_PATH/chairJoy4.txt

rostopic echo -b $DATA_BAG_PATH/joyAndVOlabCorridor2March18_2016-03-18-20-50-26.bag /kinect_odometer/odometry > $DATA_BAG_PATH/visualOdometry4.txt


rostopic echo -b $DATA_BAG_PATH/joyAndVOlabCorridorMarch18_2016-03-18-20-31-29.bag /chair_joy > $DATA_BAG_PATH/chairJoy5.txt

rostopic echo -b $DATA_BAG_PATH/joyAndVOlabCorridorMarch18_2016-03-18-20-31-29.bag /kinect_odometer/odometry > $DATA_BAG_PATH/visualOdometry5.txt


g++ -std=c++11 $SOURCEFILE_PATH/JoystickAndVO.cpp -o $SOURCEFILE_PATH/JoystickAndVO


cd $SOURCEFILE_PATH

./JoystickAndVO $DATA_BAG_PATH/visualOdometry1.txt  $DATA_BAG_PATH/chairJoy1.txt  $NeuralNet_DATA_PATH/seq1.txt   

./JoystickAndVO $DATA_BAG_PATH/visualOdometry2.txt  $DATA_BAG_PATH/chairJoy2.txt  $NeuralNet_DATA_PATH/seq2.txt   

./JoystickAndVO $DATA_BAG_PATH/visualOdometry3.txt  $DATA_BAG_PATH/chairJoy3.txt   $NeuralNet_DATA_PATH/seq3.txt  

./JoystickAndVO $DATA_BAG_PATH/visualOdometry4.txt  $DATA_BAG_PATH/chairJoy4.txt   $NeuralNet_DATA_PATH/seq4.txt  

./JoystickAndVO $DATA_BAG_PATH/visualOdometry5.txt  $DATA_BAG_PATH/chairJoy5.txt  $NeuralNet_DATA_PATH/seq5.txt 

cd $NeuralNet_SOURCE_PATH

th main.lua -a  $NeuralNet_DATA_PATH/ -m nn -e 10


