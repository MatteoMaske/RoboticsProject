/*usage: change the path of the output file on line 85 with your own locosim path and have fun*/

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <algorithm>

using namespace std;

#define BLOCKS_NUMBER 11

int main(){

    //reset the random seed
    srand(time(NULL));

    const vector<string> megaBlockNamesStl= {"X1-Y1-Z2.stl","X1-Y2-Z1.stl","X1-Y2-Z2-CHAMFER.stl", "X1-Y2-Z2.stl","X1-Y2-Z2-TWINFILLET.stl","X1-Y3-Z2-FILLET.stl","X1-Y3-Z2.stl", "X1-Y4-Z1.stl","X1-Y4-Z2.stl","X2-Y2-Z2-FILLET.stl","X2-Y2-Z2.stl" };
    const vector<string> megaBlockNames= {"X1-Y2-Z1","X1-Y1-Z2","X1-Y2-Z2-CHAMFER","X1-Y2-Z2","X1-Y2-Z2-TWINFILLET","X1-Y3-Z2-FILLET","X1-Y3-Z2","X1-Y4-Z1","X1-Y4-Z2","X2-Y2-Z2-FILLET","X2-Y2-Z2"};
    int mode;

    do{
        cout << "Choose the megaBlock you want to spawn in the world:" << endl;
        cout << "[0 for standard position, 1 for rotated position]" << endl;
        cin >> mode;
        
    }while(mode != 0 && mode != 1);

    int blockNumber;

    do{
        cout << "Choose number of megablocks to be spawned [0<x<12]" << endl;
        cin >> blockNumber;
        
    }while(blockNumber <= 0 || blockNumber >= 12);


    //generate a file named "customWorld.world" with the following content:
// <?xml version="1.0" ?>
// <sdf version="1.4">
//   <world name="default">
//     <physics type='ode'>
//       <gravity>0 0 -9.81</gravity>
//       <!-- max step size has to be a multiple of the desired task rate-->
//       <max_step_size>0.001</max_step_size> 
//       <real_time_factor>1</real_time_factor>
//     </physics>
//     <!-- A global light source -->
//     <include>
//       <uri>model://sun</uri>
//     </include>
//     <!-- A ground plane -->
//     <include>
//       <uri>model://ground_plane</uri>
//     </include>
//     <include>
//       <name>tavolo</name>
//       <uri>model://tavolo</uri>
//       <pose>0.0 0.0 0.0 0 0 0</pose>
//     </include>

//     <!-- lego block -->
//     <include>
//       <name>X1-Y1-Z2</name>
//       <uri>model://X1-Y1-Z2</uri>
//       <!-- Spawn at the left of the table-->
//       <pose>0.8 0.45 0.9 0 0 0</pose>
//     </include>

//      <gui>
//     <camera name="gzclient_camera">
//       <pose>1. 3.2 2.2 0. 0.4 -1.75</pose>
//     </camera>
//     </gui>

//   </world>
// </sdf>



    //create an output file in /home/matteo/trento_lab_home/ros_ws/src/locosim/ros_impedance_controller/worlds
    ofstream outfile;
    outfile.open("../../locosim/ros_impedance_controller/worlds/customWorld.world");
    
    //write the first part of the file
    outfile << "<?xml version=\"1.0\" ?>" << endl;
    outfile << "<sdf version=\"1.4\">" << endl;
    outfile << "  <world name=\"default\">" << endl;
    outfile << "    <physics type='ode'>" << endl;
    outfile << "      <gravity>0 0 -9.81</gravity>" << endl;
    outfile << "      <!-- max step size has to be a multiple of the desired task rate-->" << endl;
    outfile << "      <max_step_size>0.001</max_step_size> " << endl;
    outfile << "      <real_time_factor>1</real_time_factor>" << endl;
    outfile << "    </physics>" << endl;
    outfile << "    <include>" << endl;
    outfile << "      <uri>model://sun</uri>" << endl;
    outfile << "    </include>" << endl;
    outfile << "    <include>" << endl;
    outfile << "      <uri>model://ground_plane</uri>" << endl;
    outfile << "    </include>" << endl;
    outfile << "    <include>" << endl;
    outfile << "      <name>tavolo</name>" << endl;
    outfile << "      <uri>model://tavolo</uri>" << endl;
    outfile << "      <pose>0.0 0.0 0.0 0 0 0</pose>" << endl;
    outfile << "    </include>" << endl;

    //write the second part of the file

    //generate a vector with all number from 0 to 10
    vector<int> megaBlockNumbers;
    for (int i=0; i<BLOCKS_NUMBER; i++){
        megaBlockNumbers.push_back(i);
    }
    //shuffle the vector
    random_shuffle(megaBlockNumbers.begin(), megaBlockNumbers.end());

    //for each megaBlock in the vector we choose a random float for pose x and y and we write the corresponding model in the file
    for (int i=0; i<blockNumber; i++){
        //generate a random integer between 0 and 10
        int randomBlock = megaBlockNumbers[i];
        //define x as a random float between 0.1 and 0.9        
        float x = (float)rand()/(float)RAND_MAX;
        x = 0.1 + x*0.8;
        //define y as a float between 0.1 and 0.8
        float y = (float)rand()/(float)RAND_MAX;
        y = 0.1 + y*0.7;
        //define rotx, roty and rotz as a float between 0 and M_PI
        float rotx = (float)rand()/(float)RAND_MAX*M_PI;
        float roty = (float)rand()/(float)RAND_MAX*M_PI;
        float rotz = (float)rand()/(float)RAND_MAX*M_PI;
        outfile << "    <include>" << endl;
        outfile << "      <name>" << megaBlockNames[randomBlock] << "</name>" << endl;
        outfile << "      <uri>model://" << megaBlockNames[randomBlock] << "</uri>" << endl;
        if(mode == 0){
            outfile << "      <pose>" << x << " " << y << " 0.875 0 0 0</pose>" << endl; //to generate block not rotated
        }else{
            outfile << "      <pose>" << x << " " << y << " 1 " << rotx << " " << roty << " " << rotz << "</pose>" << endl; //to generate block rotated randomly
        }
        outfile << "    </include>" << endl;
    }

    //write the third part of the file
    outfile << "    <gui>" << endl;
    outfile << "    <camera name=\"gzclient_camera\">" << endl;
    outfile << "      <pose>1. 3.2 2.2 0. 0.4 -1.75</pose>" << endl;
    outfile << "    </camera>" << endl;
    outfile << "    </gui>" << endl;

    outfile << "  </world>" << endl;
    outfile << "</sdf>" << endl;



    return 0;
}