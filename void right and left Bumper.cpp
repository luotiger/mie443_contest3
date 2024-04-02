void rightBumper() {
    //the rightBumper function turns the robot right by 90 degrees if the right bumper is hit
    //make a surprised cat sound 
    //Return to its original position 
    //returns state 0 to move forward
    if(get_time_elapsed()<1000){
        turnSpd=0.6;
        linear=0;
        return;
    } 
    if(get_time_elapsed()>1000 && get_time_elapsed()<2000 ){
        turnSpd=-0.6;
        linear=0;
        return;

    }else {
        state=0;
        return;
    }
}

void leftBumper() {
    //the rightBumper function turns the robot left by 90 degrees if the right bumper is hit
    //make a surprised cat sound 
    //Return to its original position 
    //returns state 0 to move forward
    if(get_time_elapsed()<1000){
        turnSpd=-0.6;
        linear=0;
        return;
    } 
    if(get_time_elapsed()>1000 && get_time_elapsed()<2000 ){
        turnSpd=0.6;
        linear=0;
        return;

    }else {
        state=0;
        return; 
    }
}