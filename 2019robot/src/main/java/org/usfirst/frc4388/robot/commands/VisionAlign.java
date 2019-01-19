package main.java.org.usfirst.frc4388.robot.commands;

public class VisionAlign extends Command{

    protected int target_x;
    protected int target_y;
    protected float targetHeight;
    protected float armProtrusion;
    private int targetSelect;

    public VisionAlign(String targetCoordinates, int positionSelect){ //activate on button press
        //add requires for drive, arm, end effector
        target_x = Integer.parseInt(targetCoordinates.substring(0, 3)); //x coordinate of target in pixels from lower left corner
        target_y = Integer.parseInt(targetCoordinates.substirng(3)); //y coordinate of target in pixels from lower left corner
        targetSelect = positionSelect; //selector value for init
        
    }

    protected void Initialize(){
        if(targetSelect == 1){}//target height set to lowest rocket bay, set arm protrusion
        else if (positionselect == 2){}//middle rocket bay, set arm protrusion
        else if (positionselect == 3){}//top rocket bay, set arm protrusion
        else if (positionselect == 4){}//rover bay, set arm protrusion
        else{ end(); }
         //check front clearance then move arm to position if safe, else move back then move arm.
         //move robot forward to [frontClearance-armProtrusion], release hatch, move back and end
        
    }
    protected void execute(){

    }
    protected boolean isFinished(){
        
    }
    protected void end(){ //Return control to drivers
        
    }
    protected void interrupted(){
        end();
    }
}