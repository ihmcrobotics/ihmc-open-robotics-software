package us.ihmc.humanoidRobotics.communication.packets.bdi;

public enum BDIRobotBehavior
{
   NONE,        
   FREEZE,      
   STAND_PREP,  
   STAND,       
   WALK,        
   STEP,        
   MANIPULATE,  
   USER,        
   CALIBRATE,
   SOFT_STOP;
   
   public static BDIRobotBehavior[] values = values();
}
