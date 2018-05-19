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
   
   public static final BDIRobotBehavior[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static BDIRobotBehavior fromByte(byte enumAsByte)
   {
      if (enumAsByte == -1)
         return null;
      return values[enumAsByte];
   }
}
