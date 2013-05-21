package us.ihmc.darpaRoboticsChallenge.networkProcessor.compression;

public class CompressionSettings
{
   public final int FOOTSTEP_X_BITS = 9;
   public final int FOOTSTEP_Y_BITS = 9;
   public final int FOOTSTEP_Z_BITS = 14;
   
   public final boolean SEND_FOOTSTEP_ORIENTATION_QUATERNION = true;
   public final int FOOTSTEP_QUATERNION_ELEMENT_BITS = 8;
   
   public final int FOOTSTEP_YAW_BITS = 8;
   public final int FOOTSTEP_PITCH_BITS = 8;
   public final int FOOTSTEP_ROLL_BITS = 8;
   
   public final double MAX_FOOTSTEP_LENGTH = 1.0;
   public final double MAX_FOOTSTEP_HEIGHT = 0.5;
   
   
   public final int[] FOOTSTEP_POSITION_BITS = {  FOOTSTEP_X_BITS, FOOTSTEP_Y_BITS, FOOTSTEP_Z_BITS };
   public final int[] FOOTSTEP_ORIENTATION_BITS = { FOOTSTEP_YAW_BITS, FOOTSTEP_PITCH_BITS, FOOTSTEP_ROLL_BITS };
}
