package us.ihmc.sensorProcessing.outputData;

public enum LowLevelActuatorMode
{
   DISABLED,
   EFFORT,
   VELOCITY,
   POSITION;

   public static LowLevelActuatorMode[] values = values();
}
