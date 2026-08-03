package us.ihmc.sensorProcessing.model;

public enum RobotMotionStatus
{
   UNKNOWN(2),
   STANDING(3),
   IN_MOTION(4);

   public int behaviorId;

   private RobotMotionStatus(int behaviorId)
   {
      this.behaviorId = behaviorId;
   }

   public int getBehaviorId()
   {
      return behaviorId;
   }

   public static RobotMotionStatus[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static RobotMotionStatus fromByte(byte enumAsByte)
   {
      return values[enumAsByte];
   }
}
