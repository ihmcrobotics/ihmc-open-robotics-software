package us.ihmc.sensorProcessing.model;


public enum RobotMotionStatus
{
   STANDING(3), IN_MOTION(4);

   public int behaviorId;
   private RobotMotionStatus(int behaviorId)
   {
      this.behaviorId = behaviorId;
   }
   
   public int getBehaviorId()
   {
      return behaviorId;
   }
}
