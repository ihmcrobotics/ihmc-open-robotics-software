package us.ihmc.sensorProcessing.model;

import us.ihmc.tools.DocumentedEnum;

public enum RobotMotionStatus implements DocumentedEnum<RobotMotionStatus>
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

   @Override
   public String getDocumentation(RobotMotionStatus var)
   {
      switch (var)
      {
      case STANDING:
         return "Robot is standing";
      case IN_MOTION:
         return "Robot is in motion";
      }

      return "Undocumented value";
   }

   @Override
   public RobotMotionStatus[] getDocumentedValues()
   {
      return values();
   }
}
