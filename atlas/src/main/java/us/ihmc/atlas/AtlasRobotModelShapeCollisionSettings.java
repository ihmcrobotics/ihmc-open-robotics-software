package us.ihmc.atlas;

import us.ihmc.avatar.drcRobot.shapeContactSettings.DRCRobotModelShapeCollisionSettings;

public class AtlasRobotModelShapeCollisionSettings implements DRCRobotModelShapeCollisionSettings
{
   private final boolean useShapeCollision;
   
   public AtlasRobotModelShapeCollisionSettings(boolean useShapeCollision)
   {
      this.useShapeCollision = useShapeCollision;
   }
   
   @Override
   public boolean useShapeCollision()
   {
      return useShapeCollision;
   }
   
   @Override
   public boolean useHybridImpulseHandler()
   {
      return true;
   }
}
