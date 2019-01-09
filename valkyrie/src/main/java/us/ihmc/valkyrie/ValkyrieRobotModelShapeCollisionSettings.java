package us.ihmc.valkyrie;

import us.ihmc.avatar.drcRobot.shapeContactSettings.DRCRobotModelShapeCollisionSettings;

public class ValkyrieRobotModelShapeCollisionSettings implements DRCRobotModelShapeCollisionSettings
{
   private final boolean useShapeCollision;
   
   public ValkyrieRobotModelShapeCollisionSettings(boolean useShapeCollision)
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