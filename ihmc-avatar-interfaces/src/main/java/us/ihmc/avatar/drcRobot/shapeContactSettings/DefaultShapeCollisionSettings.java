package us.ihmc.avatar.drcRobot.shapeContactSettings;

public class DefaultShapeCollisionSettings implements DRCRobotModelShapeCollisionSettings
{
   @Override
   public boolean useShapeCollision()
   {
      return false;
   }
}
