package us.ihmc.avatar.drcRobot.shapeContactSettings;

public interface DRCRobotModelShapeCollisionSettings
{
   default boolean useShapeCollision()
   {
      return false;
   }

   default boolean useHybridImpulseHandler()
   {
      return true;
   }

   default double getRestitutionCoefficient()
   {
      return 0.0;
   }

   default double getFrictionCoefficient()
   {
      return 0.9;
   }

   default double getHybridSpringCoefficient()
   {
      return 100000.0;
   }

   default double getHybridDamperCoefficient()
   {
      return 500.0;
   }
}
