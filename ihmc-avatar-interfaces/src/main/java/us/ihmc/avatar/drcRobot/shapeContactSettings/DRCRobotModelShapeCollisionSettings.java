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
      return 0.01;
   }

   default double getFrictionCoefficient()
   {
      return 1.0;
   }

   default double getHybridSpringCoefficient()
   {
      return 10000;
   }

   default double getHybridDamperCoefficient()
   {
      return 1000;
   }
}
