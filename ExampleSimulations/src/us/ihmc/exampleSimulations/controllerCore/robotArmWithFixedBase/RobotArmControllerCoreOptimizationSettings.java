package us.ihmc.exampleSimulations.controllerCore.robotArmWithFixedBase;

import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.euclid.tuple2D.Vector2D;

public class RobotArmControllerCoreOptimizationSettings implements ControllerCoreOptimizationSettings
{
   private double jointAccelerationWeight = 0.005;
   private double jointJerkWeight = 0.1;

   @Override
   public double getJointAccelerationWeight()
   {
      return jointAccelerationWeight;
   }

   @Override
   public double getJointJerkWeight()
   {
      return jointJerkWeight;
   }

   @Override
   public double getRhoWeight()
   {
      return 0;
   }

   @Override
   public double getRhoMin()
   {
      return 0;
   }

   @Override
   public double getRhoRateDefaultWeight()
   {
      return 0;
   }

   @Override
   public double getRhoRateHighWeight()
   {
      return 0;
   }

   @Override
   public Vector2D getCoPWeight()
   {
      return new Vector2D();
   }

   @Override
   public Vector2D getCoPRateDefaultWeight()
   {
      return new Vector2D();
   }

   @Override
   public Vector2D getCoPRateHighWeight()
   {
      return new Vector2D();
   }

   @Override
   public int getNumberOfBasisVectorsPerContactPoint()
   {
      return 0;
   }

   @Override
   public int getNumberOfContactPointsPerContactableBody()
   {
      return 0;
   }

   @Override
   public int getNumberOfContactableBodies()
   {
      return 0;
   }

   @Override
   public int getRhoSize()
   {
      return 0;
   }
}
