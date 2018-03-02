package us.ihmc.exampleSimulations.genericQuadruped.parameters;

import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.euclid.tuple2D.Vector2D;

public class GenericQuadrupedControllerCoreOptimizationSettings implements ControllerCoreOptimizationSettings
{
   @Override
   public double getJointAccelerationWeight()
   {
      return 0;
   }

   @Override
   public double getJointJerkWeight()
   {
      return 0;
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
      return null;
   }

   @Override
   public Vector2D getCoPRateDefaultWeight()
   {
      return null;
   }

   @Override
   public Vector2D getCoPRateHighWeight()
   {
      return null;
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
