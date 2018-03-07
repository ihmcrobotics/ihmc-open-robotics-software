package us.ihmc.exampleSimulations.genericQuadruped.parameters;

import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.euclid.tuple2D.Vector2D;

public class GenericQuadrupedControllerCoreOptimizationSettings implements ControllerCoreOptimizationSettings
{
   private static final double jointAccelerationWeight = 0.005;
   private static final double jointJerkWeight = 0.1;

   private static final double defaultRhoWeight = 5e-3;
   private static final double defaultRhoMin = 0.1;
   private static final double defaultRhoRateDefaultWeight = 1e-5;
   private static final double defaultRhoRateHighWeight = 3e-3;

   private final double rhoWeight;
   private final double rhoMin;
   private final double rhoRateDefaultWeight;
   private final double rhoRateHighWeight;

   private static final Vector2D copWeight = new Vector2D(100.0, 200.0);
   private static final Vector2D copRateDefaultWeight = new Vector2D(20000.0, 20000.0);
   private static final Vector2D copRateHighWeight = new Vector2D(2500000.0, 10000000.0);

   private final int nBasisVectorsPerContactPoint = 4;
   private final int nContactPointsPerContactableBody = 1;
   private final int nContactableBodies;

   public GenericQuadrupedControllerCoreOptimizationSettings(double totalMass)
   {
      this(totalMass, 4);
   }

   public GenericQuadrupedControllerCoreOptimizationSettings(double totalMass, int numberOfContactableBodies)
   {
      rhoWeight = defaultRhoWeight * totalMass;
      rhoMin = defaultRhoMin * totalMass;
      rhoRateDefaultWeight = defaultRhoRateDefaultWeight * totalMass;
      rhoRateHighWeight = defaultRhoRateHighWeight * totalMass;

      this.nContactableBodies = numberOfContactableBodies;
   }

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
      return rhoWeight;
   }

   @Override
   public double getRhoMin()
   {
      return rhoMin;
   }

   @Override
   public double getRhoRateDefaultWeight()
   {
      return rhoRateDefaultWeight;
   }

   @Override
   public double getRhoRateHighWeight()
   {
      return rhoRateHighWeight;
   }

   @Override
   public Vector2D getCoPWeight()
   {
      return copWeight;
   }

   @Override
   public Vector2D getCoPRateDefaultWeight()
   {
      return copRateDefaultWeight;
   }

   @Override
   public Vector2D getCoPRateHighWeight()
   {
      return copRateHighWeight;
   }

   @Override
   public int getNumberOfBasisVectorsPerContactPoint()
   {
      return nBasisVectorsPerContactPoint;
   }

   @Override
   public int getNumberOfContactPointsPerContactableBody()
   {
      return nContactPointsPerContactableBody;
   }

   @Override
   public int getNumberOfContactableBodies()
   {
      return nContactableBodies;
   }

   @Override
   public int getRhoSize()
   {
      return nContactableBodies * nContactPointsPerContactableBody * nBasisVectorsPerContactPoint;
   }
}
