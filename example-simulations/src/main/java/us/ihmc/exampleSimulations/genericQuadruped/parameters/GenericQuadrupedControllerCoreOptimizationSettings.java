package us.ihmc.exampleSimulations.genericQuadruped.parameters;

import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.euclid.tuple2D.Vector2D;

public class GenericQuadrupedControllerCoreOptimizationSettings implements ControllerCoreOptimizationSettings
{
   private static final double jointAccelerationWeight = 0.005;
   private static final double jointJerkWeight = 1.0E-7;

   private static final double defaultRhoWeight = 0.00001;
   private static final double defaultRhoMin = 1.0;
   private static final double defaultRhoRateDefaultWeight = 3.0E-9;
   private static final double defaultRhoRateHighWeight = 5.0E-8;

   private final double rhoWeight;
   private final double rhoMin;
   private final double rhoRateDefaultWeight;
   private final double rhoRateHighWeight;

   private static final Vector2D copWeight = new Vector2D(0.0, 0.0);
   private static final Vector2D copRateDefaultWeight = new Vector2D();
   private static final Vector2D copRateHighWeight = new Vector2D();

   private final int nBasisVectorsPerContactPoint = 4;
   private final int nContactPointsPerContactableBody = 1;
   private final int nContactableBodies;

   public GenericQuadrupedControllerCoreOptimizationSettings(double totalMass)
   {
      this(totalMass, 4);
   }

   public GenericQuadrupedControllerCoreOptimizationSettings(double totalMass, int numberOfContactableBodies)
   {
      rhoWeight = defaultRhoWeight;// * totalMass;
      rhoMin = defaultRhoMin;// * totalMass;
      rhoRateDefaultWeight = defaultRhoRateDefaultWeight;// * totalMass;
      rhoRateHighWeight = defaultRhoRateHighWeight;// * totalMass;

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
