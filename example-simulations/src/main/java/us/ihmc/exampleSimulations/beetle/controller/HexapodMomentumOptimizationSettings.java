package us.ihmc.exampleSimulations.beetle.controller;

import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.euclid.tuple2D.Vector2D;

public class HexapodMomentumOptimizationSettings implements ControllerCoreOptimizationSettings
{
   private final int nBasisVectorsPerContactPoint = 4;
   private final int nContactPointsPerContactableBody = 1;
   private final int nContactableBodies = 6;

   private final double jointAccelerationWeight = 0.005;
   private final double jointJerkWeight = 0.1 * 0.001 * 0.001;
   private final double rhoWeight = 0.00001;
   private final double rhoMin = 4.0;
   private final double rhoRateDefaultWeight = 0.002 * 0.001 * 0.001; // 0.005
   private final double rhoRateHighWeight = 0.05 * 0.001 * 0.001;
   private final Vector2D copWeight = new Vector2D(100.0, 200.0); //750.0, 1500.0);
   private final Vector2D copRateDefaultWeight = new Vector2D(20000.0 * 0.001 * 0.001, 20000.0 * 0.001 * 0.001); //100000.0, 200000.0);
   private final Vector2D copRateHighWeight = new Vector2D(2500000.0 * 0.001 * 0.001, 10000000.0 * 0.001 * 0.001);

   /** @inheritDoc */
   @Override
   public double getJointAccelerationWeight()
   {
      return jointAccelerationWeight;
   }

   /** @inheritDoc */
   @Override
   public double getJointJerkWeight()
   {
      return jointJerkWeight;
   }

   /** @inheritDoc */
   @Override
   public double getRhoWeight()
   {
      return rhoWeight;
   }

   /** @inheritDoc */
   @Override
   public double getRhoMin()
   {
      return rhoMin;
   }

   /** @inheritDoc */
   @Override
   public double getRhoRateDefaultWeight()
   {
      return rhoRateDefaultWeight;
   }

   /** @inheritDoc */
   @Override
   public double getRhoRateHighWeight()
   {
      return rhoRateHighWeight;
   }

   /** @inheritDoc */
   @Override
   public Vector2D getCoPWeight()
   {
      return copWeight;
   }

   /** @inheritDoc */
   @Override
   public Vector2D getCoPRateDefaultWeight()
   {
      return copRateDefaultWeight;
   }

   /** @inheritDoc */
   @Override
   public Vector2D getCoPRateHighWeight()
   {
      return copRateHighWeight;
   }

   /** @inheritDoc */
   @Override
   public int getNumberOfBasisVectorsPerContactPoint()
   {
      return nBasisVectorsPerContactPoint;
   }

   /** @inheritDoc */
   @Override
   public int getNumberOfContactPointsPerContactableBody()
   {
      return nContactPointsPerContactableBody;
   }

   /** @inheritDoc */
   @Override
   public int getNumberOfContactableBodies()
   {
      return nContactableBodies;
   }
}
