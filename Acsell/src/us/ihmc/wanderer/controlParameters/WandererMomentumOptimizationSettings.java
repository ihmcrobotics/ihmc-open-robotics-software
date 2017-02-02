package us.ihmc.wanderer.controlParameters;

import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;

public class WandererMomentumOptimizationSettings extends MomentumOptimizationSettings
{
   private final Vector3d linearMomentumWeight = new Vector3d(0.05, 0.05, 0.01);
   private final Vector3d highLinearMomentumWeightForRecovery = new Vector3d(0.5, 0.5, 0.05);
   private final Vector3d angularMomentumWeight = new Vector3d(0.0, 0.0, 0.0);

   private final Vector3d defaultAngularFootWeight = new Vector3d(0.5, 0.5, 0.5);
   private final Vector3d defaultLinearFootWeight = new Vector3d(30.0, 30.0, 30.0);
   private final Vector3d highAngularFootWeight = new Vector3d(5.0, 5.0, 5.0);
   private final Vector3d highLinearFootWeight = new Vector3d(50.0, 50.0, 50.0);

   private final Vector3d chestAngularWeight = new Vector3d(15.0, 10.0, 5.0);
   private final double spineJointspaceWeight = 1.0;
   private final Vector3d pelvisAngularWeight = new Vector3d(5.0, 5.0, 5.0);

   private final int nBasisVectorsPerContactPoint = 4;
   private final int nContactPointsPerContactableBody = 4;
   private final int nContactableBodies = 2;

   private final double jointAccelerationWeight = 0.005;
   private final double jointJerkWeight = 0.1;
   private final double rhoWeight = 0.00001;
   private final double rhoMin = 4.0;
   private final double rhoRateDefaultWeight = 0.002;
   private final double rhoRateHighWeight = 0.05;
   private final Vector2d copWeight = new Vector2d(100.0, 200.0);
   private final Vector2d copRateDefaultWeight = new Vector2d(20000.0, 20000.0);
   private final Vector2d copRateHighWeight = new Vector2d(2500000.0, 10000000.0);
   private final double headJointspaceWeight = 1.0;
   private final double headTaskspaceWeight = 1.0;
   private final double headUserModeWeight = 1.0;
   private final double handUserModeWeight = 50.0;
   private final double handJointspaceWeight = 1.0;
   private final Vector3d handAngularTaskspaceWeight = new Vector3d(1.0, 1.0, 1.0);
   private final Vector3d handLinearTaskspaceWeight = new Vector3d(1.0, 1.0, 1.0);

   /** @inheritDoc */
   @Override
   public Vector3d getLinearMomentumWeight()
   {
      return linearMomentumWeight;
   }

   /** @inheritDoc */
   @Override
   public Vector3d getHighLinearMomentumWeightForRecovery()
   {
      return highLinearMomentumWeightForRecovery;
   }

   /** @inheritDoc */
   @Override
   public Vector3d getAngularMomentumWeight()
   {
      return angularMomentumWeight;
   }

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
   public Vector2d getCoPWeight()
   {
      return copWeight;
   }

   /** @inheritDoc */
   @Override
   public Vector2d getCoPRateDefaultWeight()
   {
      return copRateDefaultWeight;
   }

   /** @inheritDoc */
   @Override
   public Vector2d getCoPRateHighWeight()
   {
      return copRateHighWeight;
   }

   /** @inheritDoc */
   @Override
   public double getHeadUserModeWeight()
   {
      return headUserModeWeight;
   }

   /** @inheritDoc */
   @Override
   public double getHeadJointspaceWeight()
   {
      return headJointspaceWeight;
   }

   /** @inheritDoc */
   @Override
   public double getHeadTaskspaceWeight()
   {
      return headTaskspaceWeight;
   }

   /** @inheritDoc */
   @Override
   public Vector3d getChestAngularWeight()
   {
      return chestAngularWeight;
   }

   /** @inheritDoc */
   @Override
   public Vector3d getPelvisAngularWeight()
   {
      return pelvisAngularWeight;
   }

   /** @inheritDoc */
   @Override
   public Vector3d getDefaultLinearFootWeight()
   {
      return defaultLinearFootWeight;
   }

   /** @inheritDoc */
   @Override
   public Vector3d getDefaultAngularFootWeight()
   {
      return defaultAngularFootWeight;
   }

   /** @inheritDoc */
   @Override
   public Vector3d getHighLinearFootWeight()
   {
      return highLinearFootWeight;
   }

   /** @inheritDoc */
   @Override
   public Vector3d getHighAngularFootWeight()
   {
      return highAngularFootWeight;
   }

   /** @inheritDoc */
   @Override
   public double getHandUserModeWeight()
   {
      return handUserModeWeight;
   }

   /** @inheritDoc */
   @Override
   public double getHandJointspaceWeight()
   {
      return handJointspaceWeight;
   }

   /** @inheritDoc */
   @Override
   public Vector3d getHandAngularTaskspaceWeight()
   {
      return handAngularTaskspaceWeight;
   }

   /** @inheritDoc */
   @Override
   public Vector3d getHandLinearTaskspaceWeight()
   {
      return handLinearTaskspaceWeight;
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

   /** @inheritDoc */
   @Override
   public int getRhoSize()
   {
      return  nContactableBodies * nContactPointsPerContactableBody * nBasisVectorsPerContactPoint;
   }

   /** @inheritDoc */
   @Override
   public double getSpineJointspaceWeight()
   {
      return spineJointspaceWeight;
   }
}
