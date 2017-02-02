package us.ihmc.exampleSimulations.beetle.controller;

import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;

public class HexapodMomentumOptimizationSettings extends MomentumOptimizationSettings
{
   private final Vector3d linearMomentumWeight = new Vector3d(0.05, 0.05, 0.01);
   private final Vector3d highLinearMomentumWeightForRecovery = new Vector3d(0.5, 0.5, 0.05);
   private final Vector3d angularMomentumWeight = new Vector3d(0.0, 0.0, 0.0);

   private Vector3d defaultAngularFootWeight = new Vector3d(0.5, 0.5, 0.5);
   private Vector3d defaultLinearFootWeight = new Vector3d(30.0, 30.0, 30.0);
   private Vector3d highAngularFootWeight = new Vector3d(5.0, 5.0, 5.0);
   private Vector3d highLinearFootWeight = new Vector3d(50.0, 50.0, 50.0);

   private Vector3d chestAngularWeight = new Vector3d(15.0, 10.0, 5.0);
   private double spineJointspaceWeight = 1.0;
   private Vector3d pelvisAngularWeight = new Vector3d(5.0, 5.0, 5.0);

   private int nBasisVectorsPerContactPoint = 4;
   private int nContactPointsPerContactableBody = 4;
   private int nContactableBodies = 2;

   private double jointAccelerationWeight = 0.005;
   private double jointJerkWeight = 0.1;
   private double rhoWeight = 0.00001;
   private double rhoMin = 4.0;
   // Be careful with that guy, even 0.005 seems to make the ICP control sluggish.
   private double rhoRateDefaultWeight = 0.002; // 0.005
   private double rhoRateHighWeight = 0.05;
   private final Vector2d copWeight = new Vector2d(100.0, 200.0); //750.0, 1500.0);
   private final Vector2d copRateDefaultWeight = new Vector2d(20000.0, 20000.0); //100000.0, 200000.0);
   private final Vector2d copRateHighWeight = new Vector2d(2500000.0, 10000000.0);
   private double headJointspaceWeight = 1.0;
   private double headTaskspaceWeight = 1.0;
   private double headUserModeWeight = 1.0;
   private double handUserModeWeight = 50.0;
   private double handJointspaceWeight = 1.0;
   private Vector3d handAngularTaskspaceWeight = new Vector3d(1.0, 1.0, 1.0);
   private Vector3d handLinearTaskspaceWeight = new Vector3d(1.0, 1.0, 1.0);

   /** @inheritDoc */
   @Override
   public void scaleForceWeights(double scale)
   {
      rhoWeight /= scale;
      rhoMin *= scale;
      rhoRateDefaultWeight /= scale * scale;
      rhoRateHighWeight /= scale * scale;
      linearMomentumWeight.scale(1.0 / scale);
      highLinearMomentumWeightForRecovery.scale(1.0 / scale);
      angularMomentumWeight.scale(1.0 / scale);
   }

   /** @inheritDoc */
   @Override
   public void setHeadWeights(double jointspace, double taskspace, double userMode)
   {
      headJointspaceWeight = jointspace;
      headTaskspaceWeight = taskspace;
      headUserModeWeight = userMode;
   }

   /** @inheritDoc */
   @Override
   public void setBodyWeights(double chestWeight, double pelvisWeight)
   {
      chestAngularWeight.set(chestWeight, chestWeight, chestWeight);
      pelvisAngularWeight.set(pelvisWeight, pelvisWeight, pelvisWeight);
   }

   /** @inheritDoc */
   @Override
   public void setBodyWeights(Vector3d chestAngularWeight, Vector3d pelvisAngularWeight)
   {
      this.chestAngularWeight.set(chestAngularWeight);
      this.pelvisAngularWeight.set(pelvisAngularWeight);
   }

   /** @inheritDoc */
   @Override
   public void setHandTaskspaceControlWeights(Vector3d angular, Vector3d linear)
   {
      handAngularTaskspaceWeight.set(angular);
      handLinearTaskspaceWeight.set(linear);
   }

   /** @inheritDoc */
   @Override
   public void setManipulationWeights(double jointspace, double taskspace, double userMode)
   {
      handJointspaceWeight = jointspace;
      handAngularTaskspaceWeight.set(taskspace, taskspace, taskspace);
      handLinearTaskspaceWeight.set(taskspace, taskspace, taskspace);
      handUserModeWeight = userMode;
   }

   /** @inheritDoc */
   @Override
   public void setFootWeights(double support, double swing)
   {
      highLinearFootWeight.set(support, support, support);
      highAngularFootWeight.set(support, support, support);
      defaultLinearFootWeight.set(swing, swing, swing);
      defaultAngularFootWeight.set(swing, swing, swing);
   }

   /** @inheritDoc */
   @Override
   public void setFootWeights(Vector3d supportAngular, Vector3d supportLinear, Vector3d swingAngular, Vector3d swingLinear)
   {
      highLinearFootWeight.set(supportLinear);
      highAngularFootWeight.set(supportAngular);
      defaultLinearFootWeight.set(swingLinear);
      defaultAngularFootWeight.set(swingAngular);
   }

   /** @inheritDoc */
   @Override
   public void setPelvisWeight(double pelvisAngularWeight)
   {
      this.pelvisAngularWeight.set(pelvisAngularWeight, pelvisAngularWeight, pelvisAngularWeight);
   }

   /** @inheritDoc */
   @Override
   public void setPelvisWeights(Vector3d pelvisAngularWeight)
   {
      this.pelvisAngularWeight.set(pelvisAngularWeight);
   }

   /** @inheritDoc */
   @Override
   public void setMomentumWeight(double linearMomentumWeightX, double linearMomentumWeightY, double linearMomentumWeightZ, double angularMomentumWeightXY,
         double angularMomentumWeightZ)
   {
      linearMomentumWeight.set(linearMomentumWeightX, linearMomentumWeightY, linearMomentumWeightZ);
      setAngularMomentumWeight(angularMomentumWeightXY, angularMomentumWeightZ);
   }

   /** @inheritDoc */
   @Override
   public void setMomentumWeight(double linearMomentumWeightXY, double linearMomentumWeightZ, double angularMomentumWeightXY, double angularMomentumWeightZ)
   {
      setMomentumWeight(linearMomentumWeightXY, linearMomentumWeightXY, linearMomentumWeightZ, angularMomentumWeightXY, angularMomentumWeightZ);
   }

   /** @inheritDoc */
   @Override
   public void setAngularMomentumWeight(double angularMomentumWeightXY, double angularMomentumWeightZ)
   {
      angularMomentumWeight.set(angularMomentumWeightXY, angularMomentumWeightXY, angularMomentumWeightZ);
   }

   /** @inheritDoc */
   @Override
   public void setRhoPlaneContactRegularization(double rhoWeight)
   {
      this.rhoWeight = rhoWeight;
   }

   /** @inheritDoc */
   @Override
   public void setRhoRateWeight(double defaultWeight, double highWeight)
   {
      this.rhoRateDefaultWeight = defaultWeight;
      this.rhoRateHighWeight = highWeight;
   }

   /** @inheritDoc */
   @Override
   public void setJointWeight(double jointAccelerationWeight, double jointJerkWeight)
   {
      this.jointAccelerationWeight = jointAccelerationWeight;
      this.jointJerkWeight = jointJerkWeight;
   }

   /** @inheritDoc */
   @Override
   public void setRhoMin(double rhoMin)
   {
      this.rhoMin = rhoMin;
   }

   /** @inheritDoc */
   @Override
   public void setCoPWeight(double weightX, double weightY)
   {
      copWeight.set(weightX, weightY);
   }

   /** @inheritDoc */
   @Override
   public void setCoPWeight(Vector2d weight)
   {
      copWeight.set(weight);
   }

   /** @inheritDoc */
   @Override
   public void setCoPRateDefaultWeight(double weightX, double weightY)
   {
      copRateDefaultWeight.set(weightX, weightY);
   }

   /** @inheritDoc */
   @Override
   public void setCoPRateDefaultWeight(Vector2d weight)
   {
      copRateDefaultWeight.set(weight);
   }

   /** @inheritDoc */
   @Override
   public void setCoPRateHighWeight(double weightX, double weightY)
   {
      copRateHighWeight.set(weightX, weightY);
   }

   /** @inheritDoc */
   @Override
   public void setCoPRateHighWeight(Vector2d weight)
   {
      copRateHighWeight.set(weight);
   }

   /** @inheritDoc */
   @Override
   public void setNumberOfBasisVectorsPerContactPoint(int nBasisVectorsPerContactPoint)
   {
      this.nBasisVectorsPerContactPoint = nBasisVectorsPerContactPoint;
   }

   /** @inheritDoc */
   @Override
   public void setNumberOfContactPointsPerContactableBody(int nContactPointsPerContactableBody)
   {
      this.nContactPointsPerContactableBody = nContactPointsPerContactableBody;
   }

   /** @inheritDoc */
   @Override
   public void setNumberOfContactableBodies(int nContactableBodies)
   {
      this.nContactableBodies = nContactableBodies;
   }

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
      return nContactableBodies * nContactPointsPerContactableBody * nBasisVectorsPerContactPoint;
   }

   /** @inheritDoc */
   @Override
   public double getSpineJointspaceWeight()
   {
      return spineJointspaceWeight;
   }
}
