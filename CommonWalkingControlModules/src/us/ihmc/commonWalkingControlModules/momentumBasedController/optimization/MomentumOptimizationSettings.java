package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

public class MomentumOptimizationSettings
{
   private final Vector3d linearMomentumWeight = new Vector3d(0.05, 0.05, 0.005);
   private final Vector3d angularMomentumWeight = new Vector3d(0.0, 0.0, 0.0);

   private double jointAccelerationWeight = 0.005;
   private double jointJerkWeight = 0.1;
   private double rhoWeight = 0.00001;
   private double rhoMin = 4.0;
   private double rhoRateDefaultWeight = 0.001;
   private double rhoRateHighWeight = 0.05;
   private final Vector2d copWeight = new Vector2d(750.0, 1500.0);
   private final Vector2d copRateDefaultWeight = new Vector2d(100000.0, 200000.0);
   private final Vector2d copRateHighWeight = new Vector2d(2500000.0, 10000000.0);
   private double headWeight  = 1.0;
   private double chestWeight = 10.0;
   private double pelvisWeight = 1.0;
   private double defaultFootWeight = 10.0;
   private double highFootWeight = 50.0;
   private double handUserModeWeight = 10.0;
   private double handJointspaceWeight = 1.0;
   private double handTaskspaceWeight = 1.0;

   public MomentumOptimizationSettings()
   {
   }

   public void setBodyWeights(double headWeight, double chestWeight, double pelvisWeight)
   {
      this.headWeight = headWeight;
      this.chestWeight = chestWeight;
      this.pelvisWeight = pelvisWeight;
   }

   public void setManipulationWeights(double jointspace, double taskspace, double userMode)
   {
      handJointspaceWeight = jointspace;
      handTaskspaceWeight = taskspace;
      handUserModeWeight = userMode;
   }

   public void setFootWeights(double support, double swing)
   {
      highFootWeight = support;
      defaultFootWeight = swing;
   }

   public void setMomentumWeight(double linearMomentumWeightX, double linearMomentumWeightY, double linearMomentumWeightZ, double angularMomentumWeightXY,
         double angularMomentumWeightZ)
   {
      linearMomentumWeight.set(linearMomentumWeightX, linearMomentumWeightY, linearMomentumWeightZ);
      angularMomentumWeight.set(angularMomentumWeightXY, angularMomentumWeightXY, angularMomentumWeightZ);
   }

   public void setMomentumWeight(double linearMomentumWeightXY, double linearMomentumWeightZ, double angularMomentumWeightXY, double angularMomentumWeightZ)
   {
      setMomentumWeight(linearMomentumWeightXY, linearMomentumWeightXY, linearMomentumWeightZ, angularMomentumWeightXY, angularMomentumWeightZ);
   }

   public void setRhoPlaneContactRegularization(double rhoWeight)
   {
      this.rhoWeight = rhoWeight;
   }

   public void setRhoRateWeight(double defaultWeight, double highWeight)
   {
      this.rhoRateDefaultWeight = defaultWeight;
      this.rhoRateHighWeight = highWeight;
   }

   public void setJointWeight(double jointAccelerationWeight, double jointJerkWeight)
   {
      this.jointAccelerationWeight = jointAccelerationWeight;
      this.jointJerkWeight = jointJerkWeight;
   }

   public void setRhoMin(double rhoMin)
   {
      this.rhoMin = rhoMin;
   }

   public void setCoPWeight(double weightX, double weightY)
   {
      copWeight.set(weightX, weightY);
   }

   public void setCoPRateDefaultWeight(double weightX, double weightY)
   {
      copRateDefaultWeight.set(weightX, weightY);
   }

   public void setCoPRateHighWeight(double weightX, double weightY)
   {
      copRateHighWeight.set(weightX, weightY);
   }

   public Vector3d getLinearMomentumWeight()
   {
      return linearMomentumWeight;
   }

   public Vector3d getAngularMomentumWeight()
   {
      return angularMomentumWeight;
   }

   public double getJointAccelerationWeight()
   {
      return jointAccelerationWeight;
   }

   public double getJointJerkWeight()
   {
      return jointJerkWeight;
   }

   public double getRhoWeight()
   {
      return rhoWeight;
   }

   public double getRhoMin()
   {
      return rhoMin;
   }

   public double getRhoRateDefaultWeight()
   {
      return rhoRateDefaultWeight;
   }

   public double getRhoRateHighWeight()
   {
      return rhoRateHighWeight;
   }

   public Vector2d getCoPWeight()
   {
      return copWeight;
   }

   public Vector2d getCoPRateDefaultWeight()
   {
      return copRateDefaultWeight;
   }

   public Vector2d getCoPRateHighWeight()
   {
      return copRateHighWeight;
   }

   public double getHeadWeight()
   {
      return headWeight;
   }

   public double getChestWeight()
   {
      return chestWeight;
   }

   public double getPelvisWeight()
   {
      return pelvisWeight;
   }

   public double getDefaultFootWeight()
   {
      return defaultFootWeight;
   }

   public double getHighFootWeight()
   {
      return highFootWeight;
   }

   public double getHandUserModeWeight()
   {
      return handUserModeWeight;
   }

   public double getHandJointspaceWeight()
   {
      return handJointspaceWeight;
   }

   public double getHandTaskspaceWeight()
   {
      return handTaskspaceWeight;
   }
}
