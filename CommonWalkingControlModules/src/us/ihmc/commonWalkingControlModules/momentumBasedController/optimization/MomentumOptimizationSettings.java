package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

public abstract class MomentumOptimizationSettings
{
   public abstract void scaleForceWeights(double scale);

   public abstract void setHeadWeights(double jointspace, double taskspace, double userMode);

   public abstract void setBodyWeights(double chestWeight, double pelvisWeight);

   public abstract void setBodyWeights(Vector3d chestAngularWeight, Vector3d pelvisAngularWeight);

   public abstract void setHandTaskspaceControlWeights(Vector3d angular, Vector3d linear);

   public abstract void setManipulationWeights(double jointspace, double taskspace, double userMode);

   public abstract void setFootWeights(double support, double swing);

   public abstract void setFootWeights(Vector3d supportAngular, Vector3d supportLinear, Vector3d swingAngular, Vector3d swingLinear);

   public abstract void setPelvisWeight(double pelvisAngularWeight);

   public abstract void setPelvisWeights(Vector3d pelvisAngularWeight);

   public abstract void setMomentumWeight(double linearMomentumWeightX, double linearMomentumWeightY, double linearMomentumWeightZ, double angularMomentumWeightXY, double angularMomentumWeightZ);

   public abstract void setMomentumWeight(double linearMomentumWeightXY, double linearMomentumWeightZ, double angularMomentumWeightXY, double angularMomentumWeightZ);

   public abstract void setAngularMomentumWeight(double angularMomentumWeightXY, double angularMomentumWeightZ);

   public abstract void setRhoPlaneContactRegularization(double rhoWeight);

   public abstract void setRhoRateWeight(double defaultWeight, double highWeight);

   public abstract void setJointWeight(double jointAccelerationWeight, double jointJerkWeight);

   public abstract void setRhoMin(double rhoMin);

   public abstract void setCoPWeight(double weightX, double weightY);

   public abstract void setCoPWeight(Vector2d weight);

   public abstract void setCoPRateDefaultWeight(double weightX, double weightY);

   public abstract void setCoPRateDefaultWeight(Vector2d weight);

   public abstract void setCoPRateHighWeight(double weightX, double weightY);

   public abstract void setCoPRateHighWeight(Vector2d weight);

   public abstract void setNumberOfBasisVectorsPerContactPoint(int nBasisVectorsPerContactPoint);

   public abstract void setNumberOfContactPointsPerContactableBody(int nContactPointsPerContactableBody);

   public abstract void setNumberOfContactableBodies(int nContactableBodies);

   public abstract Vector3d getLinearMomentumWeight();

   public abstract Vector3d getHighLinearMomentumWeightForRecovery();

   public abstract Vector3d getAngularMomentumWeight();

   public abstract double getJointAccelerationWeight();

   public abstract double getJointJerkWeight();

   public abstract double getRhoWeight();

   public abstract double getRhoMin();

   public abstract double getRhoRateDefaultWeight();

   public abstract double getRhoRateHighWeight();

   public abstract Vector2d getCoPWeight();

   public abstract Vector2d getCoPRateDefaultWeight();

   public abstract Vector2d getCoPRateHighWeight();

   public abstract double getHeadUserModeWeight();

   public abstract double getHeadJointspaceWeight();

   public abstract double getHeadTaskspaceWeight();

   public abstract Vector3d getChestAngularWeight();

   public abstract Vector3d getPelvisAngularWeight();

   public abstract Vector3d getDefaultLinearFootWeight();

   public abstract Vector3d getDefaultAngularFootWeight();

   public abstract Vector3d getHighLinearFootWeight();

   public abstract Vector3d getHighAngularFootWeight();

   public abstract double getHandUserModeWeight();

   public abstract double getHandJointspaceWeight();

   public abstract Vector3d getHandAngularTaskspaceWeight();

   public abstract Vector3d getHandLinearTaskspaceWeight();

   public abstract int getNumberOfBasisVectorsPerContactPoint();

   public abstract int getNumberOfContactPointsPerContactableBody();

   public abstract int getNumberOfContactableBodies();

   public abstract int getRhoSize();

   public abstract double getSpineJointspaceWeight();
}
