package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

public abstract class MomentumOptimizationSettings
{
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
