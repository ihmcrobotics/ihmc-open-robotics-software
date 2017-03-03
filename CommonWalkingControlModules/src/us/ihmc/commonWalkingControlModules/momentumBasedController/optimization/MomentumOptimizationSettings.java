package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;

public abstract class MomentumOptimizationSettings
{
   public abstract Vector3D getLinearMomentumWeight();

   public abstract Vector3D getHighLinearMomentumWeightForRecovery();

   public abstract Vector3D getAngularMomentumWeight();

   public abstract double getJointAccelerationWeight();

   public abstract double getJointJerkWeight();

   public double getJointTorqueWeight()
   {
      return 0.005;
   }

   public abstract double getRhoWeight();

   public abstract double getRhoMin();

   public abstract double getRhoRateDefaultWeight();

   public abstract double getRhoRateHighWeight();

   public abstract Vector2D getCoPWeight();

   public abstract Vector2D getCoPRateDefaultWeight();

   public abstract Vector2D getCoPRateHighWeight();

   public abstract double getHeadUserModeWeight();

   public abstract double getHeadJointspaceWeight();

   public abstract double getHeadTaskspaceWeight();

   public abstract Vector3D getChestAngularWeight();

   public abstract Vector3D getPelvisAngularWeight();

   public abstract Vector3D getDefaultLinearFootWeight();

   public abstract Vector3D getDefaultAngularFootWeight();

   public abstract Vector3D getHighLinearFootWeight();

   public abstract Vector3D getHighAngularFootWeight();

   public abstract double getHandUserModeWeight();

   public abstract double getHandJointspaceWeight();

   public abstract Vector3D getHandAngularTaskspaceWeight();

   public abstract Vector3D getHandLinearTaskspaceWeight();

   public abstract int getNumberOfBasisVectorsPerContactPoint();

   public abstract int getNumberOfContactPointsPerContactableBody();

   public abstract int getNumberOfContactableBodies();

   public abstract int getRhoSize();

   public abstract double getSpineJointspaceWeight();
}
