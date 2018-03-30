package us.ihmc.commonWalkingControlModules.configurations;

import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;

public abstract class JumpControllerParameters extends AbstractHighLevelControllerParameters
{
   public abstract MomentumOptimizationSettings getMomentumOptimizationSettings();
   public abstract double getMotionPlanningNodeTime();
   public abstract double getForceRegularizationWeight();
   public abstract double getForceRateRegularizationWeight();
   public abstract Vector3D getMaxForce();
   public abstract Vector3D getMinForce();
   public abstract Vector3D getMaxForceRate();
   public abstract Vector3D getMinForceRate();
   public abstract double getPositionErrorGain();
   public abstract double getVelocityErrorGain();
}
