package us.ihmc.commonWalkingControlModules.configurations;

import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;

public abstract class JumpControllerParameters extends AbstractHighLevelControllerParameters
{
   public abstract MomentumOptimizationSettings getMomentumOptimizationSettings();
   public abstract double getMotionPlanningNodeTime();
   public abstract double getForceRegularizationWeight();
   public abstract double getForceRateRegularizationWeight();
   public abstract Vector3DReadOnly getMaxForce();
   public abstract Vector3DReadOnly getMinForce();
   public abstract Vector3DReadOnly getMaxForceRate();
   public abstract Vector3DReadOnly getMinForceRate();
   public abstract Vector3DReadOnly getPositionErrorGain();
   public abstract Vector3DReadOnly getVelocityErrorGain();
}
