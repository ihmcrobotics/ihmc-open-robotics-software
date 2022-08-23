package us.ihmc.commonWalkingControlModules.configurations;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public interface HumanoidRobotNaturalPosture
{
   public abstract void initialize();
   
   public abstract Quaternion getNominalStandingPoseQoffset();
   
   public abstract void setNaturalPostureOffset(QuaternionReadOnly Qoffset);

   public abstract double[] getJointPositionArray();

   default void compute(Orientation3DReadOnly Qbase)
   {
      compute(getJointPositionArray(), Qbase);
   }

   public abstract void compute(double[] q, Orientation3DReadOnly Qbase);

   /**
    * This function returns the natural posture quaternion in the world.
    */
   public abstract Quaternion getNaturalPostureQuaternion();

   /**
    * This function returns the natural posture quaternion w.r.t the base of the robot (typically the pelvis).
    */
   public abstract Quaternion getNaturalPostureQuaternionrtBase();

   /**
    *    This returns [Jacobian] -> omega_NP_rt_world_ewrt_NP = [Jacobian] * [omega_Base_rt_world_ewrt_Base; q_dot]
    */
   public abstract DMatrixRMaj getNaturalPostureJacobian();
}