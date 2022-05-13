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

   public abstract Quaternion getNaturalPostureQuaternion();

   public abstract Quaternion getNaturalPostureQuaternionrtBase();
   
   public abstract DMatrixRMaj getNaturalPostureJacobian();
}