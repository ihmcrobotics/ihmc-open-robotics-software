package us.ihmc.commonWalkingControlModules.configurations;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public interface HumanoidRobotNaturalPosture
{
   public abstract void setNaturalPostureOffset(QuaternionReadOnly Qoffset);

   public abstract void compute(double[] q, QuaternionReadOnly Qbase);

   public abstract Quaternion getNaturalPostureQuaternion();

   public abstract Quaternion getNaturalPostureQuaternionrtBase();
   
   public abstract DMatrixRMaj getNaturalPostureJacobian();

}