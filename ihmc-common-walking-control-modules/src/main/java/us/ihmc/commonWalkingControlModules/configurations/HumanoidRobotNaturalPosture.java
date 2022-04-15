package us.ihmc.commonWalkingControlModules.configurations;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public class HumanoidRobotNaturalPosture
{
   public HumanoidRobotNaturalPosture()
   {      
   }
   
   public void setNaturalPostureOffset(QuaternionReadOnly Qoffset)
   {
   }
   
   public void compute(double[] q, QuaternionReadOnly Qbase)
   {      
   }
   
   public Quaternion getNaturalPostureQuaternion()
   {
      return null;
   }
   
   public DMatrixRMaj getNaturalPostureJacobian()
   {
      return null;
   }
}