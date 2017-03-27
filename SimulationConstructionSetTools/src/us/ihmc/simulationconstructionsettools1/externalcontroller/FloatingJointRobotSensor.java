package us.ihmc.simulationconstructionsettools1.externalcontroller;

import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.FloatingJoint;

class FloatingJointRobotSensor implements SensorInterface
{
   private DoubleYoVariable q_x, q_y, q_z, qd_x, qd_y, qd_z, q_qs, q_qx, q_qy, q_qz, qd_wx, qd_wy, qd_wz;
   private DoubleYoVariable qdd_x, qdd_y, qdd_z, qdd_wx, qdd_wy, qdd_wz;

   FloatingJointRobotSensor(FloatingJoint joint)
   {
      q_x = joint.getQx();
      q_y = joint.getQy();
      q_z = joint.getQz();
      qd_x = joint.getQdx();
      qd_y = joint.getQdy();
      qd_z = joint.getQdz();
      q_qs = joint.getQuaternionQs();
      q_qx = joint.getQuaternionQx();
      q_qy = joint.getQuaternionQy();
      q_qz = joint.getQuaternionQz();
      qd_wx = joint.getAngularVelocityX();
      qd_wy = joint.getAngularVelocityY();
      qd_wz = joint.getAngularVelocityZ();
      qdd_x = joint.getQddx();
      qdd_y = joint.getQddy();
      qdd_z = joint.getQddz();
      qdd_wx = joint.getAngularAccelerationX();
      qdd_wy = joint.getAngularAccelerationY();
      qdd_wz = joint.getAngularAccelerationZ();

   }


   public double[] getMessageValues()
   {
      return new double[]
      {
         q_x.getDoubleValue(), q_y.getDoubleValue(), q_z.getDoubleValue(), qd_x.getDoubleValue(), qd_y.getDoubleValue(), qd_z.getDoubleValue(),
         q_qs.getDoubleValue(), q_qx.getDoubleValue(), q_qy.getDoubleValue(), q_qz.getDoubleValue(), qd_wx.getDoubleValue(), qd_wy.getDoubleValue(),
         qd_wz.getDoubleValue(), qdd_x.getDoubleValue(), qdd_y.getDoubleValue(), qdd_z.getDoubleValue(), qdd_wx.getDoubleValue(), qdd_wy.getDoubleValue(),
         qdd_wz.getDoubleValue()
      };

   }


   public String getYoVariableOrder()
   {
      String variableOrder = q_x.getName() + "," + q_y.getName() + "," + q_z.getName() + "," + qd_x.getName() + "," + qd_y.getName() + "," + qd_z.getName()
                             + "," + q_qs.getName() + "," + q_qx.getName() + "," + q_qy.getName() + "," + q_qz.getName() + "," + qd_wx.getName() + ","
                             + qd_wy.getName() + "," + qd_wz.getName() + "," + qdd_x.getName() + "," + qdd_y.getName() + "," + qdd_z.getName() + ","
                             + qdd_wx.getName() + "," + qdd_wy.getName() + "," + qdd_wz.getName();

      return variableOrder;
   }


   public int getNumberOfVariables()
   {
      // TODO Auto-generated method stub
      return 19;
   }


   public void setTau(double tau)
   {
   }

}
