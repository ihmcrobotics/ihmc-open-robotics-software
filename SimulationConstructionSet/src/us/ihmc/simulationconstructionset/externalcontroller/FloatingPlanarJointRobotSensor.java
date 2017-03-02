package us.ihmc.simulationconstructionset.externalcontroller;

import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.FloatingPlanarJoint;

class FloatingPlanarJointRobotSensor implements SensorInterface
{
   private DoubleYoVariable q_t1, q_t2, qd_t1, qd_t2, q_rot, qd_rot;
   private DoubleYoVariable qdd_t1, qdd_t2, qdd_rot;

   public FloatingPlanarJointRobotSensor(FloatingPlanarJoint joint)
   {
      q_t1 = joint.getQ_t1();
      q_t2 = joint.getQ_t2();
      qd_t1 = joint.getQd_t1();
      qd_t2 = joint.getQd_t2();
      q_rot = joint.getQ_rot();
      qd_rot = joint.getQd_rot();
      qdd_t1 = joint.getQdd_t1();
      qdd_t2 = joint.getQdd_t2();
      qdd_rot = joint.getQdd_rot();
   }


   @Override
   public double[] getMessageValues()
   {
      return new double[]
      {
         q_t1.getDoubleValue(), q_t2.getDoubleValue(), qd_t1.getDoubleValue(), qd_t2.getDoubleValue(), q_rot.getDoubleValue(), qd_rot.getDoubleValue(),
         qdd_t1.getDoubleValue(), qdd_t2.getDoubleValue(), qdd_rot.getDoubleValue()
      };

   }


   @Override
   public String getYoVariableOrder()
   {
      String variableOrder = q_t1.getName() + "," + q_t2.getName() + "," + qd_t1.getName() + "," + qd_t2.getName() + "," + q_rot.getName() + ","
                             + qd_rot.getName() + "," + qdd_t1.getName() + "," + qdd_t2.getName() + "," + qdd_rot.getName();

      return variableOrder;
   }


   @Override
   public int getNumberOfVariables()
   {
      // TODO Auto-generated method stub
      return 9;
   }

   @Override
   public void setTau(double tau)
   {
   }

}
