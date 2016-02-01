package us.ihmc.simulationconstructionset.externalcontroller;

import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.PinJoint;

class PinJointRobotSensor implements SensorInterface
{
   private DoubleYoVariable q, qd, qdd, tau_actual;

   PinJointRobotSensor(PinJoint joint)
   {
      q = joint.getQ();
      qd = joint.getQD();
      qdd = joint.getQDD();
      tau_actual = joint.getTau();
   }


   public double[] getMessageValues()
   {
      return new double[] {q.getDoubleValue(), qd.getDoubleValue(), qdd.getDoubleValue(), tau_actual.getDoubleValue()};
   }


   public String getYoVariableOrder()
   {
      String variableOrder = q.getName() + "," + qd.getName() + "," + qdd.getName() + "," + tau_actual.getName();

      return variableOrder;
   }


   public int getNumberOfVariables()
   {
      return 4;

   }


   public void setTau(double tau)
   {
      tau_actual.set(tau);

   }

}
