package us.ihmc.simulationConstructionSetTools.externalcontroller;

import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.SliderJoint;

class SliderJointRobotSensor implements SensorInterface
{
   private DoubleYoVariable q, qd, qdd, tau_actual;

   public SliderJointRobotSensor(SliderJoint joint)
   {
      q = joint.getQYoVariable();
      qd = joint.getQDYoVariable();
      qdd = joint.getQDDYoVariable();
      tau_actual = joint.getTauYoVariable();

   }


   @Override
   public double[] getMessageValues()
   {
      return new double[] {q.getDoubleValue(), qd.getDoubleValue(), qdd.getDoubleValue(), tau_actual.getDoubleValue()};

   }


   @Override
   public String getYoVariableOrder()
   {
      String variableOrder = q.getName() + "," + qd.getName() + "," + qdd.getName() + "," + tau_actual.getName();

      return variableOrder;
   }


   @Override
   public int getNumberOfVariables()
   {
      // TODO Auto-generated method stub
      return 4;
   }

   @Override
   public void setTau(double tau)
   {
      tau_actual.set(tau);

   }

}
