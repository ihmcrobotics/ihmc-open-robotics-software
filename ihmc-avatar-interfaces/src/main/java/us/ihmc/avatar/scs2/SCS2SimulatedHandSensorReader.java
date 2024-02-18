package us.ihmc.avatar.scs2;

import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.avatar.factory.SimulatedHandSensorReader;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.scs2.definition.controller.ControllerInput;
import us.ihmc.sensorProcessing.simulatedSensors.SensorDataContext;

public class SCS2SimulatedHandSensorReader implements SimulatedHandSensorReader
{
   private final List<OneDoFJointReadOnly> simJoints;

   public SCS2SimulatedHandSensorReader(ControllerInput controllerInput, List<String> fingerJointNames)
   {
      simJoints = fingerJointNames.stream().map(jointName -> (OneDoFJointReadOnly) controllerInput.getInput().findJoint(jointName))
                                  .collect(Collectors.toList());
   }

   @Override
   public void read(SensorDataContext sensorDataContext)
   {
      for (int i = 0; i < simJoints.size(); i++)
      {
         OneDoFJointReadOnly joint = simJoints.get(i);
         double q = joint.getQ();
         sensorDataContext.getMeasuredJointState(joint.getName()).setPosition(q);
         double qd = joint.getQd();
         sensorDataContext.getMeasuredJointState(joint.getName()).setVelocity(qd);
         double qdd = joint.getQdd();
         sensorDataContext.getMeasuredJointState(joint.getName()).setAcceleration(qdd);
         double tau = joint.getTau();
         sensorDataContext.getMeasuredJointState(joint.getName()).setEffort(tau);
      }
   }
}
