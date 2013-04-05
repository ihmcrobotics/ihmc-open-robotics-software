package us.ihmc.sensorProcessing.simulatedSensors;

import org.apache.commons.lang.mutable.MutableDouble;

import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

public class SimulatedOneDoFJointVelocitySensor extends SimulatedSensor<MutableDouble>
{
   private final ControlFlowOutputPort<Double> jointPositionOutputPort = createOutputPort();
   private final OneDoFJoint joint;
   private final MutableDouble jointVelocity = new MutableDouble();

   public SimulatedOneDoFJointVelocitySensor(String name, OneDoFJoint joint)
   {
      this.joint = joint;
   }

   public void startComputation()
   {
      jointVelocity.setValue(joint.getQd());
      corrupt(jointVelocity);
      jointPositionOutputPort.setData(jointVelocity.toDouble());
   }

   public void waitUntilComputationIsDone()
   {
      // empty
   }

   public ControlFlowOutputPort<Double> getJointPositionOutputPort()
   {
      return jointPositionOutputPort;
   }
}
