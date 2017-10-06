package us.ihmc.sensorProcessing.simulatedSensors;

import org.apache.commons.lang3.mutable.MutableDouble;

import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;

public class SimulatedOneDoFJointVelocitySensor extends SimulatedSensor<MutableDouble>
{
   private final ControlFlowOutputPort<MutableDouble> jointVelocityOutputPort = createOutputPort();
   private final OneDegreeOfFreedomJoint joint;
   private final MutableDouble jointVelocity = new MutableDouble();

   public SimulatedOneDoFJointVelocitySensor(String name, OneDegreeOfFreedomJoint joint)
   {
      this.joint = joint;
   }

   public void startComputation()
   {
      jointVelocity.setValue(joint.getQDYoVariable().getDoubleValue());
      corrupt(jointVelocity);
      jointVelocityOutputPort.setData(jointVelocity);
   }

   public void waitUntilComputationIsDone()
   {
      // empty
   }

   public ControlFlowOutputPort<MutableDouble> getJointVelocityOutputPort()
   {
      return jointVelocityOutputPort;
   }

   public void initialize()
   {
//    empty
   }
}
