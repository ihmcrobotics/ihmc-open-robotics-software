package us.ihmc.sensorProcessing.simulatedSensors;

import org.apache.commons.lang3.mutable.MutableDouble;

import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;

public class SimulatedOneDoFJointTorqueSensor extends SimulatedSensor<MutableDouble>
{
   private final ControlFlowOutputPort<MutableDouble> jointTorqueOutputPort = createOutputPort();
   private final OneDegreeOfFreedomJoint joint;
   private final MutableDouble jointTorque = new MutableDouble();

   public SimulatedOneDoFJointTorqueSensor(String name, OneDegreeOfFreedomJoint joint)
   {
      this.joint = joint;
   }

   public void startComputation()
   {
      jointTorque.setValue(joint.getTauYoVariable().getDoubleValue());
      corrupt(jointTorque);
      jointTorqueOutputPort.setData(jointTorque);
   }

   public void waitUntilComputationIsDone()
   {
      // empty
   }

   public ControlFlowOutputPort<MutableDouble> getJointTorqueOutputPort()
   {
      return jointTorqueOutputPort;
   }

   public void initialize()
   {
//    empty
   }
}
