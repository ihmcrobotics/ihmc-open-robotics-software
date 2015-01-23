package us.ihmc.sensorProcessing.simulatedSensors;

import org.apache.commons.lang.mutable.MutableDouble;

import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;

public class SimulatedOneDoFJointTorqueSensor extends SimulatedSensor<MutableDouble>
{
   private final ControlFlowOutputPort<Double> jointTorqueOutputPort = createOutputPort();
   private final OneDegreeOfFreedomJoint joint;
   private final MutableDouble jointTorque = new MutableDouble();

   public SimulatedOneDoFJointTorqueSensor(String name, OneDegreeOfFreedomJoint joint)
   {
      this.joint = joint;
   }

   public void startComputation()
   {
      jointTorque.setValue(joint.getTau().getDoubleValue());
      corrupt(jointTorque);
      jointTorqueOutputPort.setData(jointTorque.doubleValue());
   }

   public void waitUntilComputationIsDone()
   {
      // empty
   }

   public ControlFlowOutputPort<Double> getJointTorqueOutputPort()
   {
      return jointTorqueOutputPort;
   }

   public void initialize()
   {
//    empty
   }
}
