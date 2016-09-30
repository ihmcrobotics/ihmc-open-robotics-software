package us.ihmc.sensorProcessing.simulatedSensors;

import org.apache.commons.lang3.mutable.MutableDouble;

import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;

public class SimulatedOneDoFJointPositionSensor extends SimulatedSensor<MutableDouble>
{
   private final ControlFlowOutputPort<MutableDouble> jointPositionOutputPort = createOutputPort();
   private final OneDegreeOfFreedomJoint joint;
   private final MutableDouble jointPosition = new MutableDouble();

   public SimulatedOneDoFJointPositionSensor(String name, OneDegreeOfFreedomJoint joint)
   {
      this.joint = joint;
   }

   public void startComputation()
   {
      jointPosition.setValue(joint.getQYoVariable().getDoubleValue());
      corrupt(jointPosition);
      jointPositionOutputPort.setData(jointPosition);
   }

   public void waitUntilComputationIsDone()
   {
      // empty
   }

   public ControlFlowOutputPort<MutableDouble> getJointPositionOutputPort()
   {
      return jointPositionOutputPort;
   }

   public void initialize()
   {
//    empty
   }
}
