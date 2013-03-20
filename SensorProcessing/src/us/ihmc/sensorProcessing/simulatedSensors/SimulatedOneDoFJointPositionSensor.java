package us.ihmc.sensorProcessing.simulatedSensors;

import org.apache.commons.lang.mutable.MutableDouble;

import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

public class SimulatedOneDoFJointPositionSensor extends SimulatedSensor<MutableDouble>
{
   private final ControlFlowOutputPort<Double> jointPositionOutputPort = createOutputPort();
   private final OneDoFJoint joint;
   private final MutableDouble jointPosition = new MutableDouble();

   public SimulatedOneDoFJointPositionSensor(String name, OneDoFJoint joint)
   {
      this.joint = joint;
   }

   public void startComputation()
   {
      jointPosition.setValue(joint.getQ());
      corrupt(jointPosition);
      jointPositionOutputPort.setData(jointPosition.toDouble());
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
