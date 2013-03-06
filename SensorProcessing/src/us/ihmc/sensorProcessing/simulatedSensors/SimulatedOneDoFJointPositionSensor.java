package us.ihmc.sensorProcessing.simulatedSensors;

import org.apache.commons.lang.mutable.MutableDouble;

import us.ihmc.controlFlow.ControlFlowElement;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.sensorProcessing.signalCorruption.SignalCorruptorHolder;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

public class SimulatedOneDoFJointPositionSensor extends SignalCorruptorHolder<MutableDouble> implements ControlFlowElement
{
   private final ControlFlowOutputPort<Double> jointPositionOutputPort = new ControlFlowOutputPort<Double>(this);
   private final OneDoFJoint joint;
   private final MutableDouble jointPosition = new MutableDouble();

   public SimulatedOneDoFJointPositionSensor(OneDoFJoint joint)
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
