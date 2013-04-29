package us.ihmc.sensorProcessing.stateEstimation;

import us.ihmc.controlFlow.ControlFlowGraph;
import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.PointPositionDataObject;
import us.ihmc.utilities.math.geometry.FrameVector;

import java.util.Set;

public interface StateEstimatorWithPorts extends StateEstimator
{
   public abstract ControlFlowGraph getControlFlowGraph();
   public abstract ControlFlowInputPort<FrameVector> getDesiredAngularAccelerationInputPort();
   public abstract ControlFlowInputPort<FrameVector> getDesiredCenterOfMassAccelerationInputPort();
   public abstract ControlFlowInputPort<Set<PointPositionDataObject>> getPointPositionInputPort();
}
