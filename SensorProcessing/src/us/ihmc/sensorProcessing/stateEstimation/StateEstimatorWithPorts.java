package us.ihmc.sensorProcessing.stateEstimation;

import java.util.List;

import us.ihmc.controlFlow.ControlFlowGraph;
import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.PointPositionDataObject;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.PointVelocityDataObject;

public interface StateEstimatorWithPorts extends StateEstimator
{
   public abstract ControlFlowGraph getControlFlowGraph();
   public abstract ControlFlowInputPort<FrameVector> getDesiredAngularAccelerationInputPort();
   public abstract ControlFlowInputPort<FrameVector> getDesiredCenterOfMassAccelerationInputPort();
   public abstract ControlFlowInputPort<List<PointPositionDataObject>> getPointPositionInputPort();
   public abstract ControlFlowInputPort<List<PointVelocityDataObject>> getPointVelocityInputPort();
}
