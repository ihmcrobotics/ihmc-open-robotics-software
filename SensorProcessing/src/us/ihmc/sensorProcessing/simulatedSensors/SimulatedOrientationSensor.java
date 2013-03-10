package us.ihmc.sensorProcessing.simulatedSensors;

import javax.vecmath.Matrix3d;

import us.ihmc.controlFlow.ControlFlowElement;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.sensorProcessing.signalCorruption.SignalCorruptorHolder;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class SimulatedOrientationSensor extends SignalCorruptorHolder<Matrix3d> implements ControlFlowElement
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame measurementFrame;

   private final Matrix3d rotationMatrix = new Matrix3d();

   private final ControlFlowOutputPort<Matrix3d> orientationOutputPort = new ControlFlowOutputPort<Matrix3d>(this);

   public SimulatedOrientationSensor(ReferenceFrame measurementFrame)
   {
      this.measurementFrame = measurementFrame;
   }

   public void startComputation()
   {
      measurementFrame.getTransformToDesiredFrame(worldFrame).get(rotationMatrix);
      corrupt(rotationMatrix);
      orientationOutputPort.setData(rotationMatrix);
   }

   public void waitUntilComputationIsDone()
   {
      // empty
   }

   public ControlFlowOutputPort<Matrix3d> getOrientationOutputPort()
   {
      return orientationOutputPort;
   }
}
