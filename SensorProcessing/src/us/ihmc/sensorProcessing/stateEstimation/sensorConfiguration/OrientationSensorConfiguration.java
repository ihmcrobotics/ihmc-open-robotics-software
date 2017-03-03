package us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class OrientationSensorConfiguration
{
   private final ControlFlowOutputPort<RotationMatrix> outputPort;

   private final String name;
   private final ReferenceFrame measurementFrame;
   private final DenseMatrix64F orientationNoiseCovariance;

   public OrientationSensorConfiguration(ControlFlowOutputPort<RotationMatrix> outputPort, String name, ReferenceFrame measurementFrame,
           DenseMatrix64F orientationNoiseCovariance)
   {
      this.outputPort = outputPort;
      this.name = name;
      this.measurementFrame = measurementFrame;
      this.orientationNoiseCovariance = orientationNoiseCovariance;
   }

   public ReferenceFrame getMeasurementFrame()
   {
      return measurementFrame;
   }

   public String getName()
   {
      return name;
   }

   public ControlFlowOutputPort<RotationMatrix> getOutputPort()
   {
      return outputPort;
   }

   public DenseMatrix64F getOrientationNoiseCovariance()
   {
      return orientationNoiseCovariance;
   }
}
