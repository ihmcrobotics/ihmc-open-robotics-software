package us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration;

import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;

public class AngularVelocitySensorConfiguration
{
   private final ControlFlowOutputPort<Vector3d> outputPort;
   private final String name;
   private final RigidBody angularVelocityMeasurementLink;
   private final ReferenceFrame angularVelocityMeasurementFrame;
   private final DenseMatrix64F angularVelocityNoiseCovariance;
   private final DenseMatrix64F biasProcessNoiseCovariance;

   public AngularVelocitySensorConfiguration(ControlFlowOutputPort<Vector3d> outputPort, String name, RigidBody measurementLink,
           ReferenceFrame measurementFrame, DenseMatrix64F angularVelocityNoiseCovariance, DenseMatrix64F biasProcessNoiseCovariance)
   {
      this.outputPort = outputPort;
      this.name = name;
      this.angularVelocityMeasurementLink = measurementLink;
      this.angularVelocityMeasurementFrame = measurementFrame;
      this.angularVelocityNoiseCovariance = angularVelocityNoiseCovariance;
      this.biasProcessNoiseCovariance = biasProcessNoiseCovariance;
   }

   public RigidBody getAngularVelocityMeasurementLink()
   {
      return angularVelocityMeasurementLink;
   }

   public ReferenceFrame getMeasurementFrame()
   {
      return angularVelocityMeasurementFrame;
   }

   public DenseMatrix64F getAngularVelocityNoiseCovariance()
   {
      return angularVelocityNoiseCovariance;
   }

   public DenseMatrix64F getBiasProcessNoiseCovariance()
   {
      return biasProcessNoiseCovariance;
   }

   public String getName()
   {
      return name;
   }

   public ControlFlowOutputPort<Vector3d> getOutputPort()
   {
      return outputPort;
   }
}
