package us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;

public class LinearAccelerationSensorConfiguration
{
   private final ControlFlowOutputPort<Vector3D> outputPort;
   private final String name;
   private final RigidBody linearAccelerationMeasurementLink;
   private final ReferenceFrame linearAccelerationMeasurementFrame;
   
   private final double gravityZ;
   
   private final DenseMatrix64F linearAccelerationNoiseCovariance;
   private final DenseMatrix64F biasProcessNoiseCovariance;

   public LinearAccelerationSensorConfiguration(ControlFlowOutputPort<Vector3D> outputPort, String name, RigidBody measurementLink,
           ReferenceFrame measurementFrame, double gravityZ, DenseMatrix64F linearAccelerationNoiseCovariance, DenseMatrix64F biasProcessNoiseCovariance)
   {
      this.outputPort = outputPort;
      this.name = name;
      this.linearAccelerationMeasurementLink = measurementLink;
      this.linearAccelerationMeasurementFrame = measurementFrame;
      
      this.gravityZ = gravityZ;
      
      this.linearAccelerationNoiseCovariance = linearAccelerationNoiseCovariance;
      this.biasProcessNoiseCovariance = biasProcessNoiseCovariance;
   }

   public RigidBody getLinearAccelerationMeasurementLink()
   {
      return linearAccelerationMeasurementLink;
   }

   public ReferenceFrame getMeasurementFrame()
   {
      return linearAccelerationMeasurementFrame;
   }

   public DenseMatrix64F getLinearAccelerationNoiseCovariance()
   {
      return linearAccelerationNoiseCovariance;
   }

   public DenseMatrix64F getBiasProcessNoiseCovariance()
   {
      return biasProcessNoiseCovariance;
   }

   public String getName()
   {
      return name;
   }

   public ControlFlowOutputPort<Vector3D> getOutputPort()
   {
      return outputPort;
   }

   public double getGravityZ()
   {
      return gravityZ;
   }
}

