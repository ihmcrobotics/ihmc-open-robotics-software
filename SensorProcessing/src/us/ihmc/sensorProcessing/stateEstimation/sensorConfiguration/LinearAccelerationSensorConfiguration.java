package us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration;

import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;

public class LinearAccelerationSensorConfiguration
{
   private final ControlFlowOutputPort<Vector3d> outputPort;
   private final String name;
   private final RigidBody linearAccelerationMeasurementLink;
   private final ReferenceFrame linearAccelerationMeasurementFrame;
   
   private final double gravityZ;
   
   private final DenseMatrix64F linearAccelerationNoiseCovariance;
   private final DenseMatrix64F biasProcessNoiseCovariance;

   public LinearAccelerationSensorConfiguration(ControlFlowOutputPort<Vector3d> outputPort, String name, RigidBody measurementLink,
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

   public ControlFlowOutputPort<Vector3d> getOutputPort()
   {
      return outputPort;
   }

   public double getGravityZ()
   {
      return gravityZ;
   }
}

