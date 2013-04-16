package us.ihmc.sensorProcessing.stateEstimation;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.controlFlow.ControlFlowGraph;
import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.sensorProcessing.controlFlowPorts.YoFrameQuaternionControlFlowOutputPort;
import us.ihmc.sensorProcessing.controlFlowPorts.YoFrameVectorControlFlowOutputPort;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.sensorProcessing.stateEstimation.measurmentModelElements.AngularVelocityMeasurementModelElement;
import us.ihmc.sensorProcessing.stateEstimation.measurmentModelElements.OrientationMeasurementModelElement;
import us.ihmc.sensorProcessing.stateEstimation.processModelElements.AngularVelocityProcessModelElement;
import us.ihmc.sensorProcessing.stateEstimation.processModelElements.BiasProcessModelElement;
import us.ihmc.sensorProcessing.stateEstimation.processModelElements.OrientationProcessModelElement;
import us.ihmc.sensorProcessing.stateEstimation.processModelElements.ProcessModelElement;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.AngularVelocitySensorConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.OrientationSensorConfiguration;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class ComposableOrientationEstimatorCreator
{
   private static final int VECTOR3D_LENGTH = 3;

   private final RigidBody orientationEstimationLink;
   private final ControlFlowOutputPort<FullInverseDynamicsStructure> inverseDynamicsStructureOutputPort;

   private final DenseMatrix64F angularAccelerationNoiseCovariance;

   private final List<OrientationSensorConfiguration> orientationSensorConfigurations = new ArrayList<OrientationSensorConfiguration>();
   private final List<AngularVelocitySensorConfiguration> angularVelocitySensorConfigurations = new ArrayList<AngularVelocitySensorConfiguration>();

   public ComposableOrientationEstimatorCreator(DenseMatrix64F angularAccelerationNoiseCovariance, RigidBody orientationEstimationLink,
           ControlFlowOutputPort<FullInverseDynamicsStructure> inverseDynamicsStructureOutputPort)
   {
      this.angularAccelerationNoiseCovariance = angularAccelerationNoiseCovariance;
      this.orientationEstimationLink = orientationEstimationLink;
      this.inverseDynamicsStructureOutputPort = inverseDynamicsStructureOutputPort;
   }

   public void addOrientationSensorConfigurations(Collection<OrientationSensorConfiguration> orientationSensorConfigurations)
   {
      for (OrientationSensorConfiguration orientationSensorConfiguration : orientationSensorConfigurations)
      {
         this.addOrientationSensorConfiguration(orientationSensorConfiguration);
      }
   }

   public void addAngularVelocitySensorConfigurations(Collection<AngularVelocitySensorConfiguration> angularVelocitySensorConfigurations)
   {
      for (AngularVelocitySensorConfiguration angularVelocitySensorConfiguration : angularVelocitySensorConfigurations)
      {
         addAngularVelocitySensorConfiguration(angularVelocitySensorConfiguration);
      }
   }

   public void addAngularVelocitySensorConfiguration(AngularVelocitySensorConfiguration angularVelocitySensorConfiguration)
   {
      this.angularVelocitySensorConfigurations.add(angularVelocitySensorConfiguration);
   }

   public void addOrientationSensorConfiguration(OrientationSensorConfiguration orientationSensorConfiguration)
   {
      orientationSensorConfigurations.add(orientationSensorConfiguration);
   }

   public OrientationEstimatorWithPorts createOrientationEstimator(ControlFlowGraph controlFlowGraph, double controlDT, ReferenceFrame estimationFrame,
           YoVariableRegistry registry)
   {
      return new ComposableOrientationEstimator("orientationEstimator", controlDT, estimationFrame, controlFlowGraph, registry);
   }

   private class ComposableOrientationEstimator extends ComposableStateEstimator implements OrientationEstimatorWithPorts
   {
      private final ControlFlowOutputPort<FrameOrientation> orientationPort;
      private final ControlFlowOutputPort<FrameVector> angularVelocityPort;

      private final ControlFlowInputPort<FullInverseDynamicsStructure> inverseDynamicsStructureInputPort;
      private final ControlFlowInputPort<FrameVector> desiredAngularAccelerationInputPort;

      public ComposableOrientationEstimator(String name, double controlDT, ReferenceFrame estimationFrame, ControlFlowGraph controlFlowGraph,
              YoVariableRegistry parentRegistry)
      {
         super(name, controlDT, parentRegistry);

         orientationPort = new YoFrameQuaternionControlFlowOutputPort(this, name, ReferenceFrame.getWorldFrame(), parentRegistry);
         angularVelocityPort = new YoFrameVectorControlFlowOutputPort(this, name + "Omega", estimationFrame, registry);

         this.inverseDynamicsStructureInputPort = createInputPort();
         this.desiredAngularAccelerationInputPort = createInputPort();

         addOrientationProcessModelElement();
         addAngularVelocityProcessModelElement(estimationFrame, controlFlowGraph);

         for (OrientationSensorConfiguration orientationSensorConfiguration : orientationSensorConfigurations)
         {
            addOrientationSensor(estimationFrame, controlFlowGraph, orientationSensorConfiguration);
         }

         for (AngularVelocitySensorConfiguration angularVelocitySensorConfiguration : angularVelocitySensorConfigurations)
         {
            addAngularVelocitySensor(estimationFrame, controlFlowGraph, angularVelocitySensorConfiguration);
         }

         controlFlowGraph.connectElements(inverseDynamicsStructureOutputPort, inverseDynamicsStructureInputPort);

         initialize();
      }

      private void addOrientationProcessModelElement()
      {
         ProcessModelElement orientationProcessModelElement = new OrientationProcessModelElement(angularVelocityPort, orientationPort, "orientation", registry);
         addProcessModelElement(orientationPort, orientationProcessModelElement);
      }

      private void addAngularVelocityProcessModelElement(ReferenceFrame estimationFrame, ControlFlowGraph controlFlowGraph)
      {
         AngularVelocityProcessModelElement angularVelocityProcessModelElement = new AngularVelocityProcessModelElement(estimationFrame, angularVelocityPort,
               desiredAngularAccelerationInputPort, "angularVelocity", registry);

         angularVelocityProcessModelElement.setProcessNoiseCovarianceBlock(angularAccelerationNoiseCovariance);
         addProcessModelElement(angularVelocityPort, angularVelocityProcessModelElement);
      }

      private void addOrientationSensor(ReferenceFrame estimationFrame, ControlFlowGraph controlFlowGraph,
                                        OrientationSensorConfiguration orientationSensorConfiguration)
      {
         ReferenceFrame measurementFrame = orientationSensorConfiguration.getMeasurementFrame();
         ControlFlowInputPort<Matrix3d> orientationMeasurementPort = createMeasurementInputPort(VECTOR3D_LENGTH);
         String name = orientationSensorConfiguration.getName();
         DenseMatrix64F orientationNoiseCovariance = orientationSensorConfiguration.getOrientationNoiseCovariance();

         OrientationMeasurementModelElement orientationMeasurementModel = new OrientationMeasurementModelElement(orientationPort, orientationMeasurementPort,
                                                                             estimationFrame, measurementFrame, name, registry);
         orientationMeasurementModel.setNoiseCovariance(orientationNoiseCovariance);

         addMeasurementModelElement(orientationMeasurementModel);
         controlFlowGraph.connectElements(orientationSensorConfiguration.getOutputPort(), orientationMeasurementPort);
      }

      private void addAngularVelocitySensor(ReferenceFrame estimationFrame, ControlFlowGraph controlFlowGraph,
              AngularVelocitySensorConfiguration angularVelocitySensorConfiguration)
      {
         String biasName = angularVelocitySensorConfiguration.getName() + "BiasEstimate";
         ReferenceFrame measurementFrame = angularVelocitySensorConfiguration.getMeasurementFrame();
         RigidBody measurementLink = angularVelocitySensorConfiguration.getAngularVelocityMeasurementLink();
         ControlFlowInputPort<Vector3d> angularVelocityMeasurementPort = createMeasurementInputPort(VECTOR3D_LENGTH);

         ControlFlowOutputPort<FrameVector> biasPort = new YoFrameVectorControlFlowOutputPort(this, biasName, measurementFrame, registry);
         BiasProcessModelElement biasProcessModelElement = new BiasProcessModelElement(biasPort, measurementFrame, biasName, registry);
         DenseMatrix64F biasProcessNoiseCovariance = angularVelocitySensorConfiguration.getBiasProcessNoiseCovariance();
         biasProcessModelElement.setProcessNoiseCovarianceBlock(biasProcessNoiseCovariance);
         addProcessModelElement(biasPort, biasProcessModelElement);
         String name = angularVelocitySensorConfiguration.getName();
         DenseMatrix64F angularVelocityNoiseCovariance = angularVelocitySensorConfiguration.getAngularVelocityNoiseCovariance();

         AngularVelocityMeasurementModelElement angularVelocityMeasurementModel = new AngularVelocityMeasurementModelElement(angularVelocityPort, biasPort,
                                                                                     angularVelocityMeasurementPort, orientationEstimationLink,
                                                                                     estimationFrame, measurementLink, measurementFrame,
                                                                                     inverseDynamicsStructureInputPort, name, registry);
         angularVelocityMeasurementModel.setNoiseCovariance(angularVelocityNoiseCovariance);

         addMeasurementModelElement(angularVelocityMeasurementModel);
         controlFlowGraph.connectElements(angularVelocitySensorConfiguration.getOutputPort(), angularVelocityMeasurementPort);
      }

      public FrameOrientation getEstimatedOrientation()
      {
         return orientationPort.getData();
      }

      public FrameVector getEstimatedAngularVelocity()
      {
         return angularVelocityPort.getData();
      }

      public FramePoint getEstimatedCoMPosition()
      {
         return null;
      }

      public FrameVector getEstimatedCoMVelocity()
      {
         return null;
      }

      public void setEstimatedOrientation(FrameOrientation orientation)
      {
         orientationPort.setData(orientation);
      }

      public void setEstimatedAngularVelocity(FrameVector angularVelocity)
      {
         angularVelocityPort.setData(angularVelocity);
      }

      public void setEstimatedCoMPosition(FramePoint estimatedCoMPosition)
      {
         // do nothing
      }

      public void setEstimatedCoMVelocity(FrameVector estimatedCoMVelocity)
      {
         // do nothing
      }

      public DenseMatrix64F getCovariance()
      {
         return kalmanFilter.getCovariance();
      }

      public DenseMatrix64F getState()
      {
         return kalmanFilter.getState();
      }

      public void setState(DenseMatrix64F x, DenseMatrix64F covariance)
      {
         kalmanFilter.setState(x, covariance);
      }

      public ControlFlowInputPort<FrameVector> getDesiredAngularAccelerationInputPort()
      {
         return desiredAngularAccelerationInputPort;
      }

      public ControlFlowInputPort<FrameVector> getDesiredCenterOfMassAccelerationInputPort()
      {
         return null;
      }

   }
}
