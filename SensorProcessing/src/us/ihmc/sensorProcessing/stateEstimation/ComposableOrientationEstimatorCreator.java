package us.ihmc.sensorProcessing.stateEstimation;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.controlFlow.ControlFlowGraph;
import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.sensorProcessing.controlFlowPorts.YoFrameQuaternionControlFlowOutputPort;
import us.ihmc.sensorProcessing.controlFlowPorts.YoFrameVectorControlFlowOutputPort;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.sensorProcessing.stateEstimation.measurementModelElements.AngularVelocityMeasurementModelElement;
import us.ihmc.sensorProcessing.stateEstimation.measurementModelElements.OrientationMeasurementModelElement;
import us.ihmc.sensorProcessing.stateEstimation.processModelElements.AngularVelocityProcessModelElement;
import us.ihmc.sensorProcessing.stateEstimation.processModelElements.BiasProcessModelElement;
import us.ihmc.sensorProcessing.stateEstimation.processModelElements.OrientationProcessModelElement;
import us.ihmc.sensorProcessing.stateEstimation.processModelElements.ProcessModelElement;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.AngularVelocitySensorConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.OrientationSensorConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.PointPositionDataObject;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.PointVelocityDataObject;

public class ComposableOrientationEstimatorCreator
{
//   private static final int VECTOR3D_LENGTH = 3;

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

   public StateEstimatorWithPorts createOrientationEstimator(ControlFlowGraph controlFlowGraph, double controlDT, ReferenceFrame estimationFrame,
           YoVariableRegistry registry)
   {
      return new ComposableOrientationEstimator("orientationEstimator", controlDT, estimationFrame, controlFlowGraph, registry);
   }

   private class ComposableOrientationEstimator extends ComposableStateEstimator implements StateEstimatorWithPorts
   {
      private final ControlFlowGraph controlFlowGraph;
      
      private final ControlFlowOutputPort<FrameOrientation> orientationStatePort;
      private final ControlFlowOutputPort<FrameVector> angularVelocityStatePort;

      private final ControlFlowInputPort<FullInverseDynamicsStructure> inverseDynamicsStructureInputPort;
      private final ControlFlowInputPort<FrameVector> desiredAngularAccelerationInputPort;
      private final ReferenceFrame estimationFrame;

      public ComposableOrientationEstimator(String name, double controlDT, ReferenceFrame estimationFrame, ControlFlowGraph controlFlowGraph,
              YoVariableRegistry parentRegistry)
      {
         super(name, controlDT, parentRegistry);

         this.estimationFrame = estimationFrame;
         this.controlFlowGraph = controlFlowGraph;
         
         orientationStatePort = new YoFrameQuaternionControlFlowOutputPort(this, name, ReferenceFrame.getWorldFrame(), parentRegistry);
         angularVelocityStatePort = new YoFrameVectorControlFlowOutputPort(this, name + "Omega", estimationFrame, registry);

         this.inverseDynamicsStructureInputPort = createInputPort("inverseDynamicsStructureInputPort");
         this.desiredAngularAccelerationInputPort = createInputPort("desiredAngularAccelerationInputPort");

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
         ProcessModelElement orientationProcessModelElement = new OrientationProcessModelElement(angularVelocityStatePort, orientationStatePort, "orientation", registry);
         addProcessModelElement(orientationStatePort, orientationProcessModelElement);
      }

      private void addAngularVelocityProcessModelElement(ReferenceFrame estimationFrame, ControlFlowGraph controlFlowGraph)
      {
         AngularVelocityProcessModelElement angularVelocityProcessModelElement = new AngularVelocityProcessModelElement(estimationFrame, angularVelocityStatePort,
               desiredAngularAccelerationInputPort, "angularVelocity", registry);

         angularVelocityProcessModelElement.setProcessNoiseCovarianceBlock(angularAccelerationNoiseCovariance);
         addProcessModelElement(angularVelocityStatePort, angularVelocityProcessModelElement);
      }

      private void addOrientationSensor(ReferenceFrame estimationFrame, ControlFlowGraph controlFlowGraph,
                                        OrientationSensorConfiguration orientationSensorConfiguration)
      {
         ReferenceFrame measurementFrame = orientationSensorConfiguration.getMeasurementFrame();
         ControlFlowInputPort<RotationMatrix> measurementInputPort = createInputPort("orientationMeasurementInputPort");

         ControlFlowInputPort<RotationMatrix> orientationMeasurementPort = measurementInputPort;
         String name = orientationSensorConfiguration.getName();
         DenseMatrix64F orientationNoiseCovariance = orientationSensorConfiguration.getOrientationNoiseCovariance();

         OrientationMeasurementModelElement orientationMeasurementModel = new OrientationMeasurementModelElement(orientationStatePort, orientationMeasurementPort,
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
         ControlFlowInputPort<Vector3D> measurementInputPort = createInputPort("angularVelocityMeasurementInputPort");

         ControlFlowInputPort<Vector3D> angularVelocityMeasurementPort = measurementInputPort;

         ControlFlowOutputPort<FrameVector> biasPort = new YoFrameVectorControlFlowOutputPort(this, biasName, measurementFrame, registry);
         BiasProcessModelElement biasProcessModelElement = new BiasProcessModelElement(biasPort, measurementFrame, biasName, registry);
         DenseMatrix64F biasProcessNoiseCovariance = angularVelocitySensorConfiguration.getBiasProcessNoiseCovariance();
         biasProcessModelElement.setProcessNoiseCovarianceBlock(biasProcessNoiseCovariance);
         addProcessModelElement(biasPort, biasProcessModelElement);
         String name = angularVelocitySensorConfiguration.getName();
         DenseMatrix64F angularVelocityNoiseCovariance = angularVelocitySensorConfiguration.getAngularVelocityNoiseCovariance();

         AngularVelocityMeasurementModelElement angularVelocityMeasurementModel = new AngularVelocityMeasurementModelElement(angularVelocityStatePort, biasPort,
                                                                                     angularVelocityMeasurementPort, orientationEstimationLink,
                                                                                     estimationFrame, measurementLink, measurementFrame,
                                                                                     inverseDynamicsStructureInputPort, name, registry);
         angularVelocityMeasurementModel.setNoiseCovariance(angularVelocityNoiseCovariance);

         addMeasurementModelElement(angularVelocityMeasurementModel);
         controlFlowGraph.connectElements(angularVelocitySensorConfiguration.getOutputPort(), angularVelocityMeasurementPort);
      }

      public void getEstimatedOrientation(FrameOrientation estimatedOrientationToPack)
      {
         estimatedOrientationToPack.setIncludingFrame(orientationStatePort.getData());
      }

      public void getEstimatedAngularVelocity(FrameVector estimatedAngularVelocityToPack)
      {
         estimatedAngularVelocityToPack.setIncludingFrame(angularVelocityStatePort.getData());
      }

      public void getEstimatedCoMPosition(FramePoint estimatedCoMPositionToPack)
      {
         // Do nothing.
      }
      
      public void getEstimatedCoMVelocity(FrameVector estimatedCoMVelocityToPack)
      {
         // Do nothing.
      }

      public void setEstimatedOrientation(FrameOrientation orientation)
      {
         orientationStatePort.setData(orientation);
      }

      public void setEstimatedAngularVelocity(FrameVector angularVelocity)
      {
         angularVelocityStatePort.setData(angularVelocity);
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

      public void initializeOrientationEstimateToMeasurement()
      {
         // R^W_M
         OrientationSensorConfiguration firstOrientationSensorConfiguration = orientationSensorConfigurations.get(0);
         ControlFlowOutputPort<RotationMatrix> firstOrientationMeasurementOutputPort = firstOrientationSensorConfiguration.getOutputPort();
         RotationMatrix measurementToWorld = firstOrientationMeasurementOutputPort.getData();

         // R^M_E
         ReferenceFrame measurementFrame = firstOrientationSensorConfiguration.getMeasurementFrame();
         FrameOrientation estimationFrameOrientation = new FrameOrientation(estimationFrame);
         estimationFrameOrientation.changeFrame(measurementFrame);
         RotationMatrix estimationToMeasurement = estimationFrameOrientation.getMatrix3dCopy();

         // R^W_E
         RotationMatrix estimationToWorld = new RotationMatrix();
         estimationToWorld.set(measurementToWorld);
         estimationToWorld.multiply(estimationToMeasurement);
         FrameOrientation initialOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame(), estimationToWorld);
         setEstimatedOrientation(initialOrientation);
      }

      public ControlFlowInputPort<FrameVector> getDesiredAngularAccelerationInputPort()
      {
         return desiredAngularAccelerationInputPort;
      }

      public ControlFlowInputPort<FrameVector> getDesiredCenterOfMassAccelerationInputPort()
      {
         return null;
      }

      public ControlFlowInputPort<List<PointPositionDataObject>> getPointPositionInputPort()
      {
         return null;
      }
      
      public ControlFlowInputPort<List<PointVelocityDataObject>> getPointVelocityInputPort()
      {
         return null;
      }

      public ControlFlowGraph getControlFlowGraph()
      {
         return controlFlowGraph;
      }

      public void getEstimatedPelvisPosition(FramePoint estimatedPelvisPositionToPack)
      {
         throw new RuntimeException("No pelvis position estimated with this state estimator (" + getClass().getSimpleName() + ").");
      }

      public void getEstimatedPelvisLinearVelocity(FrameVector estimatedPelvisLinearVelocityToPack)
      {
         throw new RuntimeException("No pelvis velocity estimated with this state estimator (" + getClass().getSimpleName() + ").");
      }

   }
}
