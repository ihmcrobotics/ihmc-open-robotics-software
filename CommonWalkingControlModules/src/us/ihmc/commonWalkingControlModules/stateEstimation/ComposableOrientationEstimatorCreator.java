package us.ihmc.commonWalkingControlModules.stateEstimation;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.stateEstimation.measurementModelElements.AngularVelocityMeasurementModelElement;
import us.ihmc.commonWalkingControlModules.stateEstimation.measurementModelElements.OrientationMeasurementModelElement;
import us.ihmc.commonWalkingControlModules.stateEstimation.processModelElements.AngularVelocityProcessModelElement;
import us.ihmc.commonWalkingControlModules.stateEstimation.processModelElements.BiasProcessModelElement;
import us.ihmc.commonWalkingControlModules.stateEstimation.processModelElements.OrientationProcessModelElement;
import us.ihmc.controlFlow.ControlFlowGraph;
import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class ComposableOrientationEstimatorCreator
{
   private final RigidBody orientationEstimationLink;
   private final TwistCalculator twistCalculator;

   private final DenseMatrix64F angularAccelerationNoiseCovariance;

   private final List<NewOrientationSensorConfiguration> orientationSensorConfigurations = new ArrayList<NewOrientationSensorConfiguration>();
   private final List<NewAngularVelocitySensorConfiguration> angularVelocitySensorConfigurations = new ArrayList<NewAngularVelocitySensorConfiguration>();

   public ComposableOrientationEstimatorCreator(DenseMatrix64F angularAccelerationNoiseCovariance, RigidBody orientationEstimationLink, TwistCalculator twistCalculator)
   {
      this.angularAccelerationNoiseCovariance = angularAccelerationNoiseCovariance;
      this.orientationEstimationLink = orientationEstimationLink;
      this.twistCalculator = twistCalculator;
   }

   public void addOrientationSensorConfigurations(OrientationSensorConfiguration<?> orientationSensorConfigurations)
   {
      ArrayList<NewOrientationSensorConfiguration> newOrientationSensorConfigurations = orientationSensorConfigurations.getNewOrientationSensorConfiguration();
      for (NewOrientationSensorConfiguration orientationSensorConfiguration : newOrientationSensorConfigurations)
      {
         this.addOrientationSensorConfiguration(orientationSensorConfiguration);
      }
   }

   public void addAngularVelocitySensorConfigurations(AngularVelocitySensorConfiguration<?> angularVelocitySensorConfigurations)
   {
      ArrayList<NewAngularVelocitySensorConfiguration> newAngularVelocitySensorConfigurations =
         angularVelocitySensorConfigurations.getNewAngularVelocitySensorConfiguration();
      for (NewAngularVelocitySensorConfiguration angularVelocitySensorConfiguration : newAngularVelocitySensorConfigurations)
      {
         addAngularVelocitySensorConfiguration(angularVelocitySensorConfiguration);
      }
   }

   public void addAngularVelocitySensorConfiguration(NewAngularVelocitySensorConfiguration angularVelocitySensorConfiguration)
   {
      this.angularVelocitySensorConfigurations.add(angularVelocitySensorConfiguration);
   }

   public void addOrientationSensorConfiguration(NewOrientationSensorConfiguration orientationSensorConfiguration)
   {
      orientationSensorConfigurations.add(orientationSensorConfiguration);
   }

   public OrientationEstimator createOrientationEstimator(ControlFlowGraph controlFlowGraph, double controlDT, ReferenceFrame estimationFrame,
           ControlFlowOutputPort<FrameVector> angularAccelerationOutputPort, YoVariableRegistry registry)
   {
      return new ComposableOrientationEstimator("orientationEstimator", controlDT, estimationFrame, controlFlowGraph, angularAccelerationOutputPort, registry);
   }

   private class ComposableOrientationEstimator extends ComposableStateEstimator implements OrientationEstimator
   {
      private final ControlFlowOutputPort<FrameOrientation> orientationPort;
      private final ControlFlowOutputPort<FrameVector> angularVelocityPort;

      public ComposableOrientationEstimator(String name, double controlDT, ReferenceFrame estimationFrame, ControlFlowGraph controlFlowGraph,
              ControlFlowOutputPort<FrameVector> angularAccelerationOutputPort, YoVariableRegistry parentRegistry)
      {
         super(name, controlDT, parentRegistry);

         orientationPort = new YoFrameQuaternionControlFlowOutputPort(this, name, ReferenceFrame.getWorldFrame(), parentRegistry);
         addStatePort(orientationPort, 3);

         angularVelocityPort = new YoFrameVectorControlFlowOutputPort(this, name + "Omega", estimationFrame, registry);
         addStatePort(angularVelocityPort, 3);

         addOrientationProcessModelElement();
         addAngularVelocityProcessModelElement(estimationFrame, controlFlowGraph, angularAccelerationOutputPort);

         for (NewOrientationSensorConfiguration orientationSensorConfiguration : orientationSensorConfigurations)
         {
            addOrientationSensor(estimationFrame, controlFlowGraph, orientationSensorConfiguration);
         }

         for (NewAngularVelocitySensorConfiguration angularVelocitySensorConfiguration : angularVelocitySensorConfigurations)
         {
            addAngularVelocitySensor(controlFlowGraph, angularVelocitySensorConfiguration);
         }

         initialize();
      }

      private void addAngularVelocityProcessModelElement(ReferenceFrame estimationFrame, ControlFlowGraph controlFlowGraph,
            ControlFlowOutputPort<FrameVector> angularAccelerationOutputPort)
      {
         ControlFlowInputPort<FrameVector> angularAccelerationPort;
         if (angularAccelerationOutputPort != null)
         {
            angularAccelerationPort = createProcessInputPort(3);
            controlFlowGraph.connectElements(angularAccelerationOutputPort, angularAccelerationPort);
         }
         else
            angularAccelerationPort = null;

         AngularVelocityProcessModelElement angularVelocityProcessModelElement = new AngularVelocityProcessModelElement(estimationFrame, angularVelocityPort, angularAccelerationPort,
                 "angularVelocity", registry);

         angularVelocityProcessModelElement.setProcessNoiseCovarianceBlock(angularAccelerationNoiseCovariance);
         addProcessModelElement(angularVelocityPort, angularVelocityProcessModelElement);
      }

      private void addOrientationProcessModelElement()
      {
         ProcessModelElement orientationProcessModelElement = new OrientationProcessModelElement(angularVelocityPort, orientationPort, "orientation", registry);
         addProcessModelElement(orientationPort, orientationProcessModelElement);
      }

      private void addAngularVelocitySensor(ControlFlowGraph controlFlowGraph, NewAngularVelocitySensorConfiguration angularVelocitySensorConfiguration)
      {
         String biasName = angularVelocitySensorConfiguration.getName() + "BiasEstimate";
         ReferenceFrame measurementFrame = angularVelocitySensorConfiguration.getMeasurementFrame();
         RigidBody measurementLink = angularVelocitySensorConfiguration.getAngularVelocityMeasurementLink();
         ControlFlowInputPort<Vector3d> angularVelocityMeasurementPort = createMeasurementInputPort(3);

         ControlFlowOutputPort<FrameVector> biasPort = new YoFrameVectorControlFlowOutputPort(this, biasName, measurementFrame, registry);
         addStatePort(biasPort, 3);

         BiasProcessModelElement biasProcessModelElement = new BiasProcessModelElement(biasPort, measurementFrame, biasName, registry);
         DenseMatrix64F biasProcessNoiseCovariance = angularVelocitySensorConfiguration.getBiasProcessNoiseCovariance();
         biasProcessModelElement.setProcessNoiseCovarianceBlock(biasProcessNoiseCovariance);
         addProcessModelElement(biasPort, biasProcessModelElement);
         String name = angularVelocitySensorConfiguration.getName();
         DenseMatrix64F angularVelocityNoiseCovariance = angularVelocitySensorConfiguration.getAngularVelocityNoiseCovariance();

         AngularVelocityMeasurementModelElement angularVelocityMeasurementModel = new AngularVelocityMeasurementModelElement(angularVelocityPort, biasPort,
                                                                                     angularVelocityMeasurementPort, orientationEstimationLink,
                                                                                     measurementLink, measurementFrame, twistCalculator,
                                                                                     name, registry);
         angularVelocityMeasurementModel.setNoiseCovariance(angularVelocityNoiseCovariance);

         addMeasurementModelElement(angularVelocityMeasurementPort, angularVelocityMeasurementModel);
         controlFlowGraph.connectElements(angularVelocitySensorConfiguration.getOutputPort(), angularVelocityMeasurementPort);
      }

      private void addOrientationSensor(ReferenceFrame estimationFrame, ControlFlowGraph controlFlowGraph,
            NewOrientationSensorConfiguration orientationSensorConfiguration)
      {
         ReferenceFrame measurementFrame = orientationSensorConfiguration.getMeasurementFrame();
         ControlFlowInputPort<Matrix3d> orientationMeasurementPort = createMeasurementInputPort(3);
         String name = orientationSensorConfiguration.getName();
         DenseMatrix64F orientationNoiseCovariance = orientationSensorConfiguration.getOrientationNoiseCovariance();

         OrientationMeasurementModelElement orientationMeasurementModel = new OrientationMeasurementModelElement(orientationPort,
                                                                             orientationMeasurementPort, estimationFrame, measurementFrame,
                                                                             name, registry);
         orientationMeasurementModel.setNoiseCovariance(orientationNoiseCovariance);

         addMeasurementModelElement(orientationMeasurementPort, orientationMeasurementModel);
         controlFlowGraph.connectElements(orientationSensorConfiguration.getOutputPort(), orientationMeasurementPort);
      }

      public FrameOrientation getEstimatedOrientation()
      {
         return orientationPort.getData();
      }

      public FrameVector getEstimatedAngularVelocity()
      {
         return angularVelocityPort.getData();
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
   }
}
