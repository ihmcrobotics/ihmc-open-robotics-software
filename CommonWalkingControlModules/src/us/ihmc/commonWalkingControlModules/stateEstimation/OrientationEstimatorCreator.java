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

public class OrientationEstimatorCreator
{
   private final RigidBody orientationEstimationLink;
   private final TwistCalculator twistCalculator;

   private final DenseMatrix64F angularAccelerationNoiseCovariance;

   private final List<NewOrientationSensorConfiguration> orientationSensorConfigurations = new ArrayList<NewOrientationSensorConfiguration>();
   private final List<NewAngularVelocitySensorConfiguration> angularVelocitySensorConfigurations = new ArrayList<NewAngularVelocitySensorConfiguration>();

   private final List<ControlFlowInputPort<Vector3d>> angularVelocityMeasurementPorts = new ArrayList<ControlFlowInputPort<Vector3d>>();
   private final List<ControlFlowInputPort<Matrix3d>> orientationMeasurementPorts = new ArrayList<ControlFlowInputPort<Matrix3d>>();

   public OrientationEstimatorCreator(DenseMatrix64F angularAccelerationNoiseCovariance, RigidBody orientationEstimationLink, TwistCalculator twistCalculator)
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

   public void addOrientationSensorConfiguration(NewOrientationSensorConfiguration orientationSensorConfiguration)
   {
      orientationSensorConfigurations.add(orientationSensorConfiguration);
   }

   public void addAngularVelocitySensorConfiguration(NewAngularVelocitySensorConfiguration angularVelocitySensorConfiguration)
   {
      this.angularVelocitySensorConfigurations.add(angularVelocitySensorConfiguration);
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
      private final AngularVelocityProcessModelElement angularVelocityProcessModelElement;

      public ComposableOrientationEstimator(String name, double controlDT, ReferenceFrame estimationFrame, ControlFlowGraph controlFlowGraph,
              ControlFlowOutputPort<FrameVector> angularAccelerationOutputPort, YoVariableRegistry parentRegistry)
      {
         super(name, controlDT, parentRegistry);

         orientationPort = new YoFrameQuaternionControlFlowOutputPort(this, name, ReferenceFrame.getWorldFrame(), parentRegistry);
         addStatePort(orientationPort, 3);

         angularVelocityPort = new YoFrameVectorControlFlowOutputPort(this, name + "Omega", estimationFrame, registry);
         addStatePort(angularVelocityPort, 3);

         // process model
         ProcessModelElement orientationProcessModelElement = new OrientationProcessModelElement(angularVelocityPort, orientationPort, "orientation", registry);
         addProcessModelElement(orientationPort, orientationProcessModelElement);

         ControlFlowInputPort<FrameVector> angularAccelerationPort;
         if (angularAccelerationOutputPort != null)
         {
            angularAccelerationPort = createProcessInputPort(3);
            controlFlowGraph.connectElements(angularAccelerationOutputPort, angularAccelerationPort);
         }
         else
            angularAccelerationPort = null;

         angularVelocityProcessModelElement = new AngularVelocityProcessModelElement(estimationFrame, angularVelocityPort, angularAccelerationPort,
                 "angularVelocity", registry);

         angularVelocityProcessModelElement.setProcessNoiseCovarianceBlock(angularAccelerationNoiseCovariance);
         addProcessModelElement(angularVelocityPort, angularVelocityProcessModelElement);


         // measurement model
         for (NewOrientationSensorConfiguration orientationSensorConfiguration : orientationSensorConfigurations)
         {
            ReferenceFrame measurementFrame = orientationSensorConfiguration.getMeasurementFrame();
            ControlFlowInputPort<Matrix3d> orientationMeasurementPort = createMeasurementInputPort(3);
            controlFlowGraph.connectElements(orientationSensorConfiguration.getOutputPort(), orientationMeasurementPort);
            orientationMeasurementPorts.add(orientationMeasurementPort);

            OrientationMeasurementModelElement orientationMeasurementModel = new OrientationMeasurementModelElement(orientationPort,
                                                                                orientationMeasurementPort, estimationFrame, measurementFrame,
                                                                                orientationSensorConfiguration.getName(), registry);

            DenseMatrix64F orientationNoiseCovariance = orientationSensorConfiguration.getOrientationNoiseCovariance();
            orientationMeasurementModel.setNoiseCovariance(orientationNoiseCovariance);
            addMeasurementModelElement(orientationMeasurementPort, orientationMeasurementModel);
         }

         for (NewAngularVelocitySensorConfiguration angularVelocitySensorConfiguration : angularVelocitySensorConfigurations)
         {
            String biasName = angularVelocitySensorConfiguration.getName() + "BiasEstimate";
            ReferenceFrame measurementFrame = angularVelocitySensorConfiguration.getMeasurementFrame();
            RigidBody measurementLink = angularVelocitySensorConfiguration.getAngularVelocityMeasurementLink();
            ControlFlowInputPort<Vector3d> angularVelocityMeasurementPort = createMeasurementInputPort(3);

            ControlFlowOutputPort<FrameVector> biasPort = new YoFrameVectorControlFlowOutputPort(this, biasName, measurementFrame, registry);
            addStatePort(biasPort, 3);

            biasPort.setData(new FrameVector(measurementFrame));
            BiasProcessModelElement biasProcessModelElement = new BiasProcessModelElement(biasPort, measurementFrame, biasName, registry);
            DenseMatrix64F biasProcessNoiseCovariance = angularVelocitySensorConfiguration.getBiasProcessNoiseCovariance();
            biasProcessModelElement.setProcessNoiseCovarianceBlock(biasProcessNoiseCovariance);
            addProcessModelElement(biasPort, biasProcessModelElement);

            controlFlowGraph.connectElements(angularVelocitySensorConfiguration.getOutputPort(), angularVelocityMeasurementPort);
            angularVelocityMeasurementPorts.add(angularVelocityMeasurementPort);

            AngularVelocityMeasurementModelElement angularVelocityMeasurementModel = new AngularVelocityMeasurementModelElement(angularVelocityPort, biasPort,
                                                                                        angularVelocityMeasurementPort, orientationEstimationLink,
                                                                                        measurementLink, measurementFrame, twistCalculator,
                                                                                        angularVelocitySensorConfiguration.getName(), registry);

            DenseMatrix64F angularVelocityNoiseCovariance = angularVelocitySensorConfiguration.getAngularVelocityNoiseCovariance();
            angularVelocityMeasurementModel.setNoiseCovariance(angularVelocityNoiseCovariance);
            addMeasurementModelElement(angularVelocityMeasurementPort, angularVelocityMeasurementModel);
         }

         initialize();
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

      public void setAngularAccelerationNoiseCovariance(DenseMatrix64F angularAccelerationNoiseCovariance)
      {
         angularVelocityProcessModelElement.setProcessNoiseCovarianceBlock(angularAccelerationNoiseCovariance);
      }
   }


}
