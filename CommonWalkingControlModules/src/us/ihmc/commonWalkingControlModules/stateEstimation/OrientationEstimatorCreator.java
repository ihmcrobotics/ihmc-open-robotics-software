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

   public ComposableStateEstimator createOrientationEstimator(ControlFlowGraph controlFlowGraph, double controlDT, ReferenceFrame estimationFrame, YoVariableRegistry registry)
   {
      ComposableStateEstimator ret = new ComposableStateEstimator("orientationEstimator", controlDT);

      // states
      ControlFlowOutputPort<FrameOrientation> orientationPort = ret.createStatePort(3);
      ControlFlowOutputPort<FrameVector> angularVelocityPort = ret.createStatePort(3);

      // process inputs
      ControlFlowInputPort<FrameVector> angularAccelerationPort = ret.createProcessInputPort(3);

      // measurement ports will be created with the measurement models below...

      // process model
      ProcessModelElement orientationProcessModelElement = new OrientationProcessModelElement(angularVelocityPort, orientationPort, "orientation", registry);
      ret.addProcessModelElement(orientationPort, orientationProcessModelElement);

      AngularVelocityProcessModelElement angularVelocityProcessModelElement = new AngularVelocityProcessModelElement(estimationFrame, angularVelocityPort,
                                                                                 angularAccelerationPort, "angularVelocity", registry);

      angularVelocityProcessModelElement.setProcessNoiseCovarianceBlock(angularAccelerationNoiseCovariance);
      ret.addProcessModelElement(angularVelocityPort, angularVelocityProcessModelElement);


      // measurement model
      for (NewOrientationSensorConfiguration orientationSensorConfiguration : orientationSensorConfigurations)
      {
         ReferenceFrame measurementFrame = orientationSensorConfiguration.getMeasurementFrame();
         ControlFlowInputPort<Matrix3d> orientationMeasurementPort = ret.createMeasurementInputPort(3);
         controlFlowGraph.connectElements(orientationSensorConfiguration.getOutputPort(), orientationMeasurementPort);
         orientationMeasurementPorts.add(orientationMeasurementPort);

         OrientationMeasurementModelElement orientationMeasurementModel = new OrientationMeasurementModelElement(orientationPort, orientationMeasurementPort,
                                                                             estimationFrame, measurementFrame, orientationSensorConfiguration.getName(),
                                                                             registry);

         DenseMatrix64F orientationNoiseCovariance = orientationSensorConfiguration.getOrientationNoiseCovariance();
         orientationMeasurementModel.setNoiseCovariance(orientationNoiseCovariance);
         ret.addMeasurementModelElement(orientationMeasurementPort, orientationMeasurementModel);
      }

      for (NewAngularVelocitySensorConfiguration angularVelocitySensorConfiguration : angularVelocitySensorConfigurations)
      {
         ControlFlowOutputPort<FrameVector> biasPort = ret.createStatePort(3);
         BiasProcessModelElement biasProcessModelElement = new BiasProcessModelElement(biasPort, angularVelocitySensorConfiguration.getName() + "Bias",
                                                              registry);
         DenseMatrix64F biasProcessNoiseCovariance = angularVelocitySensorConfiguration.getBiasProcessNoiseCovariance();
         biasProcessModelElement.setProcessNoiseCovarianceBlock(biasProcessNoiseCovariance);
         ret.addProcessModelElement(biasPort, biasProcessModelElement);

         RigidBody measurementLink = angularVelocitySensorConfiguration.getAngularVelocityMeasurementLink();
         ReferenceFrame measurementFrame = angularVelocitySensorConfiguration.getMeasurementFrame();
         ControlFlowInputPort<Vector3d> angularVelocityMeasurementPort = ret.createMeasurementInputPort(3);
         controlFlowGraph.connectElements(angularVelocitySensorConfiguration.getOutputPort(), angularVelocityMeasurementPort);
         angularVelocityMeasurementPorts.add(angularVelocityMeasurementPort);

         AngularVelocityMeasurementModelElement angularVelocityMeasurementModel = new AngularVelocityMeasurementModelElement(angularVelocityPort, biasPort,
                                                                                     angularVelocityMeasurementPort, orientationEstimationLink,
                                                                                     measurementLink, measurementFrame, twistCalculator,
                                                                                     angularVelocitySensorConfiguration.getName(), registry);

         DenseMatrix64F angularVelocityNoiseCovariance = angularVelocitySensorConfiguration.getAngularVelocityNoiseCovariance();
         angularVelocityMeasurementModel.setNoiseCovariance(angularVelocityNoiseCovariance);
         ret.addMeasurementModelElement(angularVelocityMeasurementPort, angularVelocityMeasurementModel);
      }

      ret.initialize();
      return ret;
   }
}
