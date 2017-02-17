package us.ihmc.sensorProcessing.stateEstimation.evaluation;

import java.util.Collection;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.controlFlow.ControlFlowGraph;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.AfterJointReferenceFrameNameMap;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.sensorProcessing.simulatedSensors.JointAndIMUSensorMap;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.stateEstimation.ComposableOrientationAndCoMEstimatorCreator;
import us.ihmc.sensorProcessing.stateEstimation.IMUSelectorAndDataConverter;
import us.ihmc.sensorProcessing.stateEstimation.JointStateFullRobotModelUpdater;
import us.ihmc.sensorProcessing.stateEstimation.OrientationStateRobotModelUpdater;
import us.ihmc.sensorProcessing.stateEstimation.PointMeasurementNoiseParameters;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimationDataFromController;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorWithPorts;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.AngularVelocitySensorConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.LinearAccelerationSensorConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.OrientationSensorConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.SensorConfigurationFactory;

public class SensorAndEstimatorAssembler
{
   private static final boolean VISUALIZE_CONTROL_FLOW_GRAPH = false; //false;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ControlFlowGraph controlFlowGraph;

   // The following are the elements added to the controlFlowGraph:
   @SuppressWarnings("unused")
   private final StateEstimationDataFromController stateEstimatorDataFromControllerSource;
   private final JointStateFullRobotModelUpdater jointStateFullRobotModelUpdater;
   private final ComposableOrientationAndCoMEstimatorCreator.ComposableOrientationAndCoMEstimator stateEstimator;
   private final OrientationStateRobotModelUpdater orientationStateRobotModelUpdater;
   private final IMUSelectorAndDataConverter imuSelectorAndDataConverter;

   private final boolean assumePerfectIMU;

   public SensorAndEstimatorAssembler(StateEstimationDataFromController stateEstimatorDataFromControllerSource,
         JointAndIMUSensorMap jointAndIMUSensorMap, StateEstimatorParameters stateEstimatorParameters, double gravitationalAcceleration,
         FullInverseDynamicsStructure inverseDynamicsStructure, AfterJointReferenceFrameNameMap estimatorReferenceFrameMap,
         RigidBodyToIndexMap estimatorRigidBodyToIndexMap, YoVariableRegistry parentRegistry)
   {
      this.assumePerfectIMU = true; //stateEstimatorParameters.getAssumePerfectIMU();
      double estimatorDT = stateEstimatorParameters.getEstimatorDT();

      SensorNoiseParameters sensorNoiseParametersForEstimator = stateEstimatorParameters.getSensorNoiseParameters();
      // broken
      PointMeasurementNoiseParameters pointMeasurementNoiseParameters = null; //stateEstimatorParameters.getPointMeasurementNoiseParameters();
      
      this.stateEstimatorDataFromControllerSource = stateEstimatorDataFromControllerSource;
      SensorConfigurationFactory sensorConfigurationFactory = new SensorConfigurationFactory(sensorNoiseParametersForEstimator, gravitationalAcceleration);

      ReferenceFrame estimationFrame = inverseDynamicsStructure.getEstimationFrame();

      // Sensor configurations for estimator
      Collection<OrientationSensorConfiguration> orientationSensorConfigurations = sensorConfigurationFactory
            .createOrientationSensorConfigurations(jointAndIMUSensorMap.getOrientationSensors());

      Collection<AngularVelocitySensorConfiguration> angularVelocitySensorConfigurations = sensorConfigurationFactory
            .createAngularVelocitySensorConfigurations(jointAndIMUSensorMap.getAngularVelocitySensors());

      Collection<LinearAccelerationSensorConfiguration> linearAccelerationSensorConfigurations = sensorConfigurationFactory
            .createLinearAccelerationSensorConfigurations(jointAndIMUSensorMap.getLinearAccelerationSensors());

      controlFlowGraph = new ControlFlowGraph();
      jointStateFullRobotModelUpdater = new JointStateFullRobotModelUpdater(controlFlowGraph, jointAndIMUSensorMap, inverseDynamicsStructure);

      ControlFlowOutputPort<FullInverseDynamicsStructure> inverseDynamicsStructureOutputPort = null;

      if (!assumePerfectIMU)
      {
         imuSelectorAndDataConverter = null;
         orientationStateRobotModelUpdater = null;
         inverseDynamicsStructureOutputPort = jointStateFullRobotModelUpdater.getInverseDynamicsStructureOutputPort();
      }
      else
      {
         imuSelectorAndDataConverter = new IMUSelectorAndDataConverter(controlFlowGraph, orientationSensorConfigurations, angularVelocitySensorConfigurations,
               jointStateFullRobotModelUpdater.getInverseDynamicsStructureOutputPort(), estimatorDT, registry);

         orientationStateRobotModelUpdater = new OrientationStateRobotModelUpdater(controlFlowGraph,
               imuSelectorAndDataConverter.getInverseDynamicsStructureOutputPort(), imuSelectorAndDataConverter.getOrientationOutputPort(),
               imuSelectorAndDataConverter.getAngularVelocityOutputPort());

         inverseDynamicsStructureOutputPort = orientationStateRobotModelUpdater.getInverseDynamicsStructureOutputPort();
      }

      double angularAccelerationProcessNoiseStandardDeviation = sensorNoiseParametersForEstimator.getAngularAccelerationProcessNoiseStandardDeviation();
      DenseMatrix64F angularAccelerationNoiseCovariance = createDiagonalCovarianceMatrix(angularAccelerationProcessNoiseStandardDeviation, 3);

      RigidBody estimationLink = inverseDynamicsStructure.getEstimationLink();

      double comAccelerationProcessNoiseStandardDeviation = sensorNoiseParametersForEstimator.getComAccelerationProcessNoiseStandardDeviation();
      DenseMatrix64F comAccelerationNoiseCovariance = createDiagonalCovarianceMatrix(comAccelerationProcessNoiseStandardDeviation, 3);

      ComposableOrientationAndCoMEstimatorCreator orientationAndCoMEstimatorCreator = new ComposableOrientationAndCoMEstimatorCreator(
            pointMeasurementNoiseParameters, angularAccelerationNoiseCovariance, comAccelerationNoiseCovariance, estimationLink,
            inverseDynamicsStructureOutputPort, assumePerfectIMU);

      if (!assumePerfectIMU)
      {
         orientationAndCoMEstimatorCreator.addOrientationSensorConfigurations(orientationSensorConfigurations);
         orientationAndCoMEstimatorCreator.addAngularVelocitySensorConfigurations(angularVelocitySensorConfigurations);
         orientationAndCoMEstimatorCreator.addLinearAccelerationSensorConfigurations(linearAccelerationSensorConfigurations);
      }

      stateEstimator = orientationAndCoMEstimatorCreator.createOrientationAndCoMEstimator(controlFlowGraph, estimatorDT, estimationFrame,
            estimatorReferenceFrameMap, estimatorRigidBodyToIndexMap, registry);
      stateEstimatorDataFromControllerSource.connectDesiredAccelerationPorts(controlFlowGraph, stateEstimator);

      parentRegistry.addChild(registry);

   }

   public void initialize()
   {
      controlFlowGraph.initializeAfterConnections();

      if (VISUALIZE_CONTROL_FLOW_GRAPH)
      {
         controlFlowGraph.visualize();
      }
   }

   private static DenseMatrix64F createDiagonalCovarianceMatrix(double standardDeviation, int size)
   {
      DenseMatrix64F orientationCovarianceMatrix = new DenseMatrix64F(size, size);
      CommonOps.setIdentity(orientationCovarianceMatrix);
      CommonOps.scale(MathTools.square(standardDeviation), orientationCovarianceMatrix);

      return orientationCovarianceMatrix;
   }

   public ControlFlowGraph getControlFlowGraph()
   {
      return controlFlowGraph;
   }

   public StateEstimatorWithPorts getEstimator()
   {
      return stateEstimator;
   }

   public void initializeEstimatorToActual(FramePoint initialCoMPosition, FrameOrientation initialEstimationLinkOrientation)
   {
      stateEstimator.setEstimatedCoMPosition(initialCoMPosition);
      if (!assumePerfectIMU)
      {
         stateEstimator.setEstimatedOrientation(initialEstimationLinkOrientation);
      }
   }
}
