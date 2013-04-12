package us.ihmc.sensorProcessing.stateEstimation.evaluation;

import java.util.Collection;

import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.controlFlow.ControlFlowGraph;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.sensorProcessing.simulatedSensors.SensorMap;
import us.ihmc.sensorProcessing.stateEstimation.ComposableOrientationAndCoMEstimatorCreator;
import us.ihmc.sensorProcessing.stateEstimation.ComposableOrientationEstimatorCreator;
import us.ihmc.sensorProcessing.stateEstimation.DesiredCoMAndAngularAccelerationOutputPortsHolder;
import us.ihmc.sensorProcessing.stateEstimation.JointStateFullRobotModelUpdater;
import us.ihmc.sensorProcessing.stateEstimation.OrientationEstimator;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.AngularVelocitySensorConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.LinearAccelerationSensorConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.OrientationSensorConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.PointVelocitySensorConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.SensorConfigurationFactory;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class SensorAndEstimatorAssembler
{
   private static final boolean ESTIMATE_COM = true;

   private final double comAccelerationProcessNoiseStandardDeviation = Math.sqrt(1e-1);
   private final double angularAccelerationProcessNoiseStandardDeviation = Math.sqrt(1e-1);

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ControlFlowGraph controlFlowGraph;
   private final OrientationEstimator orientationEstimator;

   public SensorAndEstimatorAssembler(Vector3d gravitationalAcceleration, FullInverseDynamicsStructure inverseDynamicsStructure, double controlDT,
                                      SensorMap sensorMap, DesiredCoMAndAngularAccelerationOutputPortsHolder desiredCoMAndAngularAccelerationOutputPortsHolder,
                                      YoVariableRegistry parentRegistry)
   {
      SensorConfigurationFactory SensorConfigurationFactory = new SensorConfigurationFactory(gravitationalAcceleration);

      // Sensor configurations for estimator
      Collection<OrientationSensorConfiguration> orientationSensorConfigurations =
         SensorConfigurationFactory.createOrientationSensorConfigurations(sensorMap.getOrientationSensors());

      Collection<AngularVelocitySensorConfiguration> angularVelocitySensorConfigurations =
         SensorConfigurationFactory.createAngularVelocitySensorConfigurations(sensorMap.getAngularVelocitySensors());

      Collection<LinearAccelerationSensorConfiguration> linearAccelerationSensorConfigurations =
         SensorConfigurationFactory.createLinearAccelerationSensorConfigurations(sensorMap.getLinearAccelerationSensors());

      Collection<PointVelocitySensorConfiguration> pointVelocitySensorConfigurations =
         SensorConfigurationFactory.createPointVelocitySensorConfigurations(sensorMap.getPointVelocitySensors());

      controlFlowGraph = new ControlFlowGraph();
      JointStateFullRobotModelUpdater jointStateFullRobotModelUpdater = new JointStateFullRobotModelUpdater(controlFlowGraph, sensorMap,
                                                                           inverseDynamicsStructure);

      ControlFlowOutputPort<FullInverseDynamicsStructure> inverseDynamicsStructureOutputPort =
         jointStateFullRobotModelUpdater.getInverseDynamicsStructureOutputPort();

      RigidBody estimationLink = inverseDynamicsStructure.getEstimationLink();
      ReferenceFrame estimationFrame = estimationLink.getParentJoint().getFrameAfterJoint();

      DenseMatrix64F angularAccelerationNoiseCovariance = createDiagonalCovarianceMatrix(angularAccelerationProcessNoiseStandardDeviation, 3);

      if (ESTIMATE_COM)
      {
         DenseMatrix64F comAccelerationNoiseCovariance = createDiagonalCovarianceMatrix(comAccelerationProcessNoiseStandardDeviation, 3);

         ComposableOrientationAndCoMEstimatorCreator orientationEstimatorCreator =
            new ComposableOrientationAndCoMEstimatorCreator(angularAccelerationNoiseCovariance, comAccelerationNoiseCovariance, estimationLink,
               inverseDynamicsStructureOutputPort);
         orientationEstimatorCreator.addOrientationSensorConfigurations(orientationSensorConfigurations);
         orientationEstimatorCreator.addAngularVelocitySensorConfigurations(angularVelocitySensorConfigurations);
         orientationEstimatorCreator.addLinearAccelerationSensorConfigurations(linearAccelerationSensorConfigurations);
         orientationEstimatorCreator.addPointVelocitySensorConfigurations(pointVelocitySensorConfigurations);

         // TODO: Not sure if we need to do this here:
         inverseDynamicsStructure.updateInternalState();

         orientationEstimator = orientationEstimatorCreator.createOrientationEstimator(controlFlowGraph, controlDT,
                 inverseDynamicsStructure.getRootInverseDynamicsJoint(), estimationLink, estimationFrame,
                 desiredCoMAndAngularAccelerationOutputPortsHolder.getDesiredAngularAccelerationOutputPort(),
                 desiredCoMAndAngularAccelerationOutputPortsHolder.getDesiredCenterOfMassAccelerationOutputPort(), registry);
      }
      else
      {
         ComposableOrientationEstimatorCreator orientationEstimatorCreator = new ComposableOrientationEstimatorCreator(angularAccelerationNoiseCovariance,
                                                                                estimationLink, inverseDynamicsStructureOutputPort);
         orientationEstimatorCreator.addOrientationSensorConfigurations(orientationSensorConfigurations);
         orientationEstimatorCreator.addAngularVelocitySensorConfigurations(angularVelocitySensorConfigurations);

         orientationEstimator = orientationEstimatorCreator.createOrientationEstimator(controlFlowGraph, controlDT, estimationFrame,
                 desiredCoMAndAngularAccelerationOutputPortsHolder.getDesiredAngularAccelerationOutputPort(), registry);
      }

      controlFlowGraph.initializeAfterConnections();
//      controlFlowGraph.visualize();

      parentRegistry.addChild(registry);
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

   public OrientationEstimator getOrientationEstimator()
   {
      return orientationEstimator;
   }
}
