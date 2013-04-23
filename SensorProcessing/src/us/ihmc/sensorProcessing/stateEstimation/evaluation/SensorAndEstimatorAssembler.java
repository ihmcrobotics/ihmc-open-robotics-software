package us.ihmc.sensorProcessing.stateEstimation.evaluation;

import java.util.Collection;

import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.controlFlow.ControlFlowGraph;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.sensorProcessing.simulatedSensors.JointAndIMUSensorMap;
import us.ihmc.sensorProcessing.simulatedSensors.PointPositionSensorMap;
import us.ihmc.sensorProcessing.simulatedSensors.PointVelocitySensorMap;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitions;
import us.ihmc.sensorProcessing.stateEstimation.ComposableOrientationAndCoMEstimatorCreator;
import us.ihmc.sensorProcessing.stateEstimation.ComposableOrientationEstimatorCreator;
import us.ihmc.sensorProcessing.stateEstimation.DesiredCoMAndAngularAccelerationDataSource;
import us.ihmc.sensorProcessing.stateEstimation.JointAndIMUSensorDataSource;
import us.ihmc.sensorProcessing.stateEstimation.JointStateFullRobotModelUpdater;
import us.ihmc.sensorProcessing.stateEstimation.PointPositionSensorDataSource;
import us.ihmc.sensorProcessing.stateEstimation.PointVelocitySensorDataSource;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorWithPorts;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.AngularVelocitySensorConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.LinearAccelerationSensorConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.OrientationSensorConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.PointPositionSensorConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.PointVelocitySensorConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.SensorConfigurationFactory;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class SensorAndEstimatorAssembler
{
   private static final boolean ESTIMATE_COM = true;
   private static final boolean VISUALIZE_CONTROL_FLOW_GRAPH = false;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ControlFlowGraph controlFlowGraph;
   private final StateEstimatorWithPorts orientationEstimator;
   private final JointAndIMUSensorDataSource jointSensorDataSource;
   
   private final PointPositionSensorDataSource pointPositionSensorDataSource;
   private final PointVelocitySensorDataSource pointVelocitySensorDataSource;
   
   private final DesiredCoMAndAngularAccelerationDataSource desiredCoMAndAngularAccelerationDataSource;
   
   public SensorAndEstimatorAssembler(StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions,
         SensorNoiseParameters sensorNoiseParametersForEstimator, Vector3d gravitationalAcceleration, 
         FullInverseDynamicsStructure inverseDynamicsStructure, double controlDT,
         YoVariableRegistry parentRegistry)
   {
      SensorConfigurationFactory SensorConfigurationFactory = new SensorConfigurationFactory(sensorNoiseParametersForEstimator, gravitationalAcceleration);

      jointSensorDataSource = new JointAndIMUSensorDataSource(stateEstimatorSensorDefinitions);
      JointAndIMUSensorMap jointAndIMUSensorMap = jointSensorDataSource.getSensorMap();
      
      pointPositionSensorDataSource = new PointPositionSensorDataSource(stateEstimatorSensorDefinitions);
      PointPositionSensorMap pointPositionSensorMap = pointPositionSensorDataSource.getSensorMap();
      
      pointVelocitySensorDataSource = new PointVelocitySensorDataSource(stateEstimatorSensorDefinitions);
      PointVelocitySensorMap pointVelocitySensorMap = pointVelocitySensorDataSource.getSensorMap();
      
      ReferenceFrame estimationFrame = inverseDynamicsStructure.getEstimationFrame();
      desiredCoMAndAngularAccelerationDataSource = new DesiredCoMAndAngularAccelerationDataSource(estimationFrame);      

      // Sensor configurations for estimator
      Collection<OrientationSensorConfiguration> orientationSensorConfigurations =
         SensorConfigurationFactory.createOrientationSensorConfigurations(jointAndIMUSensorMap.getOrientationSensors());

      Collection<AngularVelocitySensorConfiguration> angularVelocitySensorConfigurations =
         SensorConfigurationFactory.createAngularVelocitySensorConfigurations(jointAndIMUSensorMap.getAngularVelocitySensors());

      Collection<LinearAccelerationSensorConfiguration> linearAccelerationSensorConfigurations =
         SensorConfigurationFactory.createLinearAccelerationSensorConfigurations(jointAndIMUSensorMap.getLinearAccelerationSensors());

      Collection<PointPositionSensorConfiguration> pointPositionSensorConfigurations =
            SensorConfigurationFactory.createPointPositionSensorConfigurations(pointPositionSensorMap.getPointPositionSensors());
      
      Collection<PointVelocitySensorConfiguration> pointVelocitySensorConfigurations =
         SensorConfigurationFactory.createPointVelocitySensorConfigurations(pointVelocitySensorMap.getPointVelocitySensors());

      controlFlowGraph = new ControlFlowGraph();
      JointStateFullRobotModelUpdater jointStateFullRobotModelUpdater = new JointStateFullRobotModelUpdater(controlFlowGraph, jointAndIMUSensorMap,
                                                                           inverseDynamicsStructure);

      ControlFlowOutputPort<FullInverseDynamicsStructure> inverseDynamicsStructureOutputPort =
         jointStateFullRobotModelUpdater.getInverseDynamicsStructureOutputPort();

      double angularAccelerationProcessNoiseStandardDeviation = sensorNoiseParametersForEstimator.getAngularAccelerationProcessNoiseStandardDeviation();
      DenseMatrix64F angularAccelerationNoiseCovariance = createDiagonalCovarianceMatrix(angularAccelerationProcessNoiseStandardDeviation, 3);

      RigidBody estimationLink = inverseDynamicsStructure.getEstimationLink();

      if (ESTIMATE_COM)
      {
         double comAccelerationProcessNoiseStandardDeviation = sensorNoiseParametersForEstimator.getComAccelerationProcessNoiseStandardDeviation();
         DenseMatrix64F comAccelerationNoiseCovariance = createDiagonalCovarianceMatrix(comAccelerationProcessNoiseStandardDeviation, 3);

         ComposableOrientationAndCoMEstimatorCreator orientationEstimatorCreator =
            new ComposableOrientationAndCoMEstimatorCreator(angularAccelerationNoiseCovariance, comAccelerationNoiseCovariance, estimationLink,
               inverseDynamicsStructureOutputPort);
         orientationEstimatorCreator.addOrientationSensorConfigurations(orientationSensorConfigurations);
         orientationEstimatorCreator.addAngularVelocitySensorConfigurations(angularVelocitySensorConfigurations);
         orientationEstimatorCreator.addLinearAccelerationSensorConfigurations(linearAccelerationSensorConfigurations);
         
         orientationEstimatorCreator.addPointPositionSensorConfigurations(pointPositionSensorConfigurations);
         orientationEstimatorCreator.addPointVelocitySensorConfigurations(pointVelocitySensorConfigurations);

         // TODO: Not sure if we need to do this here:
         inverseDynamicsStructure.updateInternalState();

         orientationEstimator = orientationEstimatorCreator.createOrientationEstimator(controlFlowGraph, controlDT,
                 inverseDynamicsStructure.getRootJoint(), estimationLink, estimationFrame, registry);
         
      }
      else
      {
         ComposableOrientationEstimatorCreator orientationEstimatorCreator = new ComposableOrientationEstimatorCreator(angularAccelerationNoiseCovariance,
                                                                                estimationLink, inverseDynamicsStructureOutputPort);
         orientationEstimatorCreator.addOrientationSensorConfigurations(orientationSensorConfigurations);
         orientationEstimatorCreator.addAngularVelocitySensorConfigurations(angularVelocitySensorConfigurations);

         orientationEstimator = orientationEstimatorCreator.createOrientationEstimator(controlFlowGraph, controlDT, estimationFrame, registry);
      }
      
      desiredCoMAndAngularAccelerationDataSource.connectDesiredAccelerationPorts(controlFlowGraph, orientationEstimator);

      parentRegistry.addChild(registry);
      
      controlFlowGraph.initializeAfterConnections();
      controlFlowGraph.startComputation();
      controlFlowGraph.waitUntilComputationIsDone();

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

   public StateEstimatorWithPorts getOrientationEstimator()
   {
      return orientationEstimator;
   }
   
   public JointAndIMUSensorDataSource getJointAndIMUSensorDataSource()
   {
      return jointSensorDataSource;
   }
   
   public PointPositionSensorDataSource getPointPositionSensorDataSource()
   {
      return pointPositionSensorDataSource;
   }
   
   public PointVelocitySensorDataSource getPointVelocitySensorDataSource()
   {
      return pointVelocitySensorDataSource;
   }

   public DesiredCoMAndAngularAccelerationDataSource getDesiredCoMAndAngularAccelerationDataSource()
   {
      return desiredCoMAndAngularAccelerationDataSource;
   }
  
}
