package us.ihmc.sensorProcessing.stateEstimation.evaluation;

import java.util.Collection;

import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.controlFlow.ControlFlowGraph;
import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.sensorProcessing.simulatedSensors.SensorMap;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.stateEstimation.ComposableOrientationAndCoMEstimatorCreator;
import us.ihmc.sensorProcessing.stateEstimation.ComposableOrientationEstimatorCreator;
import us.ihmc.sensorProcessing.stateEstimation.DesiredCoMAndAngularAccelerationOutputPortsHolder;
import us.ihmc.sensorProcessing.stateEstimation.JointStateFullRobotModelUpdater;
import us.ihmc.sensorProcessing.stateEstimation.OrientationEstimatorWithPorts;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.AngularVelocitySensorConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.LinearAccelerationSensorConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.OrientationSensorConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.PointVelocitySensorConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.SensorConfigurationFactory;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class SensorAndEstimatorAssembler
{
   private static final boolean ESTIMATE_COM = true;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ControlFlowGraph controlFlowGraph;
   private final OrientationEstimatorWithPorts orientationEstimator;

   public SensorAndEstimatorAssembler(SensorNoiseParameters sensorNoiseParametersForEstimator, Vector3d gravitationalAcceleration, 
         FullInverseDynamicsStructure inverseDynamicsStructure, double controlDT,
         SensorMap sensorMap,
         YoVariableRegistry parentRegistry)
   {
      SensorConfigurationFactory SensorConfigurationFactory = new SensorConfigurationFactory(sensorNoiseParametersForEstimator, gravitationalAcceleration);

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

      double angularAccelerationProcessNoiseStandardDeviation = sensorNoiseParametersForEstimator.getAngularAccelerationProcessNoiseStandardDeviation();
      DenseMatrix64F angularAccelerationNoiseCovariance = createDiagonalCovarianceMatrix(angularAccelerationProcessNoiseStandardDeviation, 3);

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

   public OrientationEstimatorWithPorts getOrientationEstimator()
   {
      return orientationEstimator;
   }
   
//   public static void connectDesiredAccelerationPorts(ControlFlowGraph controlFlowGraph, OrientationEstimatorWithPorts orientationEstimatorWithPorts,
//         DesiredCoMAndAngularAccelerationOutputPortsHolder desiredCoMAndAngularAccelerationOutputPortsHolder)
//   {
//      ControlFlowOutputPort<FrameVector> desiredAngularAccelerationOutputPort = desiredCoMAndAngularAccelerationOutputPortsHolder.getDesiredAngularAccelerationOutputPort();
//      ControlFlowOutputPort<FrameVector> desiredCenterOfMassAccelerationOutputPort = desiredCoMAndAngularAccelerationOutputPortsHolder.getDesiredCenterOfMassAccelerationOutputPort();
//      
//      ControlFlowInputPort<FrameVector> desiredAngularAccelerationInputPort = orientationEstimatorWithPorts.getDesiredAngularAccelerationInputPort();
//      ControlFlowInputPort<FrameVector> desiredCenterOfMassAccelerationInputPort = orientationEstimatorWithPorts.getDesiredCenterOfMassAccelerationInputPort();
//
//      if (desiredAngularAccelerationOutputPort != null)
//      {
//         controlFlowGraph.connectElements(desiredAngularAccelerationOutputPort, desiredAngularAccelerationInputPort);
//      }
//
//      if (desiredCenterOfMassAccelerationOutputPort != null)
//      {
//         controlFlowGraph.connectElements(desiredCenterOfMassAccelerationOutputPort, desiredCenterOfMassAccelerationInputPort);
//      }
//   }
}
