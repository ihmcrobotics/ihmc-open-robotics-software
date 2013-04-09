package us.ihmc.sensorProcessing.stateEstimation.evaluation;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.controlFlow.ControlFlowGraph;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.sensorProcessing.simulatedSensors.IMUDefinition;
import us.ihmc.sensorProcessing.simulatedSensors.PointVelocitySensorDefinition;
import us.ihmc.sensorProcessing.simulatedSensors.SensorDefinitionHolder;
import us.ihmc.sensorProcessing.simulatedSensors.SensorMap;
import us.ihmc.sensorProcessing.stateEstimation.ComposableOrientationAndCoMEstimatorCreator;
import us.ihmc.sensorProcessing.stateEstimation.ComposableOrientationEstimatorCreator;
import us.ihmc.sensorProcessing.stateEstimation.DesiredCoMAndAngularAccelerationOutputPortsHolder;
import us.ihmc.sensorProcessing.stateEstimation.OrientationEstimator;
import us.ihmc.sensorProcessing.stateEstimation.SensorOutputPortsHolder;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.AngularVelocitySensorConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.LinearAccelerationSensorConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.OrientationSensorConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.PointVelocitySensorConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.SensorConfigurationFactory;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.RobotController;

public class ComposableStateEstimatorEvaluatorController implements RobotController
{
   private static final boolean INITIALIZE_ANGULAR_VELOCITY_ESTIMATE_TO_ACTUAL = true;    // false;

   private static final boolean ESTIMATE_COM = true;
   
   private final double comAccelerationProcessNoiseStandardDeviation = Math.sqrt(1e-1);
   private final double angularAccelerationProcessNoiseStandardDeviation = Math.sqrt(1e-1);

   private final Vector3d gravitationalAcceleration;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final StateEstimatorEvaluatorFullRobotModel estimatedFullRobotModel;
   private final TwistCalculator estimatedTwistCalculator;
   private final SpatialAccelerationCalculator estimatedSpatialAccelerationCalculator;

   private final ControlFlowGraph controlFlowGraph;
   private final OrientationEstimator orientationEstimator;

   private final ComposableStateEstimatorEvaluatorErrorCalculator composableStateEstimatorEvaluatorErrorCalculator;
   
   public ComposableStateEstimatorEvaluatorController(StateEstimatorEvaluatorRobot robot, 
         StateEstimatorEvaluatorFullRobotModel estimatedFullRobotModel,
         double controlDT,
         SensorMap sensorMap, 
           DesiredCoMAndAngularAccelerationOutputPortsHolder desiredCoMAndAngularAccelerationOutputPortsHolder)
   {
      this.gravitationalAcceleration = new Vector3d();
      robot.getGravity(gravitationalAcceleration);

      this.estimatedFullRobotModel = estimatedFullRobotModel;
      estimatedTwistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), estimatedFullRobotModel.getElevator());
      estimatedSpatialAccelerationCalculator = new SpatialAccelerationCalculator(estimatedFullRobotModel.getElevator(), estimatedTwistCalculator, 0.0, false);

      SensorConfigurationFactory SensorConfigurationFactory = new SensorConfigurationFactory(gravitationalAcceleration);
      
      // Sensor configurations for estimator
      Collection<OrientationSensorConfiguration> orientationSensorConfigurations = SensorConfigurationFactory.createOrientationSensorConfigurations(sensorMap.getOrientationSensors());

      Collection<AngularVelocitySensorConfiguration> angularVelocitySensorConfigurations = SensorConfigurationFactory.createAngularVelocitySensorConfigurations(sensorMap.getAngularVelocitySensors());

      Collection<LinearAccelerationSensorConfiguration> linearAccelerationSensorConfigurations =
            SensorConfigurationFactory.createLinearAccelerationSensorConfigurations(sensorMap.getLinearAccelerationSensors());

      Collection<PointVelocitySensorConfiguration> pointVelocitySensorConfigurations = SensorConfigurationFactory.createPointVelocitySensorConfigurations(sensorMap.getPointVelocitySensors());

      controlFlowGraph = new ControlFlowGraph();
      RigidBody estimationLink = estimatedFullRobotModel.getRootBody();
      ReferenceFrame estimationFrame = estimationLink.getParentJoint().getFrameAfterJoint();

      DenseMatrix64F angularAccelerationNoiseCovariance = createDiagonalCovarianceMatrix(angularAccelerationProcessNoiseStandardDeviation, 3);



      if (ESTIMATE_COM)
      {
         DenseMatrix64F comAccelerationNoiseCovariance = createDiagonalCovarianceMatrix(comAccelerationProcessNoiseStandardDeviation, 3);

         ComposableOrientationAndCoMEstimatorCreator orientationEstimatorCreator =
            new ComposableOrientationAndCoMEstimatorCreator(angularAccelerationNoiseCovariance, comAccelerationNoiseCovariance, estimationLink,
               estimatedTwistCalculator, estimatedSpatialAccelerationCalculator);
         orientationEstimatorCreator.addOrientationSensorConfigurations(orientationSensorConfigurations);
         orientationEstimatorCreator.addAngularVelocitySensorConfigurations(angularVelocitySensorConfigurations);
         orientationEstimatorCreator.addLinearAccelerationSensorConfigurations(linearAccelerationSensorConfigurations);
         orientationEstimatorCreator.addPointVelocitySensorConfigurations(pointVelocitySensorConfigurations);

         updateInternalState();
         orientationEstimator = orientationEstimatorCreator.createOrientationEstimator(controlFlowGraph, controlDT,
                 estimatedFullRobotModel.getRootInverseDynamicsJoint(), estimationLink, estimationFrame, 
                 desiredCoMAndAngularAccelerationOutputPortsHolder.getDesiredAngularAccelerationOutputPort(),
                 desiredCoMAndAngularAccelerationOutputPortsHolder.getDesiredCenterOfMassAccelerationOutputPort(), registry);
      }
      else
      {
         ComposableOrientationEstimatorCreator orientationEstimatorCreator = new ComposableOrientationEstimatorCreator(angularAccelerationNoiseCovariance,
                                                                                estimationLink, estimatedTwistCalculator);
         orientationEstimatorCreator.addOrientationSensorConfigurations(orientationSensorConfigurations);
         orientationEstimatorCreator.addAngularVelocitySensorConfigurations(angularVelocitySensorConfigurations);

         orientationEstimator = orientationEstimatorCreator.createOrientationEstimator(controlFlowGraph, controlDT, estimationFrame,
               desiredCoMAndAngularAccelerationOutputPortsHolder.getDesiredAngularAccelerationOutputPort(), registry);
      }

      controlFlowGraph.initializeAfterConnections();

      if (INITIALIZE_ANGULAR_VELOCITY_ESTIMATE_TO_ACTUAL)
      {
         estimatedFullRobotModel.updateBasedOnRobot(true);

         Matrix3d rotationMatrix = new Matrix3d();
         robot.getRootJoint().getRotationToWorld(rotationMatrix);
         Vector3d angularVelocityInBody = robot.getRootJoint().getAngularVelocityInBody();

         FrameOrientation estimatedOrientation = orientationEstimator.getEstimatedOrientation();
         estimatedOrientation.set(rotationMatrix);
         orientationEstimator.setEstimatedOrientation(estimatedOrientation);

         FrameVector estimatedAngularVelocity = orientationEstimator.getEstimatedAngularVelocity();
         estimatedAngularVelocity.set(angularVelocityInBody);
         orientationEstimator.setEstimatedAngularVelocity(estimatedAngularVelocity);

         // TODO: This wasn't doing anything.
         // DenseMatrix64F x = orientationEstimator.getState();
         // MatrixTools.insertTuple3dIntoEJMLVector(angularVelocityInBody, x, 3);
         // orientationEstimator.setState(x, orientationEstimator.getCovariance());
      }

      this.composableStateEstimatorEvaluatorErrorCalculator = new ComposableStateEstimatorEvaluatorErrorCalculator(robot, orientationEstimator, registry);
   }


   public void initialize()
   {
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return registry.getName();
   }

   public String getDescription()
   {
      return getName();
   }

   public void doControl()
   {
      updateInternalState();

      controlFlowGraph.startComputation();
      controlFlowGraph.waitUntilComputationIsDone();

      if (!ESTIMATE_COM)    // this is being done inside the state estimator for the CoM estimator
      {
         estimatedFullRobotModel.updateBasedOnEstimator(orientationEstimator);
         estimatedTwistCalculator.compute();
      }

      composableStateEstimatorEvaluatorErrorCalculator.computeErrors();
   }

   private void updateInternalState()
   {
      /*
       * supersense joint positions and velocities and update twist
       * calculator and spatial accel calculator based on this data TODO:
       * need a class that does this based on sensors
       */
      estimatedFullRobotModel.updateBasedOnRobot(false);
      estimatedTwistCalculator.compute();
      estimatedSpatialAccelerationCalculator.compute();
   }

   private static DenseMatrix64F createDiagonalCovarianceMatrix(double standardDeviation, int size)
   {
      DenseMatrix64F orientationCovarianceMatrix = new DenseMatrix64F(size, size);
      CommonOps.setIdentity(orientationCovarianceMatrix);
      CommonOps.scale(MathTools.square(standardDeviation), orientationCovarianceMatrix);

      return orientationCovarianceMatrix;
   }
   
}
