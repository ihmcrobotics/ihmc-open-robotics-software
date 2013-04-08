package us.ihmc.sensorProcessing.stateEstimation.evaluation;

import java.util.ArrayList;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.controlFlow.ControlFlowGraph;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.sensorProcessing.simulatedSensors.SimulatedSensorsFactory;
import us.ihmc.sensorProcessing.simulatedSensors.IMUDefinition;
import us.ihmc.sensorProcessing.simulatedSensors.PointVelocitySensorDefinition;
import us.ihmc.sensorProcessing.stateEstimation.ComposableOrientationAndCoMEstimatorCreator;
import us.ihmc.sensorProcessing.stateEstimation.ComposableOrientationEstimatorCreator;
import us.ihmc.sensorProcessing.stateEstimation.DesiredCoMAndAngularAccelerationOutputPortsHolder;
import us.ihmc.sensorProcessing.stateEstimation.OrientationEstimator;
import us.ihmc.sensorProcessing.stateEstimation.SensorOutputPortsHolder;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.AngularVelocitySensorConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.LinearAccelerationSensorConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.OrientationSensorConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.PointVelocitySensorConfiguration;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.AngleTools;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.RobotController;

public class ComposableStateEstimatorEvaluatorController implements RobotController
{
   private static final boolean INITIALIZE_ANGULAR_VELOCITY_ESTIMATE_TO_ACTUAL = true;    // false;

   private static final boolean ESTIMATE_COM = true;

   private final double orientationMeasurementStandardDeviation = Math.sqrt(1e-2);
   private final double angularVelocityMeasurementStandardDeviation = 1e-1;    // 1e-3;    // 2e-4;
   private final double linearAccelerationMeasurementStandardDeviation = 1e0;    // 1e-1;    // 1.7e-2;
   private final double pointVelocityMeasurementStandardDeviation = 1e-1;    // 1e0; //1e1; //1e-1; //1e-10; //1e-1;    // 1.7e-2;

   private final double comAccelerationProcessNoiseStandardDeviation = Math.sqrt(1e-1);
   private final double angularAccelerationProcessNoiseStandardDeviation = Math.sqrt(1e-1);

   private final double angularVelocityBiasProcessNoiseStandardDeviation = Math.sqrt(1e-5);
   private final double linearAccelerationBiasProcessNoiseStandardDeviation = Math.sqrt(1e-4);

   private final Vector3d gravitationalAcceleration;

   private final YoVariableRegistry registry = new YoVariableRegistry("QuaternionOrientationEstimatorEvaluatorController");

   private final DoubleYoVariable orientationError = new DoubleYoVariable("orientationError", registry);
   private final DoubleYoVariable angularVelocityError = new DoubleYoVariable("angularVelocityError", registry);
   private final DoubleYoVariable comPositionError = new DoubleYoVariable("comPositionError", registry);
   private final DoubleYoVariable comVelocityError = new DoubleYoVariable("comVelocityError", registry);

   private final StateEstimatorEvaluatorRobot robot;
   private final double controlDT;

   private final StateEstimatorEvaluatorFullRobotModel estimatedFullRobotModel;
   private final TwistCalculator estimatedTwistCalculator;
   private final SpatialAccelerationCalculator estimatedSpatialAccelerationCalculator;

   private final ControlFlowGraph controlFlowGraph;
   private final OrientationEstimator orientationEstimator;

   public ComposableStateEstimatorEvaluatorController(StateEstimatorEvaluatorRobot robot, double controlDT,
           SensorOutputPortsHolder sensorOutputPortsHolder, 
           DesiredCoMAndAngularAccelerationOutputPortsHolder desiredCoMAndAngularAccelerationOutputPortsHolder)
   {
      this.robot = robot;
      this.controlDT = controlDT;
      this.gravitationalAcceleration = new Vector3d();
      robot.getGravity(gravitationalAcceleration);

      estimatedFullRobotModel = new StateEstimatorEvaluatorFullRobotModel(robot, robot.getIMUMounts(), robot.getVelocityPoints());
      estimatedTwistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), estimatedFullRobotModel.getElevator());
      estimatedSpatialAccelerationCalculator = new SpatialAccelerationCalculator(estimatedFullRobotModel.getElevator(), estimatedTwistCalculator, 0.0, false);


      // Sensor configurations for estimator
      ArrayList<OrientationSensorConfiguration> orientationSensorConfigurations = createOrientationSensorConfigurations(estimatedFullRobotModel,
                                                                                     sensorOutputPortsHolder.getOrientationOutputPorts());

      ArrayList<AngularVelocitySensorConfiguration> angularVelocitySensorConfigurations = createAngularVelocitySensorConfigurations(estimatedFullRobotModel,
                                                                                             sensorOutputPortsHolder.getAngularVelocityOutputPorts());

      ArrayList<LinearAccelerationSensorConfiguration> linearAccelerationSensorConfigurations =
         createLinearAccelerationSensorConfigurations(estimatedFullRobotModel, sensorOutputPortsHolder.getLinearAccelerationOutputPorts());

      ArrayList<PointVelocitySensorConfiguration> pointVelocitySensorConfigurations = createPointVelocitySensorConfigurations(estimatedFullRobotModel,
            sensorOutputPortsHolder.getPointVelocitySensorOutputPorts());

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
         estimatedFullRobotModel.updateBasedOnRobot(robot, true);

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

   }

   private ArrayList<OrientationSensorConfiguration> createOrientationSensorConfigurations(StateEstimatorEvaluatorFullRobotModel estimatedFullRobotModel,
           ArrayList<ControlFlowOutputPort<Matrix3d>> orientationOutputPorts)
   {
      ArrayList<OrientationSensorConfiguration> orientationSensorConfigurations = new ArrayList<OrientationSensorConfiguration>();

      ArrayList<IMUDefinition> estimatedIMUDefinitions = estimatedFullRobotModel.getIMUDefinitions();
      for (int i = 0; i < estimatedIMUDefinitions.size(); i++)
      {
         IMUDefinition estimatedIMUDefinition = estimatedIMUDefinitions.get(i);
         String sensorName = estimatedIMUDefinition.getName() + "Orientation";

         DenseMatrix64F orientationNoiseCovariance = createDiagonalCovarianceMatrix(orientationMeasurementStandardDeviation, 3);

         RigidBody estimatedMeasurementBody = estimatedIMUDefinition.getRigidBody();
         ReferenceFrame estimatedMeasurementFrame = SimulatedSensorsFactory.createMeasurementFrame(sensorName, "EstimatedMeasurementFrame",
                                                       estimatedIMUDefinition, estimatedMeasurementBody);

         OrientationSensorConfiguration orientationSensorConfiguration = new OrientationSensorConfiguration(orientationOutputPorts.get(i), sensorName,
                                                                            estimatedMeasurementFrame, orientationNoiseCovariance);
         orientationSensorConfigurations.add(orientationSensorConfiguration);
      }


      return orientationSensorConfigurations;
   }

   private ArrayList<AngularVelocitySensorConfiguration> createAngularVelocitySensorConfigurations(
           StateEstimatorEvaluatorFullRobotModel estimatedFullRobotModel, ArrayList<ControlFlowOutputPort<Vector3d>> angularVelocityOutputPorts)
   {
      ArrayList<AngularVelocitySensorConfiguration> angularVelocitySensorConfigurations = new ArrayList<AngularVelocitySensorConfiguration>();

      ArrayList<IMUDefinition> estimatedIMUDefinitions = estimatedFullRobotModel.getIMUDefinitions();
      for (int i = 0; i < estimatedIMUDefinitions.size(); i++)
      {
         IMUDefinition estimatedIMUDefinition = estimatedIMUDefinitions.get(i);
         String sensorName = estimatedIMUDefinition.getName() + "AngularVelocity";

         DenseMatrix64F angularVelocityNoiseCovariance = createDiagonalCovarianceMatrix(angularVelocityMeasurementStandardDeviation, 3);
         DenseMatrix64F angularVelocityBiasProcessNoiseCovariance = createDiagonalCovarianceMatrix(angularVelocityBiasProcessNoiseStandardDeviation, 3);

         RigidBody estimatedMeasurementBody = estimatedIMUDefinition.getRigidBody();
         ReferenceFrame estimatedMeasurementFrame = SimulatedSensorsFactory.createMeasurementFrame(sensorName, "EstimatedMeasurementFrame",
                                                       estimatedIMUDefinition, estimatedMeasurementBody);

         AngularVelocitySensorConfiguration angularVelocitySensorConfiguration = new AngularVelocitySensorConfiguration(angularVelocityOutputPorts.get(i),
                                                                                    sensorName, estimatedMeasurementBody, estimatedMeasurementFrame,
                                                                                    angularVelocityNoiseCovariance, angularVelocityBiasProcessNoiseCovariance);
         angularVelocitySensorConfigurations.add(angularVelocitySensorConfiguration);
      }


      return angularVelocitySensorConfigurations;
   }


   private ArrayList<LinearAccelerationSensorConfiguration> createLinearAccelerationSensorConfigurations(
           StateEstimatorEvaluatorFullRobotModel estimatedFullRobotModel, ArrayList<ControlFlowOutputPort<Vector3d>> linearAccelerationOutputPorts)
   {
      ArrayList<LinearAccelerationSensorConfiguration> linearAccelerationSensorConfigurations = new ArrayList<LinearAccelerationSensorConfiguration>();

      ArrayList<IMUDefinition> estimatedIMUDefinitions = estimatedFullRobotModel.getIMUDefinitions();
      for (int i = 0; i < estimatedIMUDefinitions.size(); i++)
      {
         IMUDefinition estimatedIMUDefinition = estimatedIMUDefinitions.get(i);
         String sensorName = estimatedIMUDefinition.getName() + "LinearAcceleration";

         DenseMatrix64F linearAccelerationNoiseCovariance = createDiagonalCovarianceMatrix(linearAccelerationMeasurementStandardDeviation, 3);
         DenseMatrix64F linearAccelerationBiasProcessNoiseCovariance = createDiagonalCovarianceMatrix(linearAccelerationBiasProcessNoiseStandardDeviation, 3);

         RigidBody estimatedMeasurementBody = estimatedIMUDefinition.getRigidBody();
         ReferenceFrame estimatedMeasurementFrame = SimulatedSensorsFactory.createMeasurementFrame(sensorName, "EstimatedMeasurementFrame",
                                                       estimatedIMUDefinition, estimatedMeasurementBody);

         LinearAccelerationSensorConfiguration linearAccelerationSensorConfiguration =
            new LinearAccelerationSensorConfiguration(linearAccelerationOutputPorts.get(i), sensorName, estimatedMeasurementBody, estimatedMeasurementFrame,
               gravitationalAcceleration.getZ(), linearAccelerationNoiseCovariance, linearAccelerationBiasProcessNoiseCovariance);

         linearAccelerationSensorConfigurations.add(linearAccelerationSensorConfiguration);
      }

      return linearAccelerationSensorConfigurations;
   }


   private ArrayList<PointVelocitySensorConfiguration> createPointVelocitySensorConfigurations(StateEstimatorEvaluatorFullRobotModel estimatedFullRobotModel,
           ArrayList<ControlFlowOutputPort<Vector3d>> pointVelocityOutputPorts)
   {
      ArrayList<PointVelocitySensorConfiguration> pointVelocitySensorConfigurations = new ArrayList<PointVelocitySensorConfiguration>();


      ArrayList<PointVelocitySensorDefinition> pointVelocitySensorDefinitions = estimatedFullRobotModel.getPointVelocitySensorDefinitions();

      for (int i = 0; i < pointVelocitySensorDefinitions.size(); i++)
      {
         PointVelocitySensorDefinition pointVelocitySensorDefinition = pointVelocitySensorDefinitions.get(i);
         String sensorName = pointVelocitySensorDefinition.getName() + "PointVelocity";

         DenseMatrix64F pointVelocityNoiseCovariance = createDiagonalCovarianceMatrix(pointVelocityMeasurementStandardDeviation, 3);

         RigidBody estimatedMeasurementBody = pointVelocitySensorDefinition.getRigidBody();
         ReferenceFrame estimatedFrameAfterJoint = estimatedMeasurementBody.getParentJoint().getFrameAfterJoint();

         Vector3d offset = new Vector3d();
         pointVelocitySensorDefinition.getOffset(offset);
         FramePoint estimatedVelocityPoint = new FramePoint(estimatedFrameAfterJoint, offset);

         PointVelocitySensorConfiguration pointVelocitySensorConfiguration = new PointVelocitySensorConfiguration(pointVelocityOutputPorts.get(i), sensorName,
                                                                                estimatedMeasurementBody, estimatedVelocityPoint,
                                                                                angularAccelerationProcessNoiseStandardDeviation, pointVelocityNoiseCovariance,
                                                                                pointVelocityNoiseCovariance);

         pointVelocitySensorConfigurations.add(pointVelocitySensorConfiguration);
      }

      return pointVelocitySensorConfigurations;
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

      computeOrientationError();
      computeAngularVelocityError();
      computeCoMPositionError();
      computeCoMVelocityError();
   }

   private void updateInternalState()
   {
      /*
       * supersense joint positions and velocities and update twist
       * calculator and spatial accel calculator based on this data TODO:
       * need a class that does this based on sensors
       */
      estimatedFullRobotModel.updateBasedOnRobot(robot, false);
      estimatedTwistCalculator.compute();
      estimatedSpatialAccelerationCalculator.compute();
   }

   private void computeOrientationError()
   {
      FrameOrientation estimatedOrientation = orientationEstimator.getEstimatedOrientation();
      Quat4d estimatedOrientationQuat4d = new Quat4d();
      estimatedOrientation.getQuaternion(estimatedOrientationQuat4d);

      Quat4d orientationErrorQuat4d = new Quat4d(robot.getActualOrientation());
      orientationErrorQuat4d.mulInverse(estimatedOrientationQuat4d);

      AxisAngle4d orientationErrorAxisAngle = new AxisAngle4d();
      orientationErrorAxisAngle.set(orientationErrorQuat4d);

      double errorAngle = AngleTools.trimAngleMinusPiToPi(orientationErrorAxisAngle.getAngle());

      orientationError.set(Math.abs(errorAngle));
   }

   private void computeAngularVelocityError()
   {
      Vector3d estimatedAngularVelocity = orientationEstimator.getEstimatedAngularVelocity().getVectorCopy();
      Vector3d actualAngularVelocity = robot.getActualAngularVelocity();

      actualAngularVelocity.sub(estimatedAngularVelocity);
      angularVelocityError.set(actualAngularVelocity.length());
   }

   private void computeCoMPositionError()
   {
      Point3d comPoint = new Point3d();
      Vector3d linearVelocity = new Vector3d();
      Vector3d angularMomentum = new Vector3d();

      robot.computeCOMMomentum(comPoint, linearVelocity, angularMomentum);

      Vector3d comError = new Vector3d();
      comError.set(orientationEstimator.getEstimatedCoMPosition().getPointCopy());
      comError.sub(comPoint);

      comPositionError.set(comError.length());
   }

   private void computeCoMVelocityError()
   {
      Point3d comPoint = new Point3d();
      Vector3d linearVelocity = new Vector3d();
      Vector3d angularMomentum = new Vector3d();

      double mass = robot.computeCOMMomentum(comPoint, linearVelocity, angularMomentum);
      linearVelocity.scale(1.0 / mass);

      Vector3d estimatedCoMVelocity = orientationEstimator.getEstimatedCoMVelocity().getVectorCopy();

      estimatedCoMVelocity.sub(linearVelocity);
      comVelocityError.set(estimatedCoMVelocity.length());
   }

   private static DenseMatrix64F createDiagonalCovarianceMatrix(double standardDeviation, int size)
   {
      DenseMatrix64F orientationCovarianceMatrix = new DenseMatrix64F(size, size);
      CommonOps.setIdentity(orientationCovarianceMatrix);
      CommonOps.scale(MathTools.square(standardDeviation), orientationCovarianceMatrix);

      return orientationCovarianceMatrix;
   }

}
