package us.ihmc.commonWalkingControlModules.stateEstimation;

import java.util.ArrayList;
import java.util.Random;

import javax.media.j3d.Transform3D;
import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.controlFlow.AbstractControlFlowElement;
import us.ihmc.controlFlow.ControlFlowGraph;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.sensorProcessing.signalCorruption.GaussianOrientationCorruptor;
import us.ihmc.sensorProcessing.signalCorruption.GaussianVectorCorruptor;
import us.ihmc.sensorProcessing.signalCorruption.RandomWalkBiasVectorCorruptor;
import us.ihmc.sensorProcessing.simulatedSensors.SimulatedAngularVelocitySensor;
import us.ihmc.sensorProcessing.simulatedSensors.SimulatedLinearAccelerationSensor;
import us.ihmc.sensorProcessing.simulatedSensors.SimulatedOrientationSensor;
import us.ihmc.sensorProcessing.simulatedSensors.SimulatedPointVelocitySensor;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.AngleTools;
import us.ihmc.utilities.math.geometry.Direction;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.CenterOfMassAccelerationCalculator;
import us.ihmc.utilities.screwTheory.CenterOfMassCalculator;
import us.ihmc.utilities.screwTheory.CenterOfMassJacobian;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.IMUMount;
import com.yobotics.simulationconstructionset.KinematicPoint;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.RobotController;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

public class ComposableStateEstimatorEvaluatorController implements RobotController
{
   private static final boolean INITIALIZE_ANGULAR_VELOCITY_ESTIMATE_TO_ACTUAL = true;    // false;
   private static final boolean USE_ANGULAR_ACCELERATION_INPUT = true;

   private static final boolean CREATE_ORIENTATION_SENSOR = true;
   private static final boolean CREATE_ANGULAR_VELOCITY_SENSOR = true;
   private static final boolean CREATE_LINEAR_ACCELERATION_SENSOR = true;
   private static final boolean CREATE_POINT_VELOCITY_SENSOR = true;

   private static final boolean ALLOW_CHANGING_BIASES = true;

   private static final boolean ESTIMATE_COM = true;

   private static final boolean CORRUPT_SIMULATED_SENSORS = true;

   // from a recent pull request:
   // https://bitbucket.org/osrf/drcsim/pull-request/172/add-noise-model-to-sensors-gazebo-16/diff
   // <noise>
   // <type>gaussian</type>
   // <!-- Noise parameters from Boston Dynamics
   // (http://gazebosim.org/wiki/Sensor_noise):
   // rates (rad/s): mean=0, stddev=2e-4
   // accels (m/s/s): mean=0, stddev=1.7e-2
   // rate bias (rad/s): 5e-6 - 1e-5
   // accel bias (m/s/s): 1e-1
   // Experimentally, simulation provide rates with noise of
   // about 1e-3 rad/s and accels with noise of about 1e-1 m/s/s.
   // So we don't expect to see the noise unless number of inner iterations
   // are increased.
   //
   // We will add bias.  In this model, bias is sampled once for rates
   // and once for accels at startup; the sign (negative or positive)
   // of each bias is then switched with equal probability.  Thereafter,
   // the biases are fixed additive offsets.  We choose
   // bias means and stddevs to produce biases close to the provided
   // data. -->
   // <rate>
   // <mean>0.0</mean>
   // <stddev>2e-4</stddev>
   // <bias_mean>0.0000075</bias_mean>
   // <bias_stddev>0.0000008</bias_stddev>
   // </rate>
   // <accel>
   // <mean>0.0</mean>
   // <stddev>1.7e-2</stddev>
   // <bias_mean>0.1</bias_mean>
   // <bias_stddev>0.001</bias_stddev>
   // </accel>
   // </noise>

   private final double orientationMeasurementStandardDeviation = Math.sqrt(1e-2);
   private final double angularVelocityMeasurementStandardDeviation = 1e-1;    // 1e-3;    // 2e-4;
   private final double linearAccelerationMeasurementStandardDeviation = 1e0;    // 1e-1;    // 1.7e-2;
   private final double pointVelocityMeasurementStandardDeviation = 1e-1;    // 1e0; //1e1; //1e-1; //1e-10; //1e-1;    // 1.7e-2;

   private final double angularAccelerationProcessNoiseStandardDeviation = Math.sqrt(1e-1);
   private final double comAccelerationProcessNoiseStandardDeviation = Math.sqrt(1e-1);
   private final double angularVelocityBiasProcessNoiseStandardDeviation = Math.sqrt(1e-5);
   private final double linearAccelerationBiasProcessNoiseStandardDeviation = Math.sqrt(1e-4);

   private final double gazeboAngularVelocityBiasStandardDeviation = 0.0000008;
   private final double gazeboLinearAccelerationBiasStandardDeviation = 0.001;

   private final double gazeboAngularVelocityBiasMean = 0.0000075;
   private final double gazeboLinearAccelerationBiasMean = 0.1;

   private final Vector3d gravitationalAcceleration;

   private final YoVariableRegistry registry = new YoVariableRegistry("QuaternionOrientationEstimatorEvaluatorController");

   private final DoubleYoVariable orientationError = new DoubleYoVariable("orientationError", registry);
   private final DoubleYoVariable angularVelocityError = new DoubleYoVariable("angularVelocityError", registry);
   private final DoubleYoVariable comPositionError = new DoubleYoVariable("comPositionError", registry);
   private final DoubleYoVariable comVelocityError = new DoubleYoVariable("comVelocityError", registry);

   private final StateEstimatorEstimatorEvaluatorRobot robot;
   private final double controlDT;

   private final StateEstimatorEvaluatorFullRobotModel perfectFullRobotModel;
   private final TwistCalculator perfectTwistCalculator;
   private final SpatialAccelerationCalculator perfectSpatialAccelerationCalculator;

   private final StateEstimatorEvaluatorFullRobotModel estimatedFullRobotModel;
   private final TwistCalculator estimatedTwistCalculator;
   private final SpatialAccelerationCalculator estimatedSpatialAccelerationCalculator;

   private final CenterOfMassCalculator perfectCenterOfMassCalculator;
   private final CenterOfMassJacobian perfectCenterOfMassJacobian;
   private final CenterOfMassAccelerationCalculator perfectCenterOfMassAccelerationCalculator;

   private final YoFramePoint perfectCoM = new YoFramePoint("perfectCoM", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector perfectCoMd = new YoFrameVector("perfectCoMd", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector perfectCoMdd = new YoFrameVector("perfectCoMdd", ReferenceFrame.getWorldFrame(), registry);

   private final ControlFlowGraph controlFlowGraph;
   private final OrientationEstimator orientationEstimator;

   private final Random random = new Random(1779L);

   public ComposableStateEstimatorEvaluatorController(StateEstimatorEstimatorEvaluatorRobot robot, double controlDT)
   {
      this.robot = robot;
      this.controlDT = controlDT;
      this.gravitationalAcceleration = new Vector3d();
      robot.getGravity(gravitationalAcceleration);

      perfectFullRobotModel = new StateEstimatorEvaluatorFullRobotModel(robot);
      perfectTwistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), perfectFullRobotModel.getElevator());

      perfectSpatialAccelerationCalculator = new SpatialAccelerationCalculator(perfectFullRobotModel.getElevator(), perfectTwistCalculator, 0.0, false);
      estimatedFullRobotModel = new StateEstimatorEvaluatorFullRobotModel(robot);
      estimatedTwistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), estimatedFullRobotModel.getElevator());
      estimatedSpatialAccelerationCalculator = new SpatialAccelerationCalculator(estimatedFullRobotModel.getElevator(), estimatedTwistCalculator, 0.0, false);

      perfectCenterOfMassCalculator = new CenterOfMassCalculator(perfectFullRobotModel.getElevator(), ReferenceFrame.getWorldFrame());
      perfectCenterOfMassJacobian = new CenterOfMassJacobian(perfectFullRobotModel.getElevator());
      perfectCenterOfMassAccelerationCalculator = new CenterOfMassAccelerationCalculator(perfectFullRobotModel.getElevator(),
              perfectSpatialAccelerationCalculator);

      ArrayList<OrientationSensorConfiguration> orientationSensorConfigurations = createOrientationSensors(perfectFullRobotModel, estimatedFullRobotModel);
      ArrayList<AngularVelocitySensorConfiguration> angularVelocitySensorConfigurations = createAngularVelocitySensors(perfectFullRobotModel,
                                                                                             estimatedFullRobotModel);
      ArrayList<LinearAccelerationSensorConfiguration> linearAccelerationSensorConfigurations = createLinearAccelerationSensors(perfectFullRobotModel,
                                                                                                   estimatedFullRobotModel);

      ArrayList<PointVelocitySensorConfiguration> pointVelocitySensorConfigurations = createPointVelocitySensors(perfectFullRobotModel,
                                                                                         estimatedFullRobotModel);

      controlFlowGraph = new ControlFlowGraph();
      RigidBody estimationLink = estimatedFullRobotModel.getRootBody();
      ReferenceFrame estimationFrame = estimationLink.getParentJoint().getFrameAfterJoint();

      DenseMatrix64F angularAccelerationNoiseCovariance = createDiagonalCovarianceMatrix(angularAccelerationProcessNoiseStandardDeviation, 3);

      ControlFlowOutputPort<FrameVector> desiredAngularAccelerationOutputPort = null;
      if (ESTIMATE_COM || USE_ANGULAR_ACCELERATION_INPUT)
      {
         AngularAccelerationFromRobotStealer angularAccelerationFromRobotStealer = new AngularAccelerationFromRobotStealer(robot, estimationFrame);
         desiredAngularAccelerationOutputPort = angularAccelerationFromRobotStealer.getOutputPort();
      }

      if (ESTIMATE_COM)
      {
         CenterOfMassAccelerationFromFullRobotModelStealer centerOfMassAccelerationFromFullRobotModelStealer =
            new CenterOfMassAccelerationFromFullRobotModelStealer(perfectFullRobotModel.getElevator(), perfectSpatialAccelerationCalculator);
         ControlFlowOutputPort<FrameVector> desiredCenterOfMassAccelerationOutputPort = centerOfMassAccelerationFromFullRobotModelStealer.getOutputPort();

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
                 estimatedFullRobotModel.getRootInverseDynamicsJoint(), estimationLink, estimationFrame, desiredAngularAccelerationOutputPort,
                 desiredCenterOfMassAccelerationOutputPort, registry);
      }
      else
      {
         ComposableOrientationEstimatorCreator orientationEstimatorCreator = new ComposableOrientationEstimatorCreator(angularAccelerationNoiseCovariance,
                                                                                estimationLink, estimatedTwistCalculator);
         orientationEstimatorCreator.addOrientationSensorConfigurations(orientationSensorConfigurations);
         orientationEstimatorCreator.addAngularVelocitySensorConfigurations(angularVelocitySensorConfigurations);

         orientationEstimator = orientationEstimatorCreator.createOrientationEstimator(controlFlowGraph, controlDT, estimationFrame,
                 desiredAngularAccelerationOutputPort, registry);
      }

      controlFlowGraph.initializeAfterConnections();

      if (INITIALIZE_ANGULAR_VELOCITY_ESTIMATE_TO_ACTUAL)
      {
         robot.update();
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

         System.out.println("Estimated orientation = " + orientationEstimator.getEstimatedOrientation());
         System.out.println("Estimated angular velocity = " + estimatedAngularVelocity);
      }

   }

   private ArrayList<AngularVelocitySensorConfiguration> createAngularVelocitySensors(StateEstimatorEvaluatorFullRobotModel perfectFullRobotModel,
           StateEstimatorEvaluatorFullRobotModel estimatedFullRobotModel)
   {
      ArrayList<AngularVelocitySensorConfiguration> angularVelocitySensorConfigurations = new ArrayList<AngularVelocitySensorConfiguration>();

      if (CREATE_ANGULAR_VELOCITY_SENSOR)
      {
         for (IMUMount imuMount : perfectFullRobotModel.getIMUMounts())
         {
            String sensorName = imuMount.getName() + "AngularVelocity";

            RigidBody perfectMeasurmentBody = perfectFullRobotModel.getIMUBody(imuMount);
            ReferenceFrame perfectMeasurementFrame = createMeasurementFrame(sensorName, "PerfectMeasurementFrame", imuMount, perfectMeasurmentBody);

            SimulatedAngularVelocitySensor angularVelocitySensor = new SimulatedAngularVelocitySensor(sensorName, perfectTwistCalculator,
                                                                      perfectMeasurmentBody, perfectMeasurementFrame, registry);
            GaussianVectorCorruptor angularVelocityCorruptor = new GaussianVectorCorruptor(1235L, sensorName, registry);
            angularVelocityCorruptor.setStandardDeviation(angularVelocityMeasurementStandardDeviation);
            if (CORRUPT_SIMULATED_SENSORS)
               angularVelocitySensor.addSignalCorruptor(angularVelocityCorruptor);

            RandomWalkBiasVectorCorruptor biasVectorCorruptor = new RandomWalkBiasVectorCorruptor(1236L, sensorName, controlDT, registry);
            biasVectorCorruptor.setBias(computeGazeboBiasVector(gazeboAngularVelocityBiasMean, gazeboAngularVelocityBiasStandardDeviation, random));    // new Vector3d(0.0, 0.0, 0.0));

            if (ALLOW_CHANGING_BIASES)
            {
               biasVectorCorruptor.setStandardDeviation(angularVelocityBiasProcessNoiseStandardDeviation);
            }

            angularVelocitySensor.addSignalCorruptor(biasVectorCorruptor);

            DenseMatrix64F angularVelocityNoiseCovariance = createDiagonalCovarianceMatrix(angularVelocityMeasurementStandardDeviation, 3);
            DenseMatrix64F angularVelocityBiasProcessNoiseCovariance = createDiagonalCovarianceMatrix(angularVelocityBiasProcessNoiseStandardDeviation, 3);

            RigidBody estimatedMeasurementBody = estimatedFullRobotModel.getIMUBody(imuMount);
            ReferenceFrame estimatedMeasurementFrame = createMeasurementFrame(sensorName, "EstimatedMeasurementFrame", imuMount, estimatedMeasurementBody);

            AngularVelocitySensorConfiguration angularVelocitySensorConfiguration =
               new AngularVelocitySensorConfiguration(angularVelocitySensor.getAngularVelocityOutputPort(), sensorName, estimatedMeasurementBody,
                  estimatedMeasurementFrame, angularVelocityNoiseCovariance, angularVelocityBiasProcessNoiseCovariance);
            angularVelocitySensorConfigurations.add(angularVelocitySensorConfiguration);
         }
      }

      return angularVelocitySensorConfigurations;
   }

   private ArrayList<LinearAccelerationSensorConfiguration> createLinearAccelerationSensors(StateEstimatorEvaluatorFullRobotModel perfectFullRobotModel,
           StateEstimatorEvaluatorFullRobotModel estimatedFullRobotModel)
   {
      ArrayList<LinearAccelerationSensorConfiguration> linearAccelerationSensorConfigurations = new ArrayList<LinearAccelerationSensorConfiguration>();

      if (CREATE_LINEAR_ACCELERATION_SENSOR)
      {
         for (IMUMount imuMount : perfectFullRobotModel.getIMUMounts())
         {
            String sensorName = imuMount.getName() + "LinearAcceleration";

            RigidBody perfectMeasurementBody = perfectFullRobotModel.getIMUBody(imuMount);
            ReferenceFrame perfectMeasurementFrame = createMeasurementFrame(sensorName, "PerfectMeasurementFrame", imuMount, perfectMeasurementBody);

            SimulatedLinearAccelerationSensor linearAccelerationSensor = new SimulatedLinearAccelerationSensor(sensorName, perfectMeasurementBody,
                                                                            perfectMeasurementFrame, perfectSpatialAccelerationCalculator,
                                                                            gravitationalAcceleration, registry);

            GaussianVectorCorruptor linearAccelerationCorruptor = new GaussianVectorCorruptor(1237L, sensorName, registry);
            linearAccelerationCorruptor.setStandardDeviation(linearAccelerationMeasurementStandardDeviation);
            if (CORRUPT_SIMULATED_SENSORS)
               linearAccelerationSensor.addSignalCorruptor(linearAccelerationCorruptor);

            RandomWalkBiasVectorCorruptor biasVectorCorruptor = new RandomWalkBiasVectorCorruptor(1286L, sensorName, controlDT, registry);
            biasVectorCorruptor.setBias(computeGazeboBiasVector(gazeboLinearAccelerationBiasMean, gazeboLinearAccelerationBiasStandardDeviation, random));    // new Vector3d(0.0, 0.0, 0.0));

            if (ALLOW_CHANGING_BIASES)
            {
               biasVectorCorruptor.setStandardDeviation(linearAccelerationBiasProcessNoiseStandardDeviation);
            }

            linearAccelerationSensor.addSignalCorruptor(biasVectorCorruptor);

            DenseMatrix64F linearAccelerationNoiseCovariance = createDiagonalCovarianceMatrix(linearAccelerationMeasurementStandardDeviation, 3);
            DenseMatrix64F linearAccelerationBiasProcessNoiseCovariance = createDiagonalCovarianceMatrix(linearAccelerationBiasProcessNoiseStandardDeviation,
                                                                             3);

            RigidBody estimatedMeasurementBody = estimatedFullRobotModel.getIMUBody(imuMount);
            ReferenceFrame estimatedMeasurementFrame = createMeasurementFrame(sensorName, "EstimatedMeasurementFrame", imuMount, estimatedMeasurementBody);

            LinearAccelerationSensorConfiguration linearAccelerationSensorConfiguration =
               new LinearAccelerationSensorConfiguration(linearAccelerationSensor.getLinearAccelerationOutputPort(), sensorName, estimatedMeasurementBody,
                  estimatedMeasurementFrame, gravitationalAcceleration.getZ(), linearAccelerationNoiseCovariance, linearAccelerationBiasProcessNoiseCovariance);

            linearAccelerationSensorConfigurations.add(linearAccelerationSensorConfiguration);
         }
      }

      return linearAccelerationSensorConfigurations;
   }

   private ArrayList<PointVelocitySensorConfiguration> createPointVelocitySensors(StateEstimatorEvaluatorFullRobotModel perfectFullRobotModel,
           StateEstimatorEvaluatorFullRobotModel estimatedFullRobotModel)
   {
      ArrayList<PointVelocitySensorConfiguration> pointVelocitySensorConfigurations = new ArrayList<PointVelocitySensorConfiguration>();

      if (CREATE_POINT_VELOCITY_SENSOR)
      {
         for (KinematicPoint kinematicPoint : perfectFullRobotModel.getVelocityPoints())
         {
            String sensorName = kinematicPoint.getName() + "PointVelocity";

            RigidBody perfectMeasurementBody = perfectFullRobotModel.getVelocityPointBody(kinematicPoint);
            ReferenceFrame perfectFrameAfterJoint = perfectMeasurementBody.getParentJoint().getFrameAfterJoint();
            FramePoint perfectVelocityPoint = new FramePoint(perfectFrameAfterJoint, kinematicPoint.getOffsetCopy());

            SimulatedPointVelocitySensor pointVelocitySensor = new SimulatedPointVelocitySensor(sensorName, perfectMeasurementBody, perfectVelocityPoint,
                                                                  perfectTwistCalculator, registry);

            GaussianVectorCorruptor pointVelocityCorruptor = new GaussianVectorCorruptor(1257L, sensorName, registry);
            pointVelocityCorruptor.setStandardDeviation(pointVelocityMeasurementStandardDeviation);
            if (CORRUPT_SIMULATED_SENSORS)
               pointVelocitySensor.addSignalCorruptor(pointVelocityCorruptor);

            DenseMatrix64F pointVelocityNoiseCovariance = createDiagonalCovarianceMatrix(pointVelocityMeasurementStandardDeviation, 3);

            RigidBody estimatedMeasurementBody = estimatedFullRobotModel.getVelocityPointBody(kinematicPoint);
            ReferenceFrame estimatedFrameAfterJoint = estimatedMeasurementBody.getParentJoint().getFrameAfterJoint();
            FramePoint estimatedVelocityPoint = new FramePoint(estimatedFrameAfterJoint, kinematicPoint.getOffsetCopy());

            PointVelocitySensorConfiguration pointVelocitySensorConfiguration =
               new PointVelocitySensorConfiguration(pointVelocitySensor.getPointVelocityOutputPort(), sensorName, estimatedMeasurementBody,
                  estimatedVelocityPoint, angularAccelerationProcessNoiseStandardDeviation, pointVelocityNoiseCovariance, pointVelocityNoiseCovariance);

            pointVelocitySensorConfigurations.add(pointVelocitySensorConfiguration);
         }
      }

      return pointVelocitySensorConfigurations;
   }

   private ArrayList<OrientationSensorConfiguration> createOrientationSensors(StateEstimatorEvaluatorFullRobotModel perfectFullRobotModel,
           StateEstimatorEvaluatorFullRobotModel estimatedFullRobotModel)
   {
      ArrayList<OrientationSensorConfiguration> orientationSensorConfigurations = new ArrayList<OrientationSensorConfiguration>();

      if (CREATE_ORIENTATION_SENSOR)
      {
         for (IMUMount imuMount : perfectFullRobotModel.getIMUMounts())
         {
            String sensorName = imuMount.getName() + "Orientation";

            RigidBody perfectMeasurementBody = perfectFullRobotModel.getIMUBody(imuMount);
            ReferenceFrame perfectMeasurementFrame = createMeasurementFrame(sensorName, "PerfectMeasurementFrame", imuMount, perfectMeasurementBody);

            SimulatedOrientationSensor sensor = new SimulatedOrientationSensor(sensorName, perfectMeasurementFrame, registry);
            GaussianOrientationCorruptor orientationCorruptor = new GaussianOrientationCorruptor(sensorName, 12334255L, registry);
            orientationCorruptor.setStandardDeviation(orientationMeasurementStandardDeviation);
            if (CORRUPT_SIMULATED_SENSORS)
               sensor.addSignalCorruptor(orientationCorruptor);

            DenseMatrix64F orientationNoiseCovariance = createDiagonalCovarianceMatrix(orientationMeasurementStandardDeviation, 3);

            RigidBody estimatedMeasurementBody = estimatedFullRobotModel.getIMUBody(imuMount);
            ReferenceFrame estimatedMeasurementFrame = createMeasurementFrame(sensorName, "EstimatedMeasurementFrame", imuMount, estimatedMeasurementBody);

            OrientationSensorConfiguration orientationSensorConfiguration = new OrientationSensorConfiguration(sensor.getOrientationOutputPort(), sensorName,
                                                                               estimatedMeasurementFrame, orientationNoiseCovariance);
            orientationSensorConfigurations.add(orientationSensorConfiguration);
         }
      }

      return orientationSensorConfigurations;
   }


   public ReferenceFrame createMeasurementFrame(String sensorName, String frameName, IMUMount imuMount, RigidBody perfectMeasurmentBody)
   {
      Transform3D transformFromMountToJoint = new Transform3D();
      imuMount.getTransformFromMountToJoint(transformFromMountToJoint);

      ReferenceFrame perfectFrameAfterJoint = perfectMeasurmentBody.getParentJoint().getFrameAfterJoint();

      if (transformFromMountToJoint.epsilonEquals(new Transform3D(), 1e-10))
      {
         return perfectFrameAfterJoint;
      }

      ReferenceFrame perfectMeasurementFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent(sensorName + frameName, perfectFrameAfterJoint,
                                                  transformFromMountToJoint);

      return perfectMeasurementFrame;
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
      perfectFullRobotModel.updateBasedOnRobot(robot, true);
      perfectTwistCalculator.compute();
      perfectSpatialAccelerationCalculator.compute();

      updateInternalState();
      updateGroundTruth();

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

   private void updateGroundTruth()
   {
      FramePoint com = new FramePoint();
      perfectCenterOfMassCalculator.compute();
      perfectCenterOfMassCalculator.packCenterOfMass(com);
      perfectCoM.set(com);

      FrameVector comd = new FrameVector();
      perfectCenterOfMassJacobian.compute();
      perfectCenterOfMassJacobian.packCenterOfMassVelocity(comd);
      comd.changeFrame(ReferenceFrame.getWorldFrame());
      perfectCoMd.set(comd);

      FrameVector comdd = new FrameVector();
      perfectCenterOfMassAccelerationCalculator.packCoMAcceleration(comdd);
      comdd.changeFrame(ReferenceFrame.getWorldFrame());
      perfectCoMdd.set(comdd);
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


   private class AngularAccelerationFromRobotStealer extends AbstractControlFlowElement
   {
      private final StateEstimatorEstimatorEvaluatorRobot robot;
      private final ControlFlowOutputPort<FrameVector> outputPort = createOutputPort();
      private final FrameVector desiredAngularAcceleration;
      private final GaussianVectorCorruptor signalCorruptor = new GaussianVectorCorruptor(123412L, getClass().getSimpleName() + "Corruptor", registry);

      public AngularAccelerationFromRobotStealer(StateEstimatorEstimatorEvaluatorRobot robot, ReferenceFrame referenceFrame)
      {
         this.robot = robot;
         this.desiredAngularAcceleration = new FrameVector(referenceFrame);
         double discreteStdDev = convertStandardDeviationToDiscreteTime(angularAccelerationProcessNoiseStandardDeviation);
         signalCorruptor.setStandardDeviation(discreteStdDev);
      }

      public void startComputation()
      {
         desiredAngularAcceleration.set(robot.getActualAngularAccelerationInBodyFrame());
         if (CORRUPT_SIMULATED_SENSORS)
            signalCorruptor.corrupt(desiredAngularAcceleration.getVector());
         outputPort.setData(desiredAngularAcceleration);
      }

      public ControlFlowOutputPort<FrameVector> getOutputPort()
      {
         return outputPort;
      }

      public void waitUntilComputationIsDone()
      {
      }

   }


   private class CenterOfMassAccelerationFromFullRobotModelStealer extends AbstractControlFlowElement
   {
      private final CenterOfMassAccelerationCalculator centerOfMassAccelerationCalculator;
      private final FrameVector comAcceleration = new FrameVector(ReferenceFrame.getWorldFrame());
      private final ControlFlowOutputPort<FrameVector> outputPort = createOutputPort();
      private final GaussianVectorCorruptor signalCorruptor = new GaussianVectorCorruptor(123412L, getClass().getSimpleName() + "Corruptor", registry);

      public CenterOfMassAccelerationFromFullRobotModelStealer(RigidBody rootBody, SpatialAccelerationCalculator spatialAccelerationCalculator)
      {
         this.centerOfMassAccelerationCalculator = new CenterOfMassAccelerationCalculator(rootBody, spatialAccelerationCalculator);

         double discreteStdDev = convertStandardDeviationToDiscreteTime(comAccelerationProcessNoiseStandardDeviation);
         signalCorruptor.setStandardDeviation(discreteStdDev);
      }

      public ControlFlowOutputPort<FrameVector> getOutputPort()
      {
         return outputPort;
      }

      public void startComputation()
      {
         centerOfMassAccelerationCalculator.packCoMAcceleration(comAcceleration);
         comAcceleration.changeFrame(ReferenceFrame.getWorldFrame());
         if (CORRUPT_SIMULATED_SENSORS)
            signalCorruptor.corrupt(comAcceleration.getVector());
         outputPort.setData(comAcceleration);
      }

      public void waitUntilComputationIsDone()
      {
      }
   }


   private static DenseMatrix64F createDiagonalCovarianceMatrix(double standardDeviation, int size)
   {
      DenseMatrix64F orientationCovarianceMatrix = new DenseMatrix64F(size, size);
      CommonOps.setIdentity(orientationCovarianceMatrix);
      CommonOps.scale(MathTools.square(standardDeviation), orientationCovarianceMatrix);

      return orientationCovarianceMatrix;
   }


   private double convertStandardDeviationToDiscreteTime(double continuousStdDev)
   {
      double continuousVariance = MathTools.square(continuousStdDev);
      double discreteVariance = continuousVariance * controlDT;
      double discreteStdDev = Math.sqrt(discreteVariance);

      return discreteStdDev;
   }

   private Vector3d computeGazeboBiasVector(double mean, double standardDeviation, Random random)
   {
      Vector3d ret = new Vector3d();
      for (Direction direction : Direction.values())
      {
         MathTools.set(ret, direction, computeGazeboBias(mean, standardDeviation, random));
      }

      return ret;
   }

   // Pull request
   // https://bitbucket.org/osrf/gazebo/pull-request/421/added-noise-to-rates-and-accels-with-test/diff
   //
   //// Sample the bias that we'll use later
   // this->accelBias = math::Rand::GetDblNormal(accelBiasMean,
   // accelBiasStddev);
   //// With equal probability, we pick a negative bias (by convention,
   //// accelBiasMean should be positive, though it would work fine if
   //// negative).
   // if (math::Rand::GetDblUniform() < 0.5)
   // this->accelBias = -this->accelBias;

   private double computeGazeboBias(double mean, double standardDeviation, Random random)
   {
      double ret = standardDeviation * random.nextGaussian() + mean;
      if (random.nextBoolean())
         ret = -ret;

      return ret;
   }

}
