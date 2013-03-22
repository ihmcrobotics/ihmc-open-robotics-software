package us.ihmc.commonWalkingControlModules.stateEstimation;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.controlFlow.AbstractControlFlowElement;
import us.ihmc.controlFlow.ControlFlowGraph;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.sensorProcessing.signalCorruption.BiasVectorCorruptor;
import us.ihmc.sensorProcessing.signalCorruption.GaussianOrientationCorruptor;
import us.ihmc.sensorProcessing.signalCorruption.GaussianVectorCorruptor;
import us.ihmc.sensorProcessing.simulatedSensors.SimulatedAngularVelocitySensor;
import us.ihmc.sensorProcessing.simulatedSensors.SimulatedOrientationSensor;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.RigidBodyInertia;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.FloatingJoint;
import com.yobotics.simulationconstructionset.Link;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.RobotController;

public class QuaternionOrientationEstimatorEvaluator
{
   private static final boolean INITIALIZE_ANGULAR_VELOCITY_ESTIMATE_TO_ACTUAL = false;
   private static final boolean USE_ANGULAR_ACCELERATION_INPUT = true;
   private static final boolean CREATE_ORIENTATION_SENSOR = true;
   private static final boolean CREATE_ANGULAR_VELOCITY_SENSOR = true;
   private static final boolean USE_COMPOSABLE_ESTIMATOR = true;

   private final double orientationMeasurementStandardDeviation = Math.sqrt(1e-1);    // 1e0); //1e-1);
   private final double angularVelocityMeasurementStandardDeviation = Math.sqrt(1e-1);    // 1e-2);
   private final double angularAccelerationProcessNoiseStandardDeviation = Math.sqrt(1.0);
   private final double angularVelocityBiasProcessNoiseStandardDeviation = Math.sqrt(1e-2);

   private final double controlDT = 0.005;

   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   public QuaternionOrientationEstimatorEvaluator()
   {
      QuaternionOrientationEstimatorEvaluatorRobot robot = new QuaternionOrientationEstimatorEvaluatorRobot();
      QuaternionOrientationEstimatorEvaluatorController controller = new QuaternionOrientationEstimatorEvaluatorController(robot, controlDT);
      robot.setController(controller);

      SimulationConstructionSet scs = new SimulationConstructionSet(robot, 32000);
      scs.addYoVariableRegistry(registry);

      scs.setDT(controlDT, 1);
      scs.setSimulateDuration(45.0);
      scs.startOnAThread();
   }

   private class QuaternionOrientationEstimatorEvaluatorRobot extends Robot
   {
      private static final long serialVersionUID = 2647791981594204134L;
      private final Link bodyLink;
      private final FloatingJoint rootJoint;

      public QuaternionOrientationEstimatorEvaluatorRobot()
      {
         super("QuaternionOrientationEstimatorEvaluatorRobot");

         rootJoint = new FloatingJoint("root", new Vector3d(), this);

         bodyLink = new Link("body");
         bodyLink.setMassAndRadiiOfGyration(10.0, 0.1, 0.2, 0.3);

         Graphics3DObject bodyLinkGraphics = new Graphics3DObject();
         bodyLinkGraphics.translate(0.0, 0.0, -0.15);
         bodyLinkGraphics.addCube(0.1, 0.2, 0.3, YoAppearance.Red());
         bodyLink.setLinkGraphics(bodyLinkGraphics);
         rootJoint.setLink(bodyLink);

         this.addRootJoint(rootJoint);

         this.setGravity(0.0);

         rootJoint.setPosition(new Point3d(0.0, 0.0, 0.4));
         rootJoint.setAngularVelocityInBody(new Vector3d(0.2, 2.5, 0.0));
         update();
      }

      public Link getBodyLink()
      {
         return bodyLink;
      }

      public FloatingJoint getRootJoint()
      {
         return rootJoint;
      }

      public Vector3d getActualAngularAccelerationInBodyFrame()
      {
         return rootJoint.getAngularAccelerationInBody();
      }
   }


   private class QuaternionOrientationEstimatorEvaluatorFullRobotModel
   {
      private final RigidBody elevator;
      private final SixDoFJoint rootInverseDynamicsJoint;
      private final RigidBody body;

      public QuaternionOrientationEstimatorEvaluatorFullRobotModel(QuaternionOrientationEstimatorEvaluatorRobot robot)
      {
         ReferenceFrame inertialFrame = ReferenceFrame.getWorldFrame();
         ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", inertialFrame, new Transform3D());
         elevator = new RigidBody("elevator", elevatorFrame);

         rootInverseDynamicsJoint = new SixDoFJoint("root", elevator, elevatorFrame);
         body = copyLinkAsRigidBody(robot.getBodyLink(), rootInverseDynamicsJoint, "body");
      }

      public SixDoFJoint getRootInverseDynamicsJoint()
      {
         return rootInverseDynamicsJoint;
      }

      public RigidBody getBody()
      {
         return body;
      }

      public void updateBasedOnRobot(QuaternionOrientationEstimatorEvaluatorRobot robot)
      {
         Transform3D temporaryRootToWorldTransform = new Transform3D();
         robot.getRootJoint().getTransformToWorld(temporaryRootToWorldTransform);
         rootInverseDynamicsJoint.setPositionAndRotation(temporaryRootToWorldTransform);
         elevator.updateFramesRecursively();

         ReferenceFrame elevatorFrame = rootInverseDynamicsJoint.getFrameBeforeJoint();
         ReferenceFrame bodyFrame = rootInverseDynamicsJoint.getFrameAfterJoint();

         Vector3d linearVelocity = new Vector3d();
         robot.getRootJoint().getVelocity(linearVelocity);
         FrameVector linearVelocityFrameVector = new FrameVector(ReferenceFrame.getWorldFrame(), linearVelocity);
         linearVelocityFrameVector.changeFrame(bodyFrame);

         Vector3d angularVelocity = new Vector3d();
         robot.getRootJoint().getAngularVelocityInBody(angularVelocity);
         FrameVector angularVelocityFrameVector = new FrameVector(bodyFrame, angularVelocity);
         angularVelocityFrameVector.changeFrame(bodyFrame);

         Twist bodyTwist = new Twist(bodyFrame, elevatorFrame, bodyFrame, linearVelocityFrameVector.getVector(), angularVelocityFrameVector.getVector());
         rootInverseDynamicsJoint.setJointTwist(bodyTwist);
      }


      public void updateBasedOnEstimator(OrientationEstimator estimator)
      {
         FrameOrientation estimatedOrientation = estimator.getEstimatedOrientation();
         FrameVector estimatedAngularVelocity = estimator.getEstimatedAngularVelocity();

         updateBasedOnEstimator(estimatedOrientation, estimatedAngularVelocity);
      }

      public void updateBasedOnEstimator(FrameOrientation estimatedOrientation, FrameVector estimatedAngularVelocity)
      {
         rootInverseDynamicsJoint.setRotation(estimatedOrientation.getQuaternion());
         elevator.updateFramesRecursively();

         ReferenceFrame elevatorFrame = rootInverseDynamicsJoint.getFrameBeforeJoint();
         ReferenceFrame bodyFrame = rootInverseDynamicsJoint.getFrameAfterJoint();

         estimatedAngularVelocity.changeFrame(bodyFrame);

         Twist bodyTwist = new Twist(bodyFrame, elevatorFrame, bodyFrame);
         bodyTwist.setAngularPart(estimatedAngularVelocity.getVector());
         rootInverseDynamicsJoint.setJointTwist(bodyTwist);
      }
   }


   private class AngularAccelerationFromRobotStealer extends AbstractControlFlowElement
   {
      private final QuaternionOrientationEstimatorEvaluatorRobot robot;
      private final ControlFlowOutputPort<FrameVector> outputPort;
      private final FrameVector desiredAngularAcceleration;

      public AngularAccelerationFromRobotStealer(QuaternionOrientationEstimatorEvaluatorRobot robot, ReferenceFrame referenceFrame)
      {
         this.robot = robot;

         this.desiredAngularAcceleration = new FrameVector(referenceFrame);
         this.outputPort = createOutputPort();
      }

      public void startComputation()
      {
         desiredAngularAcceleration.set(robot.getActualAngularAccelerationInBodyFrame());

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


   private class QuaternionOrientationEstimatorEvaluatorController implements RobotController
   {
      private final YoVariableRegistry registry = new YoVariableRegistry("QuaternionOrientationEstimatorEvaluatorController");

      private final QuaternionOrientationEstimatorEvaluatorRobot robot;

      private final QuaternionOrientationEstimatorEvaluatorFullRobotModel perfectFullRobotModel;
      private final TwistCalculator perfectTwistCalculator;

      private final QuaternionOrientationEstimatorEvaluatorFullRobotModel estimatedFullRobotModel;
      private final TwistCalculator estimatedTwistCalculator;

      private final ControlFlowGraph controlFlowGraph;
      private final OrientationEstimator orientationEstimator;

      public QuaternionOrientationEstimatorEvaluatorController(QuaternionOrientationEstimatorEvaluatorRobot robot, double controlDT)
      {
         this.robot = robot;

         perfectFullRobotModel = new QuaternionOrientationEstimatorEvaluatorFullRobotModel(robot);
         perfectTwistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), perfectFullRobotModel.getBody());
         estimatedFullRobotModel = new QuaternionOrientationEstimatorEvaluatorFullRobotModel(robot);
         estimatedTwistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), estimatedFullRobotModel.getBody());

         OrientationSensorConfiguration<ControlFlowOutputPort<Matrix3d>> orientationSensors = createOrientationSensors(perfectFullRobotModel,
                                                                                                 estimatedFullRobotModel);
         AngularVelocitySensorConfiguration<ControlFlowOutputPort<Vector3d>> angularVelocitySensors = createAngularVelocitySensors(perfectFullRobotModel,
                                                                                                         estimatedFullRobotModel);

         controlFlowGraph = new ControlFlowGraph();
         ReferenceFrame estimationFrame = estimatedFullRobotModel.getRootInverseDynamicsJoint().getFrameAfterJoint();
         RigidBody estimationLink = estimatedFullRobotModel.getBody();
         DenseMatrix64F angularAccelerationNoiseCovariance = createDiagonalCovarianceMatrix(angularAccelerationProcessNoiseStandardDeviation, 3);

         if (USE_COMPOSABLE_ESTIMATOR)
         {
            OrientationEstimatorCreator orientationEstimatorCreator = new OrientationEstimatorCreator(angularAccelerationNoiseCovariance, estimationLink, estimatedTwistCalculator);
            orientationEstimatorCreator.addOrientationSensorConfigurations(orientationSensors);
            orientationEstimatorCreator.addAngularVelocitySensorConfigurations(angularVelocitySensors);
            orientationEstimator = orientationEstimatorCreator.createOrientationEstimator(controlFlowGraph, controlDT, estimationFrame, registry);
         }
         else
         {
            orientationEstimator = new QuaternionOrientationEstimator(controlFlowGraph, "orientationEstimator", orientationSensors, angularVelocitySensors,
                  estimationLink, estimationFrame, estimatedTwistCalculator, controlDT, registry);            
         }

         orientationEstimator.setAngularAccelerationNoiseCovariance(angularAccelerationNoiseCovariance);

         if (USE_ANGULAR_ACCELERATION_INPUT)
         {
            AngularAccelerationFromRobotStealer angularAccelerationFromRobotStealer = new AngularAccelerationFromRobotStealer(robot, estimationFrame);

            controlFlowGraph.connectElements(angularAccelerationFromRobotStealer.getOutputPort(), orientationEstimator.getAngularAccelerationInputPort());
         }

         controlFlowGraph.initializeAfterConnections();


         if (INITIALIZE_ANGULAR_VELOCITY_ESTIMATE_TO_ACTUAL)
         {
            DenseMatrix64F x = orientationEstimator.getState();
            MatrixTools.insertTuple3dIntoEJMLVector(robot.getRootJoint().getAngularVelocityInBody(), x, 3);
            orientationEstimator.setState(x, orientationEstimator.getCovariance());
         }

      }

      private AngularVelocitySensorConfiguration<ControlFlowOutputPort<Vector3d>> createAngularVelocitySensors(
              QuaternionOrientationEstimatorEvaluatorFullRobotModel perfectFullRobotModel,
              QuaternionOrientationEstimatorEvaluatorFullRobotModel estimatedFullRobotModel)
      {
         AngularVelocitySensorConfiguration<ControlFlowOutputPort<Vector3d>> angularVelocitySensorConfiguration =
            new AngularVelocitySensorConfiguration<ControlFlowOutputPort<Vector3d>>();

         if (CREATE_ANGULAR_VELOCITY_SENSOR)
         {
            ReferenceFrame frameUsedForPerfectMeasurement = perfectFullRobotModel.getRootInverseDynamicsJoint().getFrameAfterJoint();
            SimulatedAngularVelocitySensor angularVelocitySensor = new SimulatedAngularVelocitySensor("imu1AngularVelocity", perfectTwistCalculator,
                                                                      perfectFullRobotModel.getBody(), frameUsedForPerfectMeasurement, registry);
            GaussianVectorCorruptor angularVelocityCorruptor = new GaussianVectorCorruptor(1235L, "gaussianAngularVelocity", registry);
            angularVelocityCorruptor.setStandardDeviation(angularVelocityMeasurementStandardDeviation);
            angularVelocitySensor.addSignalCorruptor(angularVelocityCorruptor);

            BiasVectorCorruptor biasVectorCorruptor = new BiasVectorCorruptor(1236L, "angularVelocityBiasCorruptor", controlDT, registry);
            biasVectorCorruptor.setStandardDeviation(angularVelocityBiasProcessNoiseStandardDeviation);
            biasVectorCorruptor.setBias(new Vector3d(0.0, 0.0, 0.0));
            angularVelocitySensor.addSignalCorruptor(biasVectorCorruptor);


            DenseMatrix64F angularVelocityCovarianceMatrix = createDiagonalCovarianceMatrix(angularVelocityMeasurementStandardDeviation, 3);
            DenseMatrix64F angularVelocityBiasNoiseCovariance = createDiagonalCovarianceMatrix(angularVelocityBiasProcessNoiseStandardDeviation, 3);
            ReferenceFrame measurementFrame = estimatedFullRobotModel.getRootInverseDynamicsJoint().getFrameAfterJoint();
            RigidBody estimatedBody = estimatedFullRobotModel.getBody();
            angularVelocitySensorConfiguration.addSensor(angularVelocitySensor.getAngularVelocityOutputPort(), measurementFrame, estimatedBody,
                    "imu1AngularVelocity", angularVelocityCovarianceMatrix, angularVelocityBiasNoiseCovariance);
         }

         return angularVelocitySensorConfiguration;
      }

      private OrientationSensorConfiguration<ControlFlowOutputPort<Matrix3d>> createOrientationSensors(
              QuaternionOrientationEstimatorEvaluatorFullRobotModel perfectFullRobotModel,
              QuaternionOrientationEstimatorEvaluatorFullRobotModel estimatedFullRobotModel)
      {
         OrientationSensorConfiguration<ControlFlowOutputPort<Matrix3d>> orientationSensorConfiguration =
            new OrientationSensorConfiguration<ControlFlowOutputPort<Matrix3d>>();

         if (CREATE_ORIENTATION_SENSOR)
         {
            ReferenceFrame frameUsedForPerfectMeasurement = perfectFullRobotModel.getRootInverseDynamicsJoint().getFrameAfterJoint();
            SimulatedOrientationSensor sensor = new SimulatedOrientationSensor("imu1Orientation", frameUsedForPerfectMeasurement, registry);
            GaussianOrientationCorruptor orientationCorruptor = new GaussianOrientationCorruptor("gaussianOrientation", 12345L, registry);
            orientationCorruptor.setStandardDeviation(orientationMeasurementStandardDeviation);
            sensor.addSignalCorruptor(orientationCorruptor);

            DenseMatrix64F orientationCovarianceMatrix = createDiagonalCovarianceMatrix(orientationMeasurementStandardDeviation, 3);
            ReferenceFrame measurementFrame = estimatedFullRobotModel.getRootInverseDynamicsJoint().getFrameAfterJoint();

            orientationSensorConfiguration.addSensor(sensor.getOrientationOutputPort(), measurementFrame, "imu1Orientation", orientationCovarianceMatrix);
         }

         return orientationSensorConfiguration;
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
         perfectFullRobotModel.updateBasedOnRobot(robot);
         perfectTwistCalculator.compute();
         controlFlowGraph.startComputation();
         controlFlowGraph.waitUntilComputationIsDone();
         estimatedFullRobotModel.updateBasedOnEstimator(orientationEstimator);

         // TODO: set revolute joint positions and velocities
         estimatedTwistCalculator.compute();
      }
   }


   private static DenseMatrix64F createDiagonalCovarianceMatrix(double standardDeviation, int size)
   {
      DenseMatrix64F orientationCovarianceMatrix = new DenseMatrix64F(size, size);
      CommonOps.setIdentity(orientationCovarianceMatrix);
      CommonOps.scale(MathTools.square(standardDeviation), orientationCovarianceMatrix);

      return orientationCovarianceMatrix;
   }

   private static RigidBody copyLinkAsRigidBody(Link link, InverseDynamicsJoint currentInverseDynamicsJoint, String bodyName)
   {
      Vector3d comOffset = new Vector3d();
      link.getComOffset(comOffset);
      Matrix3d momentOfInertia = new Matrix3d();
      link.getMomentOfInertia(momentOfInertia);
      ReferenceFrame nextFrame = createOffsetFrame(currentInverseDynamicsJoint, comOffset, bodyName);
      nextFrame.update();
      RigidBodyInertia inertia = new RigidBodyInertia(nextFrame, momentOfInertia, link.getMass());
      RigidBody rigidBody = new RigidBody(bodyName, inertia, currentInverseDynamicsJoint);

      return rigidBody;
   }

   private static ReferenceFrame createOffsetFrame(InverseDynamicsJoint currentInverseDynamicsJoint, Vector3d offset, String frameName)
   {
      ReferenceFrame parentFrame = currentInverseDynamicsJoint.getFrameAfterJoint();
      Transform3D transformToParent = new Transform3D();
      transformToParent.set(offset);
      ReferenceFrame beforeJointFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent(frameName, parentFrame, transformToParent);

      return beforeJointFrame;
   }

   public static void main(String[] args)
   {
      new QuaternionOrientationEstimatorEvaluator();
   }
}
