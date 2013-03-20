package us.ihmc.commonWalkingControlModules.stateEstimation;

import java.util.ArrayList;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.controlFlow.ControlFlowGraph;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.sensorProcessing.signalCorruption.GaussianOrientationCorruptor;
import us.ihmc.sensorProcessing.signalCorruption.GaussianVectorCorruptor;
import us.ihmc.sensorProcessing.simulatedSensors.SimulatedAngularVelocitySensor;
import us.ihmc.sensorProcessing.simulatedSensors.SimulatedOrientationSensor;
import us.ihmc.utilities.math.MathTools;
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
// private static final Vector3d X = new Vector3d(1.0, 0.0, 0.0);
// private static final Vector3d Y = new Vector3d(0.0, 1.0, 0.0);
// private static final Vector3d Z = new Vector3d(0.0, 0.0, 1.0);

   private final double orientationMeasurementStandardDeviation = Math.sqrt(1e-1);
   private final double angularVelocityMeasurementStandardDeviation = Math.sqrt(1e-2);

   private final double angularAccelerationNoiseStandardDeviation = Math.sqrt(1.0);
   private final double angularVelocityBiasNoiseStandardDeviation = Math.sqrt(1.0);

   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   public QuaternionOrientationEstimatorEvaluator()
   {
      QuaternionOrientationEstimatorEvaluatorRobot robot = new QuaternionOrientationEstimatorEvaluatorRobot();
      double controlDT = 0.005;

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
         bodyLinkGraphics.addCube(0.1, 0.2, 0.3, YoAppearance.Red());
         bodyLink.setLinkGraphics(bodyLinkGraphics);
         rootJoint.setLink(bodyLink);

         this.addRootJoint(rootJoint);

         this.setGravity(0.0);

         rootJoint.setPosition(new Point3d(0.0, 0.0, 0.4));
         rootJoint.setAngularVelocityInBody(new Vector3d(0.0, 0.0, 0.1));
      }

      public Link getBodyLink()
      {
         return bodyLink;
      }

      public FloatingJoint getRootJoint()
      {
         return rootJoint;
      }
   }


   private class QuaternionOrientationEstimatorEvaluatorController implements RobotController
   {
      private final YoVariableRegistry registry = new YoVariableRegistry("QuaternionOrientationEstimatorEvaluatorController");

//    private final QuaternionOrientationEstimator estimator;
      private final QuaternionOrientationEstimatorEvaluatorRobot robot;

      private final ControlFlowGraph controlFlowGraph;
      private final TwistCalculator twistCalculator;
      private final RigidBody elevator;
      private final SixDoFJoint rootInverseDynamicsJoint;

      private final QuaternionOrientationEstimator quaternionOrientationEstimator;

      public QuaternionOrientationEstimatorEvaluatorController(QuaternionOrientationEstimatorEvaluatorRobot robot, double controlDT)
      {
         this.robot = robot;

         ReferenceFrame inertialFrame = ReferenceFrame.getWorldFrame();
         ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", inertialFrame, new Transform3D());
         elevator = new RigidBody("elevator", elevatorFrame);

         rootInverseDynamicsJoint = new SixDoFJoint("root", elevator, elevatorFrame);
         RigidBody body = copyLinkAsRigidBody(robot.getBodyLink(), rootInverseDynamicsJoint, "body");

         elevator.updateFramesRecursively();
         ReferenceFrame bodyFixedFrame = rootInverseDynamicsJoint.getFrameAfterJoint();

         ReferenceFrame estimationFrame = bodyFixedFrame; // FIXME: mixing 'perfect' frames used for simulating sensors with estimated frames here!

         ArrayList<SimulatedOrientationSensor> orientationSensors = new ArrayList<SimulatedOrientationSensor>();
         SimulatedOrientationSensor sensor = new SimulatedOrientationSensor("imu1Orientation", bodyFixedFrame, registry);
         GaussianOrientationCorruptor orientationCorruptor = new GaussianOrientationCorruptor("gaussianOrientation", 12345L, registry);
         orientationCorruptor.setStandardDeviation(orientationMeasurementStandardDeviation);
         sensor.addSignalCorruptor(orientationCorruptor);
         DenseMatrix64F orientationCovarianceMatrix = createDiagonalCovarianceMatrix(orientationMeasurementStandardDeviation, 3);
         sensor.setCovarianceMatrix(orientationCovarianceMatrix);
         orientationSensors.add(sensor);

         ArrayList<SimulatedAngularVelocitySensor> angularVelocitySensors = new ArrayList<SimulatedAngularVelocitySensor>();
         twistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), body);
         SimulatedAngularVelocitySensor angularVelocitySensor = new SimulatedAngularVelocitySensor("imu1AngularVelocity", twistCalculator, body,
                                                                   bodyFixedFrame, registry);
         GaussianVectorCorruptor angularVelocityCorruptor = new GaussianVectorCorruptor(1235L, "gaussianAngularVelocity", registry);
         angularVelocityCorruptor.setStandardDeviation(angularVelocityMeasurementStandardDeviation);
         angularVelocitySensor.addSignalCorruptor(angularVelocityCorruptor);
         DenseMatrix64F angularVelocityCovarianceMatrix = createDiagonalCovarianceMatrix(angularVelocityMeasurementStandardDeviation, 3);
         angularVelocitySensor.setCovarianceMatrix(angularVelocityCovarianceMatrix);

         angularVelocitySensors.add(angularVelocitySensor);

         controlFlowGraph = new ControlFlowGraph();
         quaternionOrientationEstimator = new QuaternionOrientationEstimator(controlFlowGraph, name, orientationSensors, angularVelocitySensors,
                 estimationFrame, controlDT, registry);
         DenseMatrix64F angularAccelerationNoiseCovariance = createDiagonalCovarianceMatrix(angularAccelerationNoiseStandardDeviation, 3);
         quaternionOrientationEstimator.setAngularAccelerationNoiseCovariance(angularAccelerationNoiseCovariance);

         for (SimulatedAngularVelocitySensor simulatedAngularVelocitySensor : angularVelocitySensors)
         {
            DenseMatrix64F angularVelocityBiasNoiseCovariance = createDiagonalCovarianceMatrix(angularVelocityBiasNoiseStandardDeviation, 3);
            quaternionOrientationEstimator.setAngularVelocityBiasNoiseCovariance(simulatedAngularVelocitySensor, angularVelocityBiasNoiseCovariance);
         }

         controlFlowGraph.initializeAfterConnections();
      }

      private RigidBody copyLinkAsRigidBody(Link link, InverseDynamicsJoint currentInverseDynamicsJoint, String bodyName)
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

      private ReferenceFrame createOffsetFrame(InverseDynamicsJoint currentInverseDynamicsJoint, Vector3d offset, String frameName)
      {
         ReferenceFrame parentFrame = currentInverseDynamicsJoint.getFrameAfterJoint();
         Transform3D transformToParent = new Transform3D();
         transformToParent.set(offset);
         ReferenceFrame beforeJointFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent(frameName, parentFrame, transformToParent);

         return beforeJointFrame;
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
         // FIXME: update joint configurations and velocities based on real robot
         updateConfigurationAndVelocity();
         twistCalculator.compute();
         controlFlowGraph.startComputation();
         controlFlowGraph.waitUntilComputationIsDone();

//       estimator.startComputation();
//       estimator.waitUntilComputationIsDone();
      }

      private void updateConfigurationAndVelocity()
      {
//         FrameOrientation estimatedOrientation = quaternionOrientationEstimator.getOrientationOutputPort().getData();
//
//         Transform3D temporaryRootToWorldTransform = new Transform3D();
//         robot.getRootJoint().getTransformToWorld(temporaryRootToWorldTransform);
//         rootInverseDynamicsJoint.setRotation(estimatedOrientation.getQuaternion());
//         elevator.updateFramesRecursively();
//
//         ReferenceFrame elevatorFrame = rootInverseDynamicsJoint.getFrameBeforeJoint();
//         ReferenceFrame bodyFrame = rootInverseDynamicsJoint.getFrameAfterJoint();
//
//         FrameVector angularVelocity = quaternionOrientationEstimator.getAngularVelocityOutputPort().getData();
//         angularVelocity.changeFrame(bodyFrame);
//
//         Twist bodyTwist = new Twist(bodyFrame, elevatorFrame, bodyFrame);
//         bodyTwist.setAngularPart(angularVelocity.getVector());
//         rootInverseDynamicsJoint.setJointTwist(bodyTwist);

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
   }


   private static DenseMatrix64F createDiagonalCovarianceMatrix(double standardDeviation, int size)
   {
      DenseMatrix64F orientationCovarianceMatrix = new DenseMatrix64F(size, size);
      CommonOps.setIdentity(orientationCovarianceMatrix);
      CommonOps.scale(MathTools.square(standardDeviation), orientationCovarianceMatrix);

      return orientationCovarianceMatrix;
   }


   public static void main(String[] args)
   {
      new QuaternionOrientationEstimatorEvaluator();
   }
}
