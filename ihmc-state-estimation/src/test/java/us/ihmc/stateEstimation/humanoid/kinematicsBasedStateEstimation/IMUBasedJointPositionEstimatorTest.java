package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;
import org.opentest4j.AssertionFailedError;

import cern.colt.Arrays;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.imu.IMUSensor;

public class IMUBasedJointPositionEstimatorTest
{
   private static final int ITERATIONS = 1000;

   @Test
   public void testNoPreferredConfiguration()
   {
      Random random = new Random();//2344332);

      for (int i = 0; i < ITERATIONS; i++)
      {
         List<OneDoFJointBasics> joints = nextRevoluteChain(random, Axis3D.Z, Axis3D.Y, Axis3D.X);
         RigidBodyBasics base = joints.get(0).getPredecessor();
         RigidBodyBasics endEffector = joints.get(joints.size() - 1).getSuccessor();

         // We keeping the parent leveled to world so the first joint is aligned with world z
         IMUSensor parentIMU = nextIMU(random, base, false);
         parentIMU.setOrientationMeasurement(new YawPitchRoll(EuclidCoreRandomTools.nextDouble(random, Math.PI), 0, 0));
         IMUSensor childIMU = nextIMU(random, endEffector, true);
         IMUBasedJointPositionEstimator estimator = new IMUBasedJointPositionEstimator("IMUEstimator", parentIMU, childIMU, null);

         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
         base.updateFramesRecursively();

         ReferenceFrame childIMUFrame = childIMU.getMeasurementFrame();
         ReferenceFrame parentIMUFrame = parentIMU.getMeasurementFrame();
         Quaternion childIMUOrientation = new Quaternion(childIMUFrame.getTransformToDesiredFrame(parentIMUFrame).getRotation());
         childIMUOrientation.preMultiply(parentIMU.getOrientationMeasurement());
         // We introduce misalignment between the 2 IMUs
         childIMUOrientation.prependYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI));
         childIMU.setOrientationMeasurement(childIMUOrientation);

         double[] qTarget = joints.stream().mapToDouble(OneDoFJointReadOnly::getQ).toArray();
         // Not disturbing the Z-axis joint for now.
         double[] qError = {0.0, EuclidCoreRandomTools.nextDouble(random, 0.05), EuclidCoreRandomTools.nextDouble(random, 0.05)};
         double[] qPerturbed = new double[joints.size()];

         for (int j = 0; j < joints.size(); j++)
         {
            qPerturbed[j] = qTarget[j] + qError[j];
            joints.get(j).setQ(qPerturbed[j]);
         }
         base.updateFramesRecursively();
         estimator.compute();

         double[] qCorrection = estimator.getQCorrection().data;

         try
         {
            assertEquals(0.0, qCorrection[0], 1.0e-4);
            assertEquals(qTarget[1], qPerturbed[1] + qCorrection[1], 1.0e-3);
            // Before doing assertions on j2 we check that it's axis is not aligned with Z.
            // If it is aligned, it is selected out.
            Vector3D j2Axis = new Vector3D(joints.get(2).getJointAxis());
            joints.get(2).getFrameBeforeJoint().transformFromThisToDesiredFrame(parentIMUFrame, j2Axis);
            parentIMU.getOrientationMeasurement().transform(j2Axis);
            if (Math.abs(Axis3D.Z.dot(j2Axis)) < 0.9)
               assertEquals(qTarget[2], qPerturbed[2] + qCorrection[2], EuclidCoreTools.interpolate(1.0e-3, 1.0e-2, Math.abs(Axis3D.Z.dot(j2Axis)))); // The joint gets progressively selected out when aligne
         }
         catch (AssertionFailedError e)
         {
            System.out.println("Iteration: " + i);
            System.out.println("qTarget: " + Arrays.toString(qTarget));
            System.out.println("qError : " + Arrays.toString(qError));
            System.out.println("qCorr  : " + Arrays.toString(qCorrection));
            System.out.println("parent : " + parentIMU.getOrientationMeasurement().toStringAsYawPitchRoll());
            System.out.println("child  : " + childIMU.getOrientationMeasurement().toStringAsYawPitchRoll());
            throw e;
         }
      }
   }

   @Test
   public void testWithPreferredConfiguration()
   {
      Random random = new Random();//2344332);

      for (int i = 0; i < ITERATIONS; i++)
      {
         List<OneDoFJointBasics> joints = nextRevoluteChain(random, Axis3D.Z, Axis3D.Y, Axis3D.X);
         RigidBodyBasics base = joints.get(0).getPredecessor();
         RigidBodyBasics endEffector = joints.get(joints.size() - 1).getSuccessor();

         IMUSensor parentIMU = nextIMU(random, base, false);
         parentIMU.setOrientationMeasurement(new YawPitchRoll(EuclidCoreRandomTools.nextDouble(random, Math.PI), 0, 0));
         IMUSensor childIMU = nextIMU(random, endEffector, true);
         IMUBasedJointPositionEstimator estimator = new IMUBasedJointPositionEstimator("IMUEstimator", parentIMU, childIMU, null);

         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
         base.updateFramesRecursively();

         ReferenceFrame childIMUFrame = childIMU.getMeasurementFrame();
         ReferenceFrame parentIMUFrame = parentIMU.getMeasurementFrame();
         Quaternion childIMUOrientation = new Quaternion(childIMUFrame.getTransformToDesiredFrame(parentIMUFrame).getRotation());
         childIMUOrientation.preMultiply(parentIMU.getOrientationMeasurement());
         // We introduce misalignment between the 2 IMUs
         childIMUOrientation.prependYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI));
         childIMU.setOrientationMeasurement(childIMUOrientation);

         double[] qTarget = joints.stream().mapToDouble(OneDoFJointReadOnly::getQ).toArray();
         double[] qError = {EuclidCoreRandomTools.nextDouble(random, 0.05),
                            EuclidCoreRandomTools.nextDouble(random, 0.05),
                            EuclidCoreRandomTools.nextDouble(random, 0.05)};
         double[] qPerturbed = new double[joints.size()];

         for (int j = 0; j < joints.size(); j++)
         {
            qPerturbed[j] = qTarget[j] + qError[j];
            joints.get(j).setQ(qPerturbed[j]);
         }
         base.updateFramesRecursively();
         estimator.setPreferredJointPositions(joint -> qTarget[joints.indexOf(joint)]);
         estimator.compute();

         double[] qCorrection = estimator.getQCorrection().data;
         double[] qEst = new double[qCorrection.length];
         for (int j = 0; j < joints.size(); j++)
         {
            qEst[j] = qPerturbed[j] + qCorrection[j];
         }

         try
         {
            assertEquals(qTarget[0], qPerturbed[0] + qCorrection[0], 2.0e-3, "Diff: " + Math.abs(qTarget[0] - (qPerturbed[0] + qCorrection[0])));
            assertEquals(qTarget[1], qPerturbed[1] + qCorrection[1], 2.0e-3, "Diff: " + Math.abs(qTarget[1] - (qPerturbed[1] + qCorrection[1])));

            // Before doing assertions on j2 we check that it's axis is not aligned with Z.
            // If it is aligned, it is selected out.
            Vector3D j2Axis = new Vector3D(joints.get(2).getJointAxis());
            joints.get(2).getFrameBeforeJoint().transformFromThisToDesiredFrame(parentIMUFrame, j2Axis);
            parentIMU.getOrientationMeasurement().transform(j2Axis);

            // TODO With the preferred configuration, I was expecting the error to be contained on the last joint regardless of the middle joint configuration
            assertEquals(qTarget[2],
                         qPerturbed[2] + qCorrection[2],
                         EuclidCoreTools.interpolate(1.0e-3, 2.0e-2, Math.abs(Axis3D.Z.dot(j2Axis))),
                         "Diff: " + Math.abs(qTarget[2] - (qPerturbed[2] + qCorrection[2])));
         }
         catch (AssertionFailedError e)
         {
            System.out.println("Iteration: " + i);
            System.out.println("qTarget: " + Arrays.toString(qTarget));
            System.out.println("qError : " + Arrays.toString(qError));
            System.out.println("qCorr  : " + Arrays.toString(qCorrection));
            System.out.println("qEst   : " + Arrays.toString(qEst));
            System.out.println("parent : " + parentIMU.getOrientationMeasurement().toStringAsYawPitchRoll());
            System.out.println("child  : " + childIMU.getOrientationMeasurement().toStringAsYawPitchRoll());
            System.out.println("primary   Jacobian: " + estimator.getPrimaryJacobian());
            System.out.println("preferred Jacobian: " + estimator.getPreferredJacobian());
            System.out.println("preferred Correction: " + estimator.getQPreferred());
            throw e;
         }
      }
   }

   @Test
   public void testWithPreferredConfigurationCorruptedKinematics()
   {
      Random random = new Random();//2344332);

      for (int i = 0; i < ITERATIONS; i++)
      {
         List<OneDoFJointBasics> joints = nextRevoluteChain(random, Axis3D.Z, Axis3D.Y, Axis3D.X);
         RigidBodyBasics base = joints.get(0).getPredecessor();
         RigidBodyBasics endEffector = joints.get(joints.size() - 1).getSuccessor();

         IMUSensor parentIMU = nextIMU(random, base, false);
         parentIMU.setOrientationMeasurement(new YawPitchRoll(0 * EuclidCoreRandomTools.nextDouble(random, Math.PI), 0, 0));
         IMUSensor childIMU = nextIMU(random, endEffector, true);
         IMUBasedJointPositionEstimator estimator = new IMUBasedJointPositionEstimator("IMUEstimator", parentIMU, childIMU, null);

         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
         base.updateFramesRecursively();

         ReferenceFrame childIMUFrame = childIMU.getMeasurementFrame();
         ReferenceFrame parentIMUFrame = parentIMU.getMeasurementFrame();
         Quaternion childIMUOrientation = new Quaternion(childIMUFrame.getTransformToDesiredFrame(parentIMUFrame).getRotation());
         childIMUOrientation.preMultiply(parentIMU.getOrientationMeasurement());
         // We introduce misalignment between the 2 IMUs
         //         childIMUOrientation.prependYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI));
         childIMU.setOrientationMeasurement(childIMUOrientation);

         double[] qTarget = joints.stream().mapToDouble(OneDoFJointReadOnly::getQ).toArray();
         double[] qError = {EuclidCoreRandomTools.nextDouble(random, 0.05),
                            EuclidCoreRandomTools.nextDouble(random, 0.05),
                            EuclidCoreRandomTools.nextDouble(random, 0.05)};
         double[] qPerturbed = new double[joints.size()];

         for (int j = 0; j < joints.size(); j++)
         {
            qPerturbed[j] = qTarget[j] + qError[j];
            joints.get(j).setQ(qPerturbed[j]);
         }
         base.updateFramesRecursively();
         estimator.setPreferredJointPositions(joint -> qTarget[joints.indexOf(joint)]);
         estimator.compute();

         double[] qCorrection = estimator.getQCorrection().data;
         double[] qEst = new double[qCorrection.length];
         for (int j = 0; j < joints.size(); j++)
         {
            qEst[j] = qPerturbed[j] + qCorrection[j];
         }

         try
         {
            assertEquals(qTarget[0], qPerturbed[0] + qCorrection[0], 1.0e-3, "Diff: " + Math.abs(qTarget[0] - (qPerturbed[0] + qCorrection[0])));
            assertEquals(qTarget[1], qPerturbed[1] + qCorrection[1], 1.0e-3, "Diff: " + Math.abs(qTarget[1] - (qPerturbed[1] + qCorrection[1])));

            // Before doing assertions on j2 we check that it's axis is not aligned with Z.
            // If it is aligned, it is selected out.
            Vector3D j2Axis = new Vector3D(joints.get(2).getJointAxis());
            joints.get(2).getFrameBeforeJoint().transformFromThisToDesiredFrame(parentIMUFrame, j2Axis);
            parentIMU.getOrientationMeasurement().transform(j2Axis);

            // TODO With the preferred configuration, I was expecting the error to be contained on the last joint regardless of the middle joint configuration
            assertEquals(qTarget[2],
                         qPerturbed[2] + qCorrection[2],
                         EuclidCoreTools.interpolate(1.0e-3, 2.0e-2, Math.abs(Axis3D.Z.dot(j2Axis))),
                         "Diff: " + Math.abs(qTarget[2] - (qPerturbed[2] + qCorrection[2])));
         }
         catch (AssertionFailedError e)
         {
            System.out.println("Iteration: " + i);
            System.out.println("qTarget: " + Arrays.toString(qTarget));
            System.out.println("qError : " + Arrays.toString(qError));
            System.out.println("qCorr  : " + Arrays.toString(qCorrection));
            System.out.println("qEst   : " + Arrays.toString(qEst));
            System.out.println("parent : " + parentIMU.getOrientationMeasurement().toStringAsYawPitchRoll());
            System.out.println("child  : " + childIMU.getOrientationMeasurement().toStringAsYawPitchRoll());
            System.out.println("primary   Jacobian: " + estimator.getPrimaryJacobian());
            System.out.println("preferred Jacobian: " + estimator.getPreferredJacobian());
            System.out.println("preferred Correction: " + estimator.getQPreferred());
            throw e;
         }
      }
   }

   @Test
   public void testConvergenceNoPreferredConfiguration()
   {
      Random random = new Random();//2344332);

      for (int i = 0; i < ITERATIONS; i++)
      {
         List<OneDoFJointBasics> joints = nextRevoluteChain(random, Axis3D.Z, Axis3D.Y, Axis3D.X);
         RigidBodyBasics base = joints.get(0).getPredecessor();
         RigidBodyBasics endEffector = joints.get(joints.size() - 1).getSuccessor();

         IMUSensor parentIMU = nextIMU(random, base, false);
         parentIMU.setOrientationMeasurement(new YawPitchRoll(EuclidCoreRandomTools.nextDouble(random, Math.PI), 0, 0));
         IMUSensor childIMU = nextIMU(random, endEffector, true);
         IMUBasedJointPositionEstimator estimator = new IMUBasedJointPositionEstimator("IMUEstimator", parentIMU, childIMU, null);

         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
         base.updateFramesRecursively();

         ReferenceFrame childIMUFrame = childIMU.getMeasurementFrame();
         ReferenceFrame parentIMUFrame = parentIMU.getMeasurementFrame();
         Quaternion childIMUOrientation = new Quaternion(childIMUFrame.getTransformToDesiredFrame(parentIMUFrame).getRotation());
         childIMUOrientation.preMultiply(parentIMU.getOrientationMeasurement());
         // We introduce misalignment between the 2 IMUs
         childIMUOrientation.prependYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI));
         childIMU.setOrientationMeasurement(childIMUOrientation);

         double[] qTarget = joints.stream().mapToDouble(OneDoFJointReadOnly::getQ).toArray();
         // Not disturbing the Z-axis joint for now.
         double[] qError = {0.0, EuclidCoreRandomTools.nextDouble(random, 0.05), EuclidCoreRandomTools.nextDouble(random, 0.05)};
         double[] qPerturbed = new double[joints.size()];

         for (int j = 0; j < joints.size(); j++)
         {
            qPerturbed[j] = qTarget[j] + qError[j];
            joints.get(j).setQ(qPerturbed[j]);
         }

         double[] qCorrection = estimator.getQCorrection().data;
         double[] qEst = java.util.Arrays.copyOf(qPerturbed, qPerturbed.length);

         for (int iteration = 0; iteration < 100; iteration++)
         {
            for (int j = 0; j < joints.size(); j++)
            {
               joints.get(j).setQ(qEst[j]);
            }
            base.updateFramesRecursively();
            estimator.compute();

            for (int j = 0; j < joints.size(); j++)
            {
               qEst[j] += qCorrection[j];
            }
         }

         try
         {
            assertEquals(qTarget[0], qEst[0], 1.0e-5, "Difference: " + Math.abs(qTarget[0] - qEst[0]));
            assertEquals(qTarget[1], qEst[1], 1.0e-5, "Difference: " + Math.abs(qTarget[1] - qEst[1]));
            assertEquals(qTarget[2], qEst[2], 1.0e-5, "Difference: " + Math.abs(qTarget[2] - qEst[2]));
         }
         catch (AssertionFailedError e)
         {
            System.out.println("Iteration: " + i);
            System.out.println("qTarget: " + Arrays.toString(qTarget));
            System.out.println("qError : " + Arrays.toString(qError));
            System.out.println("qCorr  : " + Arrays.toString(qCorrection));
            System.out.println("qEst   : " + Arrays.toString(qEst));
            System.out.println("parent : " + parentIMU.getOrientationMeasurement().toStringAsYawPitchRoll());
            System.out.println("child  : " + childIMU.getOrientationMeasurement().toStringAsYawPitchRoll());
            System.out.println("primary   Jacobian: " + estimator.getPrimaryJacobian());
            System.out.println("preferred Jacobian: " + estimator.getPreferredJacobian());
            System.out.println("preferred Correction: " + estimator.getQPreferred());
            throw e;
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Trying with 2 joints only
         List<OneDoFJointBasics> joints;
         if (random.nextBoolean())
            joints = nextRevoluteChain(random, Axis3D.Y, Axis3D.X);
         else
            joints = nextRevoluteChain(random, Axis3D.X, Axis3D.Y);
         RigidBodyBasics base = joints.get(0).getPredecessor();
         RigidBodyBasics endEffector = joints.get(joints.size() - 1).getSuccessor();

         IMUSensor parentIMU = nextIMU(random, base, false);
         parentIMU.setOrientationMeasurement(new YawPitchRoll(EuclidCoreRandomTools.nextDouble(random, Math.PI), 0, 0));
         IMUSensor childIMU = nextIMU(random, endEffector, true);
         IMUBasedJointPositionEstimator estimator = new IMUBasedJointPositionEstimator("IMUEstimator", parentIMU, childIMU, null);

         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
         base.updateFramesRecursively();

         ReferenceFrame childIMUFrame = childIMU.getMeasurementFrame();
         ReferenceFrame parentIMUFrame = parentIMU.getMeasurementFrame();
         Quaternion childIMUOrientation = new Quaternion(childIMUFrame.getTransformToDesiredFrame(parentIMUFrame).getRotation());
         childIMUOrientation.preMultiply(parentIMU.getOrientationMeasurement());
         // We introduce misalignment between the 2 IMUs
         childIMUOrientation.prependYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI));
         childIMU.setOrientationMeasurement(childIMUOrientation);

         double[] qTarget = joints.stream().mapToDouble(OneDoFJointReadOnly::getQ).toArray();
         double[] qError = {EuclidCoreRandomTools.nextDouble(random, 0.05), EuclidCoreRandomTools.nextDouble(random, 0.05)};
         double[] qPerturbed = new double[joints.size()];

         for (int j = 0; j < joints.size(); j++)
         {
            qPerturbed[j] = qTarget[j] + qError[j];
            joints.get(j).setQ(qPerturbed[j]);
         }

         double[] qCorrection = estimator.getQCorrection().data;
         double[] qEst = java.util.Arrays.copyOf(qPerturbed, qPerturbed.length);

         for (int iteration = 0; iteration < 100; iteration++)
         {
            for (int j = 0; j < joints.size(); j++)
            {
               qEst[j] = EuclidCoreTools.trimAngleMinusPiToPi(qEst[j]);
               joints.get(j).setQ(qEst[j]);
            }
            base.updateFramesRecursively();
            estimator.compute();

            for (int j = 0; j < joints.size(); j++)
            {
               qEst[j] += qCorrection[j];
            }
         }

         try
         {
            assertEquals(qTarget[0], qEst[0], 1.0e-5, "Difference: " + Math.abs(qTarget[0] - qEst[0]));
            assertEquals(qTarget[1], qEst[1], 1.0e-5, "Difference: " + Math.abs(qTarget[1] - qEst[1]));
         }
         catch (AssertionFailedError e)
         {
            System.out.println("Iteration: " + i);
            System.out.println("qTarget: " + Arrays.toString(qTarget));
            System.out.println("qError : " + Arrays.toString(qError));
            System.out.println("qCorr  : " + Arrays.toString(qCorrection));
            System.out.println("qEst   : " + Arrays.toString(qEst));
            System.out.println("parent : " + parentIMU.getOrientationMeasurement().toStringAsYawPitchRoll());
            System.out.println("child  : " + childIMU.getOrientationMeasurement().toStringAsYawPitchRoll());
            System.out.println("primary   Jacobian: " + estimator.getPrimaryJacobian());
            System.out.println("preferred Jacobian: " + estimator.getPreferredJacobian());
            System.out.println("preferred Correction: " + estimator.getQPreferred());
            throw e;
         }
      }
   }

   @Test
   public void testConvergenceWithPreferredConfiguration()
   {
      Random random = new Random();//2344332);

      for (int i = 0; i < ITERATIONS; i++)
      {
         List<OneDoFJointBasics> joints = nextRevoluteChain(random, Axis3D.Z, Axis3D.Y, Axis3D.X);
         RigidBodyBasics base = joints.get(0).getPredecessor();
         RigidBodyBasics endEffector = joints.get(joints.size() - 1).getSuccessor();

         IMUSensor parentIMU = nextIMU(random, base, true);
         IMUSensor childIMU = nextIMU(random, endEffector, true);
         IMUBasedJointPositionEstimator estimator = new IMUBasedJointPositionEstimator("IMUEstimator", parentIMU, childIMU, null);

         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
         base.updateFramesRecursively();

         ReferenceFrame childIMUFrame = childIMU.getMeasurementFrame();
         ReferenceFrame parentIMUFrame = parentIMU.getMeasurementFrame();
         Quaternion childIMUOrientation = new Quaternion(childIMUFrame.getTransformToDesiredFrame(parentIMUFrame).getRotation());
         childIMUOrientation.preMultiply(parentIMU.getOrientationMeasurement());
         // We introduce misalignment between the 2 IMUs
         childIMUOrientation.prependYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI));
         childIMU.setOrientationMeasurement(childIMUOrientation);

         double[] qTarget = joints.stream().mapToDouble(OneDoFJointReadOnly::getQ).toArray();
         double[] qError = {EuclidCoreRandomTools.nextDouble(random, 0.05),
                            EuclidCoreRandomTools.nextDouble(random, 0.05),
                            EuclidCoreRandomTools.nextDouble(random, 0.05)};
         double[] qPerturbed = new double[joints.size()];

         for (int j = 0; j < joints.size(); j++)
         {
            qPerturbed[j] = qTarget[j] + qError[j];
            joints.get(j).setQ(qPerturbed[j]);
         }

         double[] qCorrection = estimator.getQCorrection().data;
         double[] qEst = java.util.Arrays.copyOf(qPerturbed, qPerturbed.length);
         estimator.setPreferredJointPositions(joint -> qTarget[joints.indexOf(joint)]);

         for (int iteration = 0; iteration < 10; iteration++)
         {
            for (int j = 0; j < joints.size(); j++)
            {
               joints.get(j).setQ(qEst[j]);
            }
            base.updateFramesRecursively();
            estimator.compute();

            for (int j = 0; j < joints.size(); j++)
            {
               qEst[j] += qCorrection[j];
            }
         }

         double[] qDiff = new double[joints.size()];
         for (int j = 0; j < qDiff.length; j++)
         {
            qDiff[j] = qTarget[j] - qEst[j];
         }

         try
         {
            assertEquals(qTarget[0], qEst[0], 1.0e-6, "Difference: " + Math.abs(qTarget[0] - qEst[0]));
            assertEquals(qTarget[1], qEst[1], 1.0e-6, "Difference: " + Math.abs(qTarget[1] - qEst[1]));
            assertEquals(qTarget[2], qEst[2], 1.0e-6, "Difference: " + Math.abs(qTarget[2] - qEst[2]));
         }
         catch (AssertionFailedError e)
         {
            System.out.println("Iteration: " + i);
            System.out.println("qTarget: " + Arrays.toString(qTarget));
            System.out.println("qError : " + Arrays.toString(qError));
            System.out.println("qEst   : " + Arrays.toString(qEst));
            System.out.println("qDiff  : " + Arrays.toString(qDiff));
            System.out.println("parent : " + parentIMU.getOrientationMeasurement().toStringAsYawPitchRoll());
            System.out.println("child  : " + childIMU.getOrientationMeasurement().toStringAsYawPitchRoll());
            System.out.println("primary   Jacobian: " + estimator.getPrimaryJacobian());
            System.out.println("preferred Jacobian: " + estimator.getPreferredJacobian());
            System.out.println("preferred Correction: " + estimator.getQPreferred());
            throw e;
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Trying with 2 joints only
         List<OneDoFJointBasics> joints;
         if (random.nextBoolean())
            joints = nextRevoluteChain(random, Axis3D.Y, Axis3D.X);
         else
            joints = nextRevoluteChain(random, Axis3D.X, Axis3D.Y);
         RigidBodyBasics base = joints.get(0).getPredecessor();
         RigidBodyBasics endEffector = joints.get(joints.size() - 1).getSuccessor();

         IMUSensor parentIMU = nextIMU(random, base, true);
         IMUSensor childIMU = nextIMU(random, endEffector, true);
         IMUBasedJointPositionEstimator estimator = new IMUBasedJointPositionEstimator("IMUEstimator", parentIMU, childIMU, null);

         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
         base.updateFramesRecursively();

         ReferenceFrame childIMUFrame = childIMU.getMeasurementFrame();
         ReferenceFrame parentIMUFrame = parentIMU.getMeasurementFrame();
         Quaternion childIMUOrientation = new Quaternion(childIMUFrame.getTransformToDesiredFrame(parentIMUFrame).getRotation());
         childIMUOrientation.preMultiply(parentIMU.getOrientationMeasurement());
         // We introduce misalignment between the 2 IMUs
         childIMUOrientation.prependYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI));
         childIMU.setOrientationMeasurement(childIMUOrientation);

         double[] qTarget = joints.stream().mapToDouble(OneDoFJointReadOnly::getQ).toArray();
         double[] qError = {EuclidCoreRandomTools.nextDouble(random, 0.05), EuclidCoreRandomTools.nextDouble(random, 0.05)};
         double[] qPerturbed = new double[joints.size()];

         for (int j = 0; j < joints.size(); j++)
         {
            qPerturbed[j] = qTarget[j] + qError[j];
            joints.get(j).setQ(qPerturbed[j]);
         }

         double[] qCorrection = estimator.getQCorrection().data;
         double[] qEst = java.util.Arrays.copyOf(qPerturbed, qPerturbed.length);
         estimator.setPreferredJointPositions(joint -> qTarget[joints.indexOf(joint)]);

         for (int iteration = 0; iteration < 10; iteration++)
         {
            for (int j = 0; j < joints.size(); j++)
            {
               joints.get(j).setQ(qEst[j]);
            }
            base.updateFramesRecursively();
            estimator.compute();

            for (int j = 0; j < joints.size(); j++)
            {
               qEst[j] += qCorrection[j];
            }
         }

         double[] qDiff = new double[joints.size()];
         for (int j = 0; j < qDiff.length; j++)
         {
            qDiff[j] = qTarget[j] - qEst[j];
         }

         try
         {
            assertEquals(qTarget[0], qEst[0], 1.0e-6, "Difference: " + Math.abs(qTarget[0] - qEst[0]));
            assertEquals(qTarget[1], qEst[1], 1.0e-6, "Difference: " + Math.abs(qTarget[1] - qEst[1]));
         }
         catch (AssertionFailedError e)
         {
            System.out.println("Iteration: " + i);
            System.out.println("qTarget: " + Arrays.toString(qTarget));
            System.out.println("qError : " + Arrays.toString(qError));
            System.out.println("qEst   : " + Arrays.toString(qEst));
            System.out.println("qDiff  : " + Arrays.toString(qDiff));
            System.out.println("parent : " + parentIMU.getOrientationMeasurement().toStringAsYawPitchRoll());
            System.out.println("child  : " + childIMU.getOrientationMeasurement().toStringAsYawPitchRoll());
            System.out.println("primary   Jacobian: " + estimator.getPrimaryJacobian());
            System.out.println("preferred Jacobian: " + estimator.getPreferredJacobian());
            System.out.println("preferred Correction: " + estimator.getQPreferred());
            throw e;
         }
      }
   }

   private static IMUSensor nextIMU(Random random, RigidBodyBasics body, boolean randomizeTransform)
   {
      RigidBodyTransform transform;
      if (randomizeTransform)
         transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      else
         transform = new RigidBodyTransform();
      IMUDefinition def = new IMUDefinition(body.getName() + "IMU", body, transform);
      IMUSensor imuSensor = new IMUSensor(def, null);
      imuSensor.setOrientationMeasurement(EuclidCoreRandomTools.nextQuaternion(random));
      return imuSensor;
   }

   private static List<OneDoFJointBasics> nextRevoluteChain(Random random, Vector3DReadOnly... axes)
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      RigidBody rootBody = new RigidBody("rootBody", worldFrame);
      RigidBodyBasics predecessor = rootBody;

      List<OneDoFJointBasics> revoluteJoints = new ArrayList<>();

      for (int i = 0; i < axes.length; i++)
      {
         RigidBodyTransform transformToParent = new RigidBodyTransform(new Quaternion(), EuclidCoreRandomTools.nextVector3D(random));
         RevoluteJoint joint = new RevoluteJoint("joint" + i, predecessor, transformToParent, axes[i]);
         revoluteJoints.add(joint);
         predecessor = MultiBodySystemRandomTools.nextRigidBody(random, "body" + i, joint);
      }

      return revoluteJoints;
   }
}
