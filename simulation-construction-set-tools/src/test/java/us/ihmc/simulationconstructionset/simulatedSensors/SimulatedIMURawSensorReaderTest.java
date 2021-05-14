package us.ihmc.simulationconstructionset.simulatedSensors;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotics.sensors.RawIMUSensorsInterface;
import us.ihmc.simulationConstructionSetTools.simulatedSensors.PerfectSimulatedIMURawSensorReader;
import us.ihmc.simulationConstructionSetTools.simulatedSensors.SimulatedIMURawSensorReader;

public class SimulatedIMURawSensorReaderTest
{
   private final RawSensors rawSensors = new RawSensors();
   private final TestingRobotModel fullRobotModel = new TestingRobotModel();
   private final RigidBodyBasics rigidBody = fullRobotModel.getBodyLink();
   private final ReferenceFrame bodyFrame = fullRobotModel.getBodyFrame();

   private final RotationMatrix actualIMUOrientation = new RotationMatrix();
   private final Vector3D actualLinearAcceleration = new Vector3D();
   private final Vector3D actualAngularVelocity = new Vector3D();

   private final AxisAngle randomBodyAxisAngle = new AxisAngle();
   private final RigidBodyTransform randomTransformBodyToWorld = new RigidBodyTransform();
   private final FrameVector3D randomLinearVelocity = new FrameVector3D((ReferenceFrame) null);
   private final FrameVector3D randomAngularVelocity = new FrameVector3D((ReferenceFrame) null);
   private final FrameVector3D randomLinearAcceleration = new FrameVector3D((ReferenceFrame) null);
   private final FrameVector3D randomAngularAcceleration = new FrameVector3D((ReferenceFrame) null);

   private final RotationMatrix expectedIMUOrientation = new RotationMatrix();
   private final Vector3D expectedAngularVelocityInIMUFrame = new Vector3D();
   private final Vector3D expectedLinearAccelerationOfIMUInIMUFrame = new Vector3D();

   private final FrameVector3D jointToIMUOffset = new FrameVector3D(bodyFrame, 2.0 * (Math.random() - 0.5), 2.0 * (Math.random() - 0.5),
                                                                2.0 * (Math.random() - 0.5));
   //private final FrameVector jointToIMUOffset = new FrameVector(bodyFrame, 1.0, 0.0, 0.0); // for debugging

   private final AxisAngle jointToIMURotation = new AxisAngle(2.0 * (Math.random() - 0.5), 2.0 * (Math.random() - 0.5), 2.0 * (Math.random() - 0.5),
                                                              Math.random() * 2.0 * Math.PI);
   //private final AxisAngle4d jointToIMURotation = new AxisAngle4d(0.0, 0.0, 0.0, 0.0); // for debugging

   private final RigidBodyTransform transformIMUToJoint = new RigidBodyTransform();
   private final RigidBodyTransform transformJointToIMU = new RigidBodyTransform();
   private ReferenceFrame imuFrame;

   public static final double GRAVITY = (2.0 * Math.random() - 1) * 15.0; // random gravity between -15 and +15 m/s^2
   public static final int IMU_INDEX = (int) (10.0 * Math.random()); // random imu index between 0 and 10

   private SimulatedIMURawSensorReader simulatedIMURawSensorReader;

   @BeforeEach
   public void setUp() throws Exception
   {
      transformIMUToJoint.getRotation().set(jointToIMURotation);
      transformIMUToJoint.getTranslation().set(jointToIMUOffset);
      transformJointToIMU.setAndInvert(transformIMUToJoint);

      imuFrame = fullRobotModel.createOffsetFrame(fullRobotModel.getBodyLink().getParentJoint(), transformIMUToJoint, "imuFrame");

      Vector3D linearAcceleration = new Vector3D(0.0, 0.0, GRAVITY);
      Vector3D angularAcceleration = new Vector3D();
      ReferenceFrame rootBodyFrame = fullRobotModel.getElevatorFrame();
      SpatialAcceleration rootAcceleration = new SpatialAcceleration(rootBodyFrame, ReferenceFrame.getWorldFrame(), rootBodyFrame,
                                                                                 angularAcceleration, linearAcceleration);
      simulatedIMURawSensorReader = new PerfectSimulatedIMURawSensorReader(rawSensors, IMU_INDEX, rigidBody, imuFrame, fullRobotModel.getElevator(),
                                                                           rootAcceleration);

      simulatedIMURawSensorReader.initialize();
   }

   @Test
   public void testRead()
   {
      for (int i = 0; i < 10000; i++)
      {
         generateAppliedOrientation();
         generateAppliedVelocity();
         generateAppliedAcceleration();

         fullRobotModel.update(randomTransformBodyToWorld, randomLinearVelocity, randomAngularVelocity, randomLinearAcceleration, randomAngularAcceleration);
         simulatedIMURawSensorReader.read();

         rawSensors.getOrientation(actualIMUOrientation, IMU_INDEX);
         rawSensors.getAngularVelocity(actualAngularVelocity, IMU_INDEX);
         rawSensors.getAcceleration(actualLinearAcceleration, IMU_INDEX);

         generateExpectedOrientation();
         generateExpectedAngularVelocity();
         generateExpectedLinearAcceleration();

         assertEqualsRotationMatrix(expectedIMUOrientation, actualIMUOrientation, 1e-3);
         assertEqualsVector(expectedAngularVelocityInIMUFrame, actualAngularVelocity, 1e-3);
         assertEqualsVector(expectedLinearAccelerationOfIMUInIMUFrame, actualLinearAcceleration, 1e-3);
      }
   }

   private void generateAppliedOrientation()
   {
      randomBodyAxisAngle.set(2.0 * (Math.random() - 0.5), 2.0 * (Math.random() - 0.5), 2.0 * (Math.random() - 0.5), Math.random() * 2.0 * Math.PI);
      //randomBodyAxisAngle.set(0.0, 0.0, 0.0, 0.0); // for debugging

      randomTransformBodyToWorld.setRotationAndZeroTranslation(randomBodyAxisAngle);
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setRotationAndZeroTranslation(randomBodyAxisAngle);
   }

   private void generateAppliedVelocity()
   {
      randomLinearVelocity.setIncludingFrame(bodyFrame, Math.random() - 0.5, Math.random() - 0.5, Math.random() - 0.5);
      randomLinearVelocity.scale(10);
      //randomLinearVelocity.set(0.0, 0.0, 0.0);  // for debugging

      randomAngularVelocity.setIncludingFrame(bodyFrame, Math.random() - 0.5, Math.random() - 0.5, Math.random() - 0.5);
      randomAngularVelocity.scale(10);
      //randomAngularVelocity.set(0.0, 0.0, 1.0);  // for debugging
   }

   private void generateAppliedAcceleration()
   {
      randomLinearAcceleration.setIncludingFrame(bodyFrame, Math.random() - 0.5, Math.random() - 0.5, Math.random() - 0.5);
      randomLinearAcceleration.scale(40);
      //randomLinearAcceleration.set(0.0, 0.0, 0.0); // for debugging

      randomAngularAcceleration.setIncludingFrame(bodyFrame, Math.random() - 0.5, Math.random() - 0.5, Math.random() - 0.5);
      randomAngularAcceleration.scale(20);
      //randomAngularAcceleration.set(0.0, 0.0, 0.0); // for debugging
   }

   private void generateExpectedOrientation()
   {
      RotationMatrix randomTransformBodyToWorldMatrix = new RotationMatrix();
      RotationMatrix transformIMUToJointMatrix = new RotationMatrix();
      randomTransformBodyToWorldMatrix.set(randomTransformBodyToWorld.getRotation());
      transformIMUToJointMatrix.set(transformIMUToJoint.getRotation());

      expectedIMUOrientation.set(randomTransformBodyToWorldMatrix);
      expectedIMUOrientation.multiply(transformIMUToJointMatrix);
   }

   private void generateExpectedAngularVelocity()
   {
      expectedAngularVelocityInIMUFrame.set(randomAngularVelocity); // in joint/body frame
      transformJointToIMU.transform(expectedAngularVelocityInIMUFrame);
   }

   private void generateExpectedLinearAcceleration()
   {
      FrameVector3D centerAppliedAccelerationPart = new FrameVector3D(randomLinearAcceleration);

      FrameVector3D centerCoriolisAccelerationPart = new FrameVector3D(bodyFrame);
      centerCoriolisAccelerationPart.cross(randomAngularVelocity, randomLinearVelocity);

      FrameVector3D gravitationalAccelerationPart = new FrameVector3D(fullRobotModel.getWorldFrame());
      gravitationalAccelerationPart.setZ(GRAVITY);
      gravitationalAccelerationPart.changeFrame(bodyFrame);

      FrameVector3D centripedalAccelerationPart = new FrameVector3D(bodyFrame);
      centripedalAccelerationPart.cross(randomAngularVelocity, jointToIMUOffset);
      centripedalAccelerationPart.cross(randomAngularVelocity, centripedalAccelerationPart);

      FrameVector3D angularAccelerationPart = new FrameVector3D(bodyFrame);
      angularAccelerationPart.cross(randomAngularAcceleration, jointToIMUOffset);

      expectedLinearAccelerationOfIMUInIMUFrame.set(centerAppliedAccelerationPart);
      expectedLinearAccelerationOfIMUInIMUFrame.add(centerCoriolisAccelerationPart);
      expectedLinearAccelerationOfIMUInIMUFrame.add(gravitationalAccelerationPart);
      expectedLinearAccelerationOfIMUInIMUFrame.add(centripedalAccelerationPart);
      expectedLinearAccelerationOfIMUInIMUFrame.add(angularAccelerationPart);

      transformJointToIMU.transform(expectedLinearAccelerationOfIMUInIMUFrame);
   }

   private static void assertEqualsVector(Vector3D expected, Vector3D actual, double delta)
   {
      assertEquals(expected.getX(), actual.getX(), delta);
      assertEquals(expected.getY(), actual.getY(), delta);
      assertEquals(expected.getZ(), actual.getZ(), delta);
   }

   private static void assertEqualsRotationMatrix(RotationMatrix expected, RotationMatrix actual, double delta)
   {
      RotationMatrix differenceMatrix = new RotationMatrix();
      differenceMatrix.setAndTranspose(expected);
      differenceMatrix.multiply(actual);

      AxisAngle differenceAxisAngle = new AxisAngle(differenceMatrix);
      double differenceAngle = differenceAxisAngle.getAngle();

      assertEquals(0.0, differenceAngle, delta);
   }

   private static class RawSensors implements RawIMUSensorsInterface
   {
      private double r_imu_m00 = 0.0;
      private double r_imu_m01 = 0.0;
      private double r_imu_m02 = 0.0;

      private double r_imu_m10 = 0.0;
      private double r_imu_m11 = 0.0;
      private double r_imu_m12 = 0.0;

      private double r_imu_m20 = 0.0;
      private double r_imu_m21 = 0.0;
      private double r_imu_m22 = 0.0;

      private double r_imu_accel_x = 0.0;
      private double r_imu_accel_y = 0.0;
      private double r_imu_accel_z = 0.0;

      private double r_imu_gyro_x = 0.0;
      private double r_imu_gyro_y = 0.0;
      private double r_imu_gyro_z = 0.0;

      private double r_imu_compass_x = 0.0;
      private double r_imu_compass_y = 0.0;
      private double r_imu_compass_z = 0.0;

      @Override
      public void setOrientation(RotationMatrix orientation, int imuIndex)
      {
         r_imu_m00 = orientation.getM00();
         r_imu_m01 = orientation.getM01();
         r_imu_m02 = orientation.getM02();

         r_imu_m10 = orientation.getM10();
         r_imu_m11 = orientation.getM11();
         r_imu_m12 = orientation.getM12();

         r_imu_m20 = orientation.getM20();
         r_imu_m21 = orientation.getM21();
         r_imu_m22 = orientation.getM22();
      }

      @Override
      public void setAcceleration(Vector3D acceleration, int imuIndex)
      {
         r_imu_accel_x = acceleration.getX();
         r_imu_accel_y = acceleration.getY();
         r_imu_accel_z = acceleration.getZ();
      }

      @Override
      public void setAngularVelocity(Vector3D gyroscope, int imuIndex)
      {
         r_imu_gyro_x = gyroscope.getX();
         r_imu_gyro_y = gyroscope.getY();
         r_imu_gyro_z = gyroscope.getZ();
      }

      @Override
      public void setCompass(Vector3D compass, int imuIndex)
      {
         r_imu_compass_x = compass.getX();
         r_imu_compass_y = compass.getY();
         r_imu_compass_z = compass.getZ();
      }

      @Override
      public void getOrientation(RotationMatrix orientationToPack, int imuIndex)
      {
         orientationToPack.set(r_imu_m00, r_imu_m01, r_imu_m02, r_imu_m10, r_imu_m11, r_imu_m12, r_imu_m20, r_imu_m21, r_imu_m22);
      }

      @Override
      public void getAcceleration(Vector3D accelerationToPack, int imuIndex)
      {
         accelerationToPack.set(r_imu_accel_x, r_imu_accel_y, r_imu_accel_z);
      }

      @Override
      public void getAngularVelocity(Vector3D angularVelocityToPack, int imuIndex)
      {
         angularVelocityToPack.set(r_imu_gyro_x, r_imu_gyro_y, r_imu_gyro_z);
      }

      @Override
      public void getCompass(Vector3D compassToPack, int imuIndex)
      {
         compassToPack.set(r_imu_compass_x, r_imu_compass_y, r_imu_compass_z);
      }
   }

   private static class TestingRobotModel
   {
      private final RigidBodyBasics elevator;
      private final RigidBodyBasics body;

      private final SixDoFJoint rootJoint;
      private final ReferenceFrame worldFrame;

      private final double Ixx = Math.random();
      private final double Iyy = Math.random();
      private final double Izz = Math.random();
      private final double mass = Math.random();
      private Vector3D comOffset = new Vector3D(Math.random() - 0.5, Math.random() - 0.5, Math.random() - 0.5);

      private final ReferenceFrame elevatorFrame;
      private final ReferenceFrame bodyFrame;

      public TestingRobotModel()
      {
         worldFrame = ReferenceFrame.getWorldFrame();

         elevator = new RigidBody("elevator", worldFrame);
         elevatorFrame = elevator.getBodyFixedFrame();

         rootJoint = new SixDoFJoint("rootJoint", elevator);

         body = new RigidBody("body", rootJoint, Ixx, Iyy, Izz, mass, comOffset);

         bodyFrame = rootJoint.getFrameAfterJoint();
      }

      public void update(RigidBodyTransform transformBodyToWorld, FrameVector3D linearVelocity, FrameVector3D angularVelocity, FrameVector3D linearAcceleration,
                         FrameVector3D angularAcceleration)
      {
         // Update Body Pose
         rootJoint.setJointConfiguration(transformBodyToWorld); // TODO correct?
         updateFrames();

         // Update Body Velocity
         Twist bodyTwist = new Twist(bodyFrame, elevatorFrame, bodyFrame, angularVelocity, linearVelocity);
         rootJoint.setJointTwist(bodyTwist);

         // Update Body Acceleration
         SpatialAcceleration accelerationOfChestWithRespectToWorld = new SpatialAcceleration(bodyFrame, worldFrame, bodyFrame,
                                                                                                         angularAcceleration,
                                                                                                         linearAcceleration);
         accelerationOfChestWithRespectToWorld.setBaseFrame(getElevatorFrame());
         rootJoint.setJointAcceleration(accelerationOfChestWithRespectToWorld);

         updateFrames();
      }

      public void updateFrames()
      {
         elevator.updateFramesRecursively();
      }

      public RigidBodyBasics getBodyLink()
      {
         return body;
      }

      public ReferenceFrame getWorldFrame()
      {
         return worldFrame;
      }

      public ReferenceFrame getBodyFrame()
      {
         return bodyFrame;
      }

      public ReferenceFrame getElevatorFrame()
      {
         return elevator.getBodyFixedFrame();
      }

      public RigidBodyBasics getElevator()
      {
         return elevator;
      }

      public FrameVector3D getReferenceFrameTransInWorldFrame(ReferenceFrame frame)
      {
         Vector3D trans = new Vector3D();
         trans.set(frame.getTransformToDesiredFrame(worldFrame).getTranslation());
         FrameVector3D ret = new FrameVector3D(worldFrame, trans);
         return ret;
      }

      private ReferenceFrame createOffsetFrame(JointBasics previousJoint, RigidBodyTransform transformToParent, String frameName)
      {
         ReferenceFrame parentFrame = previousJoint.getFrameAfterJoint();
         ReferenceFrame beforeJointFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(frameName, parentFrame, transformToParent);
         return beforeJointFrame;
      }
   }
}