package us.ihmc.simulationconstructionset.simulatedSensors;

import static org.junit.Assert.*;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.RigidBodyInertia;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.sensors.RawIMUSensorsInterface;

public class SimulatedIMURawSensorReaderTest
{
   private final RawSensors rawSensors = new RawSensors();
   private final TestingRobotModel fullRobotModel = new TestingRobotModel();
   private final RigidBody rigidBody = fullRobotModel.getBodyLink();
   private final ReferenceFrame bodyFrame = fullRobotModel.getBodyFrame();

   private final RotationMatrix actualIMUOrientation = new RotationMatrix();
   private final Vector3D actualLinearAcceleration = new Vector3D();
   private final Vector3D actualAngularVelocity = new Vector3D();

   private final AxisAngle randomBodyAxisAngle = new AxisAngle();
   private final RigidBodyTransform randomTransformBodyToWorld = new RigidBodyTransform();
   private final FrameVector randomLinearVelocity = new FrameVector((ReferenceFrame) null);
   private final FrameVector randomAngularVelocity = new FrameVector((ReferenceFrame) null);
   private final FrameVector randomLinearAcceleration = new FrameVector((ReferenceFrame) null);
   private final FrameVector randomAngularAcceleration = new FrameVector((ReferenceFrame) null);

   private final RotationMatrix expectedIMUOrientation = new RotationMatrix();
   private final Vector3D expectedAngularVelocityInIMUFrame = new Vector3D();
   private final Vector3D expectedLinearAccelerationOfIMUInIMUFrame = new Vector3D();

   private final FrameVector jointToIMUOffset = new FrameVector(bodyFrame, 2.0 * (Math.random() - 0.5), 2.0 * (Math.random() - 0.5),
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

   @Before
   public void setUp() throws Exception
   {
      transformIMUToJoint.setRotation(jointToIMURotation);
      transformIMUToJoint.setTranslation(jointToIMUOffset.getVectorCopy());
      transformJointToIMU.setAndInvert(transformIMUToJoint);

      imuFrame = fullRobotModel.createOffsetFrame(fullRobotModel.getBodyLink().getParentJoint(), transformIMUToJoint, "imuFrame");

      Vector3D linearAcceleration = new Vector3D(0.0, 0.0, GRAVITY);
      Vector3D angularAcceleration = new Vector3D();
      ReferenceFrame rootBodyFrame = fullRobotModel.getElevatorFrame();
      SpatialAccelerationVector rootAcceleration = new SpatialAccelerationVector(rootBodyFrame, ReferenceFrame.getWorldFrame(), rootBodyFrame,
                                                                                 linearAcceleration, angularAcceleration);
      simulatedIMURawSensorReader = new PerfectSimulatedIMURawSensorReader(rawSensors, IMU_INDEX, rigidBody, imuFrame, fullRobotModel.getElevator(),
                                                                           rootAcceleration);

      simulatedIMURawSensorReader.initialize();
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.2)
   @Test(timeout = 300000)
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
      randomTransformBodyToWorld.getRotation(randomTransformBodyToWorldMatrix);
      transformIMUToJoint.getRotation(transformIMUToJointMatrix);

      expectedIMUOrientation.set(randomTransformBodyToWorldMatrix);
      expectedIMUOrientation.multiply(transformIMUToJointMatrix);
   }

   private void generateExpectedAngularVelocity()
   {
      expectedAngularVelocityInIMUFrame.set(randomAngularVelocity.getVectorCopy()); // in joint/body frame
      transformJointToIMU.transform(expectedAngularVelocityInIMUFrame);
   }

   private void generateExpectedLinearAcceleration()
   {
      FrameVector centerAppliedAccelerationPart = new FrameVector(randomLinearAcceleration);

      FrameVector centerCoriolisAccelerationPart = new FrameVector(bodyFrame);
      centerCoriolisAccelerationPart.cross(randomAngularVelocity, randomLinearVelocity);

      FrameVector gravitationalAccelerationPart = new FrameVector(fullRobotModel.getWorldFrame());
      gravitationalAccelerationPart.setZ(GRAVITY);
      gravitationalAccelerationPart.changeFrame(bodyFrame);

      FrameVector centripedalAccelerationPart = new FrameVector(bodyFrame);
      centripedalAccelerationPart.cross(randomAngularVelocity, jointToIMUOffset);
      centripedalAccelerationPart.cross(randomAngularVelocity, centripedalAccelerationPart);

      FrameVector angularAccelerationPart = new FrameVector(bodyFrame);
      angularAccelerationPart.cross(randomAngularAcceleration, jointToIMUOffset);

      expectedLinearAccelerationOfIMUInIMUFrame.set(centerAppliedAccelerationPart.getVectorCopy());
      expectedLinearAccelerationOfIMUInIMUFrame.add(centerCoriolisAccelerationPart.getVectorCopy());
      expectedLinearAccelerationOfIMUInIMUFrame.add(gravitationalAccelerationPart.getVectorCopy());
      expectedLinearAccelerationOfIMUInIMUFrame.add(centripedalAccelerationPart.getVectorCopy());
      expectedLinearAccelerationOfIMUInIMUFrame.add(angularAccelerationPart.getVectorCopy());

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
      private final RigidBody elevator;
      private final RigidBody body;

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
         elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new RigidBodyTransform());

         elevator = new RigidBody("elevator", elevatorFrame);

         rootJoint = new SixDoFJoint("rootJoint", elevator, elevatorFrame);

         body = addRigidBody("body", rootJoint, Ixx, Iyy, Izz, mass, comOffset);

         bodyFrame = rootJoint.getFrameAfterJoint();
      }

      public void update(RigidBodyTransform transformBodyToWorld, FrameVector linearVelocity, FrameVector angularVelocity, FrameVector linearAcceleration,
                         FrameVector angularAcceleration)
      {
         // Update Body Pose
         rootJoint.setPositionAndRotation(transformBodyToWorld); // TODO correct?
         updateFrames();

         // Update Body Velocity
         Twist bodyTwist = new Twist(bodyFrame, elevatorFrame, bodyFrame, linearVelocity.getVector(), angularVelocity.getVector());
         rootJoint.setJointTwist(bodyTwist);

         // Update Body Acceleration
         SpatialAccelerationVector accelerationOfChestWithRespectToWorld = new SpatialAccelerationVector(bodyFrame, worldFrame, bodyFrame,
                                                                                                         linearAcceleration.getVector(),
                                                                                                         angularAcceleration.getVector());
         accelerationOfChestWithRespectToWorld.changeBaseFrameNoRelativeAcceleration(getElevatorFrame());
         rootJoint.setAcceleration(accelerationOfChestWithRespectToWorld);

         updateFrames();
      }

      public void updateFrames()
      {
         elevator.updateFramesRecursively();
      }

      public RigidBody getBodyLink()
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

      public RigidBody getElevator()
      {
         return elevator;
      }

      public FrameVector getReferenceFrameTransInWorldFrame(ReferenceFrame frame)
      {
         Vector3D trans = new Vector3D();
         frame.getTransformToDesiredFrame(worldFrame).getTranslation(trans);
         FrameVector ret = new FrameVector(worldFrame, trans);
         return ret;
      }

      private RigidBody addRigidBody(String name, InverseDynamicsJoint parentJoint, double Ixx, double Iyy, double Izz, double mass,
                                     Vector3D centerOfMassOffset)
      {
         String comFrameName = name + "CoM";
         ReferenceFrame comFrame = createOffsetFrame(parentJoint, centerOfMassOffset, comFrameName);
         RigidBodyInertia inertia = new RigidBodyInertia(comFrame, Ixx, Iyy, Izz, mass);
         RigidBody ret = new RigidBody(name, inertia, parentJoint);
         return ret;
      }

      private ReferenceFrame createOffsetFrame(InverseDynamicsJoint previousJoint, Vector3D offset, String frameName)
      {
         ReferenceFrame parentFrame = previousJoint.getFrameAfterJoint();
         RigidBodyTransform transformToParent = new RigidBodyTransform();
         transformToParent.setTranslationAndIdentityRotation(offset);
         ReferenceFrame beforeJointFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent(frameName, parentFrame, transformToParent);
         return beforeJointFrame;
      }

      private ReferenceFrame createOffsetFrame(InverseDynamicsJoint previousJoint, RigidBodyTransform transformToParent, String frameName)
      {
         ReferenceFrame parentFrame = previousJoint.getFrameAfterJoint();
         ReferenceFrame beforeJointFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent(frameName, parentFrame, transformToParent);
         return beforeJointFrame;
      }
   }
}