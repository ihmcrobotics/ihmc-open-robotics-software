package us.ihmc.perception.vio;

import controller_msgs.RobotConfigurationData;
import ihmc_common_msgs.StampedPosePacket;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullRobotModelTestTools.RandomFullHumanoidRobotModel;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationDataFactory;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.concurrent.atomic.AtomicLong;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Runs the provider the way a simulation does: without ROS 2, on an injected clock that starts near zero, with converted poses
 * going to a listener.
 */
class VIOPelvisPoseProviderTest
{
   private static final long SECOND = 1_000_000_000L;
   private static final long MILLISECOND = 1_000_000L;
   private static final long SEED = 4L;

   private final AtomicLong clock = new AtomicLong(5 * SECOND);
   private final List<StampedPosePacket> pelvisPoses = new ArrayList<>();
   /** Same seed as the provider's model, so the two have the same kinematics and joint names. */
   private final RandomFullHumanoidRobotModel controllerModel = new RandomFullHumanoidRobotModel(new Random(SEED));
   private final HumanoidReferenceFrames controllerFrames = new HumanoidReferenceFrames(controllerModel);
   private final VIOPelvisPoseProvider provider = new VIOPelvisPoseProvider("test",
                                                                            new RandomFullHumanoidRobotModel(new Random(SEED)),
                                                                            null,
                                                                            HumanoidReferenceFrames::getHeadFrame,
                                                                            null,
                                                                            clock::get,
                                                                            pelvisPoses::add);

   VIOPelvisPoseProviderTest()
   {
      controllerModel.getRootJoint().getJointPose().getPosition().set(1.0, 2.0, 0.9);
      controllerModel.getRootJoint().getJointPose().getOrientation().setYawPitchRoll(0.3, 0.0, 0.0);
      controllerModel.updateFrames();
      controllerFrames.updateFrames();
   }

   @Test
   void convertsPosesOnTheInjectedClockAndHandsThemToTheListener()
   {
      startSession();
      RigidBodyTransform firstCameraPose = pose(0.3, -0.1, 0.0, 0.2);
      RigidBodyTransform secondCameraPose = pose(0.4, -0.1, 0.0, 0.25);

      addRobotState(5 * SECOND);
      addCameraPose(5 * SECOND + 10 * MILLISECOND, firstCameraPose);
      assertTrue(pelvisPoses.isEmpty(), "Waits for a robot state after the exposure");
      addRobotState(5 * SECOND + 20 * MILLISECOND);
      addCameraPose(5 * SECOND + 30 * MILLISECOND, secondCameraPose);
      addRobotState(5 * SECOND + 40 * MILLISECOND);

      assertEquals(2, pelvisPoses.size());
      String sessionId = pelvisPoses.get(0).getFrameIdAsString();
      assertFalse(sessionId.isEmpty() || sessionId.equals("waiting"), sessionId);
      assertEquals(sessionId, pelvisPoses.get(1).getFrameIdAsString());
      assertEquals(5 * SECOND + 10 * MILLISECOND, pelvisPoses.get(0).getTimestamp());
      assertEquals(5 * SECOND + 30 * MILLISECOND, pelvisPoses.get(1).getTimestamp());

      // The first pose anchors the VIO world: it's the controller's pelvis pose.
      RigidBodyTransform controllerPelvisPose = new RigidBodyTransform(controllerModel.getRootJoint().getJointPose());
      EuclidCoreTestTools.assertEquals(controllerPelvisPose, toTransform(pelvisPoses.get(0)), 1.0e-6);

      // Later poses move the pelvis by the camera's motion in the anchored world.
      RigidBodyTransform cameraPose = controllerFrames.getHeadFrame().getTransformToWorldFrame();
      RigidBodyTransform expected = new RigidBodyTransform(cameraPose);
      expected.multiplyInvertOther(firstCameraPose);
      expected.multiply(secondCameraPose);
      expected.multiplyInvertOther(cameraPose);
      expected.multiply(controllerPelvisPose);
      EuclidCoreTestTools.assertEquals(expected, toTransform(pelvisPoses.get(1)), 1.0e-6);
   }

   @Test
   void rejectsInputStaleOnTheInjectedClockAndStartsANewSessionOnRequest()
   {
      startSession();
      addRobotState(5 * SECOND);
      addCameraPose(5 * SECOND + 10 * MILLISECOND, pose(0.0, 0.0, 0.0, 0.0));
      clock.set(6 * SECOND);
      addRobotState(5 * SECOND + 20 * MILLISECOND);
      assertTrue(pelvisPoses.isEmpty(), "A robot state 980 ms behind the injected clock is stale");

      provider.requestTrackingRestart();
      startSession();
      addRobotState(6 * SECOND);
      addCameraPose(6 * SECOND + 10 * MILLISECOND, pose(0.0, 0.0, 0.0, 0.0));
      addRobotState(6 * SECOND + 20 * MILLISECOND);
      assertEquals(1, pelvisPoses.size());
      assertEquals(6 * SECOND + 10 * MILLISECOND, pelvisPoses.get(0).getTimestamp());
   }

   /** Does what the camera's grab thread does: takes the pending restart and reports it done, at the clock's time. */
   private void startSession()
   {
      long token = provider.takePendingTrackingRestartRequest();
      assertNotEquals(0, token);
      provider.finishTrackingRestart(token, true);
   }

   private void addRobotState(long wallTime)
   {
      clock.set(Math.max(clock.get(), wallTime));
      RobotConfigurationData robotState = RobotConfigurationDataFactory.create(controllerModel.getOneDoFJoints(),
                                                                               controllerModel.getForceSensorDefinitions(),
                                                                               controllerModel.getIMUDefinitions());
      RobotConfigurationDataFactory.packJointState(robotState, controllerModel.getOneDoFJoints());
      robotState.getRootPosition().set(controllerModel.getRootJoint().getJointPose().getPosition());
      robotState.getRootOrientation().set(controllerModel.getRootJoint().getJointPose().getOrientation());
      robotState.setWallTime(wallTime);
      provider.addRobotState(robotState);
   }

   private void addCameraPose(long exposureTime, RigidBodyTransformReadOnly cameraPoseInVIOWorld)
   {
      clock.set(Math.max(clock.get(), exposureTime));
      provider.addCameraTrackingPose(exposureTime, cameraPoseInVIOWorld, new RigidBodyTransform());
   }

   private static RigidBodyTransform pose(double x, double y, double z, double yaw)
   {
      RigidBodyTransform pose = new RigidBodyTransform();
      pose.getTranslation().set(x, y, z);
      pose.getRotation().setToYawOrientation(yaw);
      return pose;
   }

   private static RigidBodyTransform toTransform(StampedPosePacket packet)
   {
      return new RigidBodyTransform(packet.getPose().getPose().getOrientation(), packet.getPose().getPose().getPosition());
   }
}
