package us.ihmc.humanoidBehaviors.behaviors.debug;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.ChestTrajectoryMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.OneDoFJointTrajectoryMessage;
import controller_msgs.msg.dds.SO3TrajectoryMessage;
import controller_msgs.msg.dds.SO3TrajectoryPointMessage;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.time.YoStopwatch;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.yoVariables.variable.YoDouble;

public class TestGarbageGenerationBehavior extends AbstractBehavior
{
   private static final double sendInterval = 0.5;
   private static final int trajectoryPoints = 50;
   private static final int steps = 4;

   private final HumanoidReferenceFrames referenceFrames;
   private final YoStopwatch timer;

   private final IHMCROS2Publisher<ArmTrajectoryMessage> armPublisher;
   private final IHMCROS2Publisher<ChestTrajectoryMessage> chestPublisher;
   private final IHMCROS2Publisher<FootstepDataListMessage> footstepPublisher;

   public TestGarbageGenerationBehavior(String robotName, Ros2Node ros2Node, HumanoidReferenceFrames referenceFrames, YoDouble yoTime)
   {
      super(robotName, ros2Node);
      this.referenceFrames = referenceFrames;
      timer = new YoStopwatch(yoTime);
      armPublisher = createPublisherForController(ArmTrajectoryMessage.class);
      chestPublisher = createPublisherForController(ChestTrajectoryMessage.class);
      footstepPublisher = createPublisherForController(FootstepDataListMessage.class);
   }

   @Override
   public void doControl()
   {
      if (timer.totalElapsed() > sendInterval)
      {
         publishTextToSpeech("Sending messages.");
         sendFootsteps();
         sendChestTrajectory();
         sendArmTrajectory();

         timer.reset();
      }
   }

   private void sendArmTrajectory()
   {
      double[] leftArmHome = new double[] {0.78, -0.1, 3.0, 1.8, -0.3, 0.7, 0.15};
      ArmTrajectoryMessage armTrajectory = HumanoidMessageTools.createArmTrajectoryMessage(RobotSide.LEFT);

      for (int jointIdx = 0; jointIdx < leftArmHome.length; jointIdx++)
      {
         OneDoFJointTrajectoryMessage jointTrajectoryMessage = armTrajectory.getJointspaceTrajectory().getJointTrajectoryMessages().add();

         for (int i = 0; i < trajectoryPoints; i++)
         {
            double percent = i / (double) (trajectoryPoints - 1);
            jointTrajectoryMessage.getTrajectoryPoints().add().set(HumanoidMessageTools.createTrajectoryPoint1DMessage(percent, leftArmHome[jointIdx], 0.0));
         }
      }

      armPublisher.publish(armTrajectory);
   }

   private void sendChestTrajectory()
   {
      ReferenceFrame pelvisZUp = referenceFrames.getPelvisZUpFrame();
      ChestTrajectoryMessage chestTrajectory = new ChestTrajectoryMessage();
      SO3TrajectoryMessage so3Trajectory = chestTrajectory.getSo3Trajectory();
      so3Trajectory.getFrameInformation().setTrajectoryReferenceFrameId(MessageTools.toFrameId(pelvisZUp));
      so3Trajectory.getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(pelvisZUp));

      for (int i = 0; i < trajectoryPoints; i++)
      {
         double percent = i / (double) (trajectoryPoints - 1);
         SO3TrajectoryPointMessage trajectoryPoint = so3Trajectory.getTaskspaceTrajectoryPoints().add();
         trajectoryPoint.setTime(percent);
         trajectoryPoint.getOrientation().set(new Quaternion());
         trajectoryPoint.getAngularVelocity().set(new Vector3D());
      }

      chestPublisher.publish(chestTrajectory);
   }

   double initialZHeight = Double.NaN;

   private void sendFootsteps()
   {
      ReferenceFrame leftSoleFrame = referenceFrames.getSoleFrame(RobotSide.LEFT);

      FramePose3D stepPoseLeft = new FramePose3D(leftSoleFrame);
      FramePose3D stepPoseRight = new FramePose3D(leftSoleFrame);
      stepPoseRight.setY(-0.25);

      stepPoseLeft.changeFrame(ReferenceFrame.getWorldFrame());
      stepPoseRight.changeFrame(ReferenceFrame.getWorldFrame());

      if (Double.isNaN(initialZHeight))
      {
         initialZHeight = stepPoseLeft.getZ();
      }
      else
      {
         stepPoseLeft.setZ(initialZHeight);
         stepPoseRight.setZ(initialZHeight);
      }

      FootstepDataListMessage footsteps = HumanoidMessageTools.createFootstepDataListMessage(1.2, 0.8);
      for (int i = 0; i < steps / 2; i++)
      {
         footsteps.getFootstepDataList().add()
                  .set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, stepPoseLeft.getPosition(), stepPoseLeft.getOrientation()));
         footsteps.getFootstepDataList().add()
                  .set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, stepPoseRight.getPosition(), stepPoseRight.getOrientation()));
      }

      footstepPublisher.publish(footsteps);
   }

   @Override
   public void onBehaviorEntered()
   {
      publishTextToSpeech("Starting GC behavior.");
      timer.reset();
   }

   @Override
   public void onBehaviorAborted()
   {
   }

   @Override
   public void onBehaviorPaused()
   {
   }

   @Override
   public void onBehaviorResumed()
   {
      timer.reset();
   }

   @Override
   public void onBehaviorExited()
   {
   }

   @Override
   public boolean isDone()
   {
      return false;
   }
}
