package us.ihmc.humanoidBehaviors.behaviors.debug;

import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.SO3TrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.SO3TrajectoryPointMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.OneDoFJointTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.time.YoStopwatch;
import us.ihmc.yoVariables.variable.YoDouble;

public class TestGarbageGenerationBehavior extends AbstractBehavior
{
   private static final double sendInterval = 0.5;
   private static final int trajectoryPoints = 50;
   private static final int steps = 4;

   private final HumanoidReferenceFrames referenceFrames;
   private final YoStopwatch timer;

   public TestGarbageGenerationBehavior(CommunicationBridgeInterface communicationBridge, HumanoidReferenceFrames referenceFrames, YoDouble yoTime)
   {
      super(communicationBridge);
      this.referenceFrames = referenceFrames;
      timer = new YoStopwatch(yoTime);
   }

   @Override
   public void doControl()
   {
      if (timer.totalElapsed() > sendInterval)
      {
         sendPacketToUI(MessageTools.createTextToSpeechPacket("Sending messages."));

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
         OneDoFJointTrajectoryMessage jointTrajectoryMessage = armTrajectory.jointspaceTrajectory.jointTrajectoryMessages.add();

         for (int i = 0; i < trajectoryPoints; i++)
         {
            double percent = i / (double) (trajectoryPoints - 1);
            jointTrajectoryMessage.trajectoryPoints.add().set(HumanoidMessageTools.createTrajectoryPoint1DMessage(percent, leftArmHome[jointIdx], 0.0));
         }
      }

      sendPacketToController(armTrajectory);
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
         SO3TrajectoryPointMessage trajectoryPoint = so3Trajectory.taskspaceTrajectoryPoints.add();
         trajectoryPoint.setTime(percent);
         trajectoryPoint.setOrientation(new Quaternion());
         trajectoryPoint.setAngularVelocity(new Vector3D());
      }

      sendPacketToController(chestTrajectory);
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
         footsteps.footstepDataList.add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, stepPoseLeft.getPosition(), stepPoseLeft.getOrientation()));
         footsteps.footstepDataList.add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, stepPoseRight.getPosition(), stepPoseRight.getOrientation()));
      }

      sendPacketToController(footsteps);
   }

   @Override
   public void onBehaviorEntered()
   {
      sendPacketToUI(MessageTools.createTextToSpeechPacket("Starting GC behavior."));
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
