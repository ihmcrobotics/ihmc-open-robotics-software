package us.ihmc.humanoidBehaviors.behaviors.debug;

import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.time.YoStopwatch;
import us.ihmc.yoVariables.variable.YoDouble;

public class TestGarbageGenerationBehavior extends AbstractBehavior
{
   private static final double swingTime = 1.2;
   private static final double transferTime = 0.8;
   private static final double sendStepsInterval = 10.0;
   private static final double sendChestInterval = 5.0;
   private static final double sendArmInterval = 3.0;

   private final HumanoidReferenceFrames referenceFrames;
   private final YoStopwatch stepTimer;
   private final YoStopwatch chestTimer;
   private final YoStopwatch armTimer;

   public TestGarbageGenerationBehavior(CommunicationBridgeInterface communicationBridge, HumanoidReferenceFrames referenceFrames, YoDouble yoTime)
   {
      super(communicationBridge);
      this.referenceFrames = referenceFrames;
      stepTimer = new YoStopwatch(yoTime);
      chestTimer = new YoStopwatch(yoTime);
      armTimer = new YoStopwatch(yoTime);
   }

   @Override
   public void doControl()
   {
      if (stepTimer.totalElapsed() > sendStepsInterval)
      {
         ReferenceFrame leftSoleFrame = referenceFrames.getSoleFrame(RobotSide.LEFT);

         FramePose stepPoseLeft = new FramePose(leftSoleFrame);
         FramePose stepPoseRight = new FramePose(leftSoleFrame);
         stepPoseRight.setY(-0.25);

         stepPoseLeft.changeFrame(ReferenceFrame.getWorldFrame());
         stepPoseRight.changeFrame(ReferenceFrame.getWorldFrame());

         FootstepDataListMessage footsteps = new FootstepDataListMessage(swingTime, transferTime);
         footsteps.add(new FootstepDataMessage(RobotSide.LEFT, stepPoseLeft.getPosition(), stepPoseLeft.getOrientation()));
         footsteps.add(new FootstepDataMessage(RobotSide.RIGHT, stepPoseRight.getPosition(), stepPoseRight.getOrientation()));

         sendPacket(new TextToSpeechPacket("Sending 2 Steps..."));
         sendPacketToController(footsteps);
         stepTimer.reset();
      }

      if (chestTimer.totalElapsed() > sendChestInterval)
      {
         ReferenceFrame pelvisZUp = referenceFrames.getPelvisZUpFrame();

         int points = 50;
         ChestTrajectoryMessage chestTrajectory = new ChestTrajectoryMessage(points);
         chestTrajectory.getFrameInformation().setTrajectoryReferenceFrame(pelvisZUp);
         chestTrajectory.getFrameInformation().setDataReferenceFrame(pelvisZUp);

         for (int i = 0; i < points; i++)
         {
            double percent = (double) i / (double) (points - 1);
            chestTrajectory.setTrajectoryPoint(0, percent, new Quaternion(), new Vector3D(), pelvisZUp);
         }

         sendPacket(new TextToSpeechPacket("Sending Chest Trajectory with " + points + " points."));
         sendPacketToController(chestTrajectory);
         stepTimer.reset();
      }

      if (armTimer.totalElapsed() > sendArmInterval)
      {
         double[] leftArmHome = new double[] {0.79, -0.1, 3.0, 1.8, -0.3, 0.7, 0.15};
         int points = 50;

         ArmTrajectoryMessage armTrajectory = new ArmTrajectoryMessage(RobotSide.LEFT, leftArmHome.length, points);
         for (int i = 0; i < points; i++)
         {
            double percent = (double) i / (double) (points - 1);
            for (int jointIdx = 0; jointIdx < leftArmHome.length; jointIdx++)
            {
               armTrajectory.setTrajectoryPoint(jointIdx, i, percent, leftArmHome[jointIdx], 0.0);
            }
         }

         sendPacket(new TextToSpeechPacket("Sending Left Arm Trajectory with " + points + " points."));
         sendPacketToController(armTrajectory);
         stepTimer.reset();
      }
   }

   @Override
   public void onBehaviorEntered()
   {
      sendPacket(new TextToSpeechPacket("Starting GC behavior."));
      stepTimer.reset();
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
      stepTimer.reset();
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
