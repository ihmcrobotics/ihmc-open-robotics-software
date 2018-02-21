package us.ihmc.humanoidBehaviors.behaviors.debug;

import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.time.YoStopwatch;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class TestICPOptimizationBehavior extends AbstractBehavior
{
   private final HumanoidReferenceFrames referenceFrames;
   private final YoDouble swingTime = new YoDouble("BehaviorSwingTime", registry);
   private final YoDouble sleepTime = new YoDouble("BehaviorSleepTime", registry);
   private final YoDouble transferTime = new YoDouble("BehaviorTransferTime", registry);
   private final YoDouble stepLength = new YoDouble("BehaviorStepLength", registry);
   private final YoBoolean stepInPlace = new YoBoolean("StepInPlace", registry);
   private final YoBoolean abortBehavior = new YoBoolean("AbortBehavior", registry);

   private final YoStopwatch timer;

   public TestICPOptimizationBehavior(CommunicationBridgeInterface communicationBridge, HumanoidReferenceFrames referenceFrames, YoDouble yoTime)
   {
      super(communicationBridge);
      this.referenceFrames = referenceFrames;

      swingTime.set(1.2);
      transferTime.set(0.6);
      sleepTime.set(10.0);
      stepLength.set(0.3);

      timer = new YoStopwatch(yoTime);
   }

   @Override
   public void doControl()
   {
      if (!(timer.totalElapsed() > sleepTime.getDoubleValue()))
         return;

      FootstepDataListMessage footsteps = HumanoidMessageTools.createFootstepDataListMessage(swingTime.getDoubleValue(), transferTime.getDoubleValue());
      footsteps.queueingProperties.setExecutionMode(ExecutionMode.OVERRIDE.toByte());
      footsteps.queueingProperties.setPreviousMessageId(FootstepDataListMessage.VALID_MESSAGE_DEFAULT_ID);
      footsteps.setDestination(PacketDestination.BROADCAST);

      ReferenceFrame leftSoleFrame = referenceFrames.getSoleFrame(RobotSide.LEFT);
      ReferenceFrame rightSoleFrame = referenceFrames.getSoleFrame(RobotSide.RIGHT);
      FramePoint3D rightFoot = new FramePoint3D(rightSoleFrame);
      rightFoot.changeFrame(leftSoleFrame);
      FramePose3D stepPose = new FramePose3D(leftSoleFrame);
      stepPose.setY(-0.25);

      if (Math.abs(rightFoot.getX()) > 0.1)
      {
         sendPacket(MessageTools.createTextToSpeechPacket("Squaring up."));
      }
      else if (!stepInPlace.getBooleanValue())
      {
         sendPacket(MessageTools.createTextToSpeechPacket("Step forward."));
         stepPose.setX(stepLength.getDoubleValue());
      }
      else
      {
         sendPacket(MessageTools.createTextToSpeechPacket("Step in place."));
      }

      stepPose.changeFrame(ReferenceFrame.getWorldFrame());

      Point3D location = new Point3D();
      Quaternion orientation = new Quaternion();
      stepPose.get(location, orientation);

      FootstepDataMessage footstepData = HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, location, orientation);
      footsteps.footstepDataList.add().set(footstepData);

      sendPacket(footsteps);
      timer.reset();
   }

   @Override
   public void onBehaviorEntered()
   {
      abortBehavior.set(false);
      stepInPlace.set(true);
      sendPacket(MessageTools.createTextToSpeechPacket("Starting to step forward and backward with the right foot."));
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

   }

   @Override
   public void onBehaviorExited()
   {

   }

   @Override
   public boolean isDone()
   {
      return abortBehavior.getBooleanValue();
   }
}
