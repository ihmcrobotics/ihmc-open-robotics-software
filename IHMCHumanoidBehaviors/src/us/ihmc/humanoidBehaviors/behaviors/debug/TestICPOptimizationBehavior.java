package us.ihmc.humanoidBehaviors.behaviors.debug;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage.FootstepOrigin;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.time.YoTimer;

public class TestICPOptimizationBehavior extends AbstractBehavior
{
   private final HumanoidReferenceFrames referenceFrames;
   private final DoubleYoVariable swingTime = new DoubleYoVariable("BehaviorSwingTime", registry);
   private final DoubleYoVariable sleepTime = new DoubleYoVariable("BehaviorSleepTime", registry);
   private final DoubleYoVariable transferTime = new DoubleYoVariable("BehaviorTransferTime", registry);
   private final BooleanYoVariable stepInPlace = new BooleanYoVariable("StepInPlace", registry);
   private final BooleanYoVariable abortBehavior = new BooleanYoVariable("AbortBehavior", registry);

   private final YoTimer timer;

   public TestICPOptimizationBehavior(CommunicationBridgeInterface communicationBridge, HumanoidReferenceFrames referenceFrames, DoubleYoVariable yoTime)
   {
      super(communicationBridge);
      this.referenceFrames = referenceFrames;

      swingTime.set(1.2);
      transferTime.set(0.6);
      sleepTime.set(10.0);

      timer = new YoTimer(yoTime);
   }

   @Override
   public void doControl()
   {
      if (!(timer.totalElapsed() > sleepTime.getDoubleValue()))
         return;

      FootstepDataListMessage footsteps = new FootstepDataListMessage(swingTime.getDoubleValue(), transferTime.getDoubleValue());
      footsteps.setExecutionMode(ExecutionMode.OVERRIDE);
      footsteps.setDestination(PacketDestination.BROADCAST);

      ReferenceFrame leftSoleFrame = referenceFrames.getSoleFrame(RobotSide.LEFT);
      ReferenceFrame rightSoleFrame = referenceFrames.getSoleFrame(RobotSide.RIGHT);
      FramePoint rightFoot = new FramePoint(rightSoleFrame);
      rightFoot.changeFrame(leftSoleFrame);
      FramePose stepPose = new FramePose(leftSoleFrame);
      stepPose.setY(-0.25);

      if (Math.abs(rightFoot.getX()) > 0.1)
      {
         sendPacket(new TextToSpeechPacket("Squaring up."));
      }
      else if (!stepInPlace.getBooleanValue())
      {
         sendPacket(new TextToSpeechPacket("Step forward."));
         stepPose.setX(0.3);
      }
      else
      {
         sendPacket(new TextToSpeechPacket("Step in place."));
      }

      stepPose.changeFrame(ReferenceFrame.getWorldFrame());

      Point3d location = new Point3d();
      Quat4d orientation = new Quat4d();
      stepPose.getPose(location, orientation);

      FootstepDataMessage footstepData = new FootstepDataMessage(RobotSide.RIGHT, location, orientation);
      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      footsteps.add(footstepData);

      sendPacket(footsteps);
      timer.reset();
   }

   @Override
   public void onBehaviorEntered()
   {
      abortBehavior.set(false);
      stepInPlace.set(true);
      sendPacket(new TextToSpeechPacket("Starting to step forward and backward with the right foot."));
   }

   @Override
   public void onBehaviorAborted()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void onBehaviorPaused()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void onBehaviorResumed()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void onBehaviorExited()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public boolean isDone()
   {
      return abortBehavior.getBooleanValue();
   }
}
