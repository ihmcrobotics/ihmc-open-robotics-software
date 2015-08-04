package us.ihmc.humanoidBehaviors.utilities;

import java.util.LinkedHashMap;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.communication.packets.behaviors.HumanoidBehaviorControlModePacket.HumanoidBehaviorControlModeEnum;
import us.ihmc.communication.subscribers.RobotDataReceiver;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface.BehaviorStatus;
import us.ihmc.robotics.humanoidRobot.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public abstract class StopThreadUpdatable implements Updatable
{
   protected final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   protected final RobotDataReceiver robotDataReceiver;
   protected final BehaviorInterface behavior;
   protected final ReferenceFrame frameToKeepTrackOf;
   protected final RigidBodyTransform currentTransformToWorld;

   private boolean stopBehaviorThread = false;

   private HumanoidBehaviorControlModeEnum requestedControlMode = HumanoidBehaviorControlModeEnum.ENABLE_ACTIONS;

   protected LinkedHashMap<HumanoidBehaviorControlModeEnum, BehaviorStatus> behaviorStatus = new LinkedHashMap<HumanoidBehaviorControlModeEnum, BehaviorStatus>();
   protected LinkedHashMap<HumanoidBehaviorControlModeEnum, RigidBodyTransform> testFrameTransformToWorld = new LinkedHashMap<HumanoidBehaviorControlModeEnum, RigidBodyTransform>();
   protected LinkedHashMap<HumanoidBehaviorControlModeEnum, HumanoidReferenceFrames> referenceFramesForLogging = new LinkedHashMap<HumanoidBehaviorControlModeEnum, HumanoidReferenceFrames>();

   public StopThreadUpdatable(RobotDataReceiver robotDataReceiver, BehaviorInterface behavior, ReferenceFrame frameToKeepTrackOf)
   {
      this.robotDataReceiver = robotDataReceiver;
      this.behavior = behavior;
      this.frameToKeepTrackOf = frameToKeepTrackOf;
      this.currentTransformToWorld = new RigidBodyTransform();

      testFrameTransformToWorld.put(HumanoidBehaviorControlModeEnum.PAUSE, new RigidBodyTransform());
      testFrameTransformToWorld.put(HumanoidBehaviorControlModeEnum.STOP, new RigidBodyTransform());
      testFrameTransformToWorld.put(HumanoidBehaviorControlModeEnum.RESUME, new RigidBodyTransform());
   }

   public HumanoidBehaviorControlModeEnum getRequestedBehaviorControlMode()
   {
      return requestedControlMode;
   }

   public void setRequestedBehaviorControlMode(HumanoidBehaviorControlModeEnum newRequestedControlMode)
   {
      if (!this.requestedControlMode.equals(newRequestedControlMode))
      {
         this.requestedControlMode = newRequestedControlMode;
         behaviorStatus.put(newRequestedControlMode, behavior.getBehaviorStatus());
         testFrameTransformToWorld.get(newRequestedControlMode).set(getCurrentTestFrameTransformToWorld());         
         referenceFramesForLogging.put(newRequestedControlMode, robotDataReceiver.getUpdatedReferenceFramesCopy());
      }
   }

   public HumanoidReferenceFrames getReferenceFramesAtTransition(HumanoidBehaviorControlModeEnum controlMode)
   {
      return referenceFramesForLogging.get(controlMode);
   }
   
   public RigidBodyTransform getCurrentTestFrameTransformToWorld()
   {
      robotDataReceiver.updateRobotModel();
      frameToKeepTrackOf.getTransformToDesiredFrame(currentTransformToWorld, worldFrame);
      return currentTransformToWorld;
   }

   public RigidBodyTransform getTestFrameTransformToWorldAtTransition(HumanoidBehaviorControlModeEnum controlMode)
   {
      return testFrameTransformToWorld.get(controlMode);
   }

   public FramePose getTestFramePoseCopyAtTransition(HumanoidBehaviorControlModeEnum controlMode)
   {
      FramePose ret = new FramePose(ReferenceFrame.getWorldFrame(), getTestFrameTransformToWorldAtTransition(controlMode));
      return ret;
   }

   public FramePose getCurrentTestFramePoseCopy()
   {
      FramePose ret = new FramePose();
      getCurrentTestFramePose(ret);
      return ret;
   }

   public void getCurrentTestFramePose(FramePose poseToPack)
   {
      poseToPack.setPoseIncludingFrame(worldFrame, getCurrentTestFrameTransformToWorld());
   }

   public FramePose2d getTestFramePose2dAtTransition(HumanoidBehaviorControlModeEnum controlMode)
   {
      FramePose2d ret = new FramePose2d();
      ret.setPose(getTestFrameTransformToWorldAtTransition(controlMode));
      return ret;
   }

   public FramePose2d getCurrentTestFramePose2dCopy()
   {
      return getTestFramePose2dCopy(getCurrentTestFrameTransformToWorld());
   }

   public FramePose2d getTestFramePose2dCopy(RigidBodyTransform testFrameTransformToWorld)
   {
      FramePose2d ret = new FramePose2d(ReferenceFrame.getWorldFrame());
      ret.setPose(testFrameTransformToWorld);
      return ret;
   }

   public BehaviorStatus getBehaviorStatusAtTransition(HumanoidBehaviorControlModeEnum controlMode)
   {
      return behaviorStatus.get(controlMode);
   }

   protected void setShouldBehaviorRunnerBeStopped(boolean stop)
   {
      stopBehaviorThread = stop;
   }

   public boolean shouldBehaviorRunnerBeStopped()
   {
      return stopBehaviorThread;
   }
}
