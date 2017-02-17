package us.ihmc.humanoidBehaviors.utilities;

import java.util.LinkedHashMap;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior.BehaviorStatus;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModePacket.BehaviorControlModeEnum;
import us.ihmc.humanoidRobotics.communication.subscribers.HumanoidRobotDataReceiver;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public abstract class StopThreadUpdatable implements Updatable
{
   protected final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   protected final HumanoidRobotDataReceiver robotDataReceiver;
   protected final AbstractBehavior behavior;
   protected final ReferenceFrame frameToKeepTrackOf;
   protected final RigidBodyTransform currentTransformToWorld;

   private boolean stopBehaviorThread = false;

   private BehaviorControlModeEnum requestedControlMode = BehaviorControlModeEnum.RESUME;

   protected LinkedHashMap<BehaviorControlModeEnum, BehaviorStatus> behaviorStatus = new LinkedHashMap<BehaviorControlModeEnum, BehaviorStatus>();
   protected LinkedHashMap<BehaviorControlModeEnum, RigidBodyTransform> testFrameTransformToWorld = new LinkedHashMap<BehaviorControlModeEnum, RigidBodyTransform>();
   protected LinkedHashMap<BehaviorControlModeEnum, HumanoidReferenceFrames> referenceFramesForLogging = new LinkedHashMap<BehaviorControlModeEnum, HumanoidReferenceFrames>();

   public StopThreadUpdatable(HumanoidRobotDataReceiver robotDataReceiver, AbstractBehavior behavior, ReferenceFrame frameToKeepTrackOf)
   {
      this.robotDataReceiver = robotDataReceiver;
      this.behavior = behavior;
      this.frameToKeepTrackOf = frameToKeepTrackOf;
      this.currentTransformToWorld = new RigidBodyTransform();

      testFrameTransformToWorld.put(BehaviorControlModeEnum.PAUSE, new RigidBodyTransform());
      testFrameTransformToWorld.put(BehaviorControlModeEnum.STOP, new RigidBodyTransform());
      testFrameTransformToWorld.put(BehaviorControlModeEnum.RESUME, new RigidBodyTransform());
   }

   public BehaviorControlModeEnum getRequestedBehaviorControlMode()
   {
      return requestedControlMode;
   }

   public void setRequestedBehaviorControlMode(BehaviorControlModeEnum newRequestedControlMode)
   {
      if (!this.requestedControlMode.equals(newRequestedControlMode))
      {
         this.requestedControlMode = newRequestedControlMode;
         behaviorStatus.put(newRequestedControlMode, behavior.getBehaviorStatus());
         testFrameTransformToWorld.get(newRequestedControlMode).set(getCurrentTestFrameTransformToWorld());         
         referenceFramesForLogging.put(newRequestedControlMode, robotDataReceiver.getUpdatedReferenceFramesCopy());
      }
   }

   public HumanoidReferenceFrames getReferenceFramesAtTransition(BehaviorControlModeEnum controlMode)
   {
      return referenceFramesForLogging.get(controlMode);
   }
   
   public RigidBodyTransform getCurrentTestFrameTransformToWorld()
   {
      robotDataReceiver.updateRobotModel();
      frameToKeepTrackOf.getTransformToDesiredFrame(currentTransformToWorld, worldFrame);
      return currentTransformToWorld;
   }

   public RigidBodyTransform getTestFrameTransformToWorldAtTransition(BehaviorControlModeEnum controlMode)
   {
      return testFrameTransformToWorld.get(controlMode);
   }

   public FramePose getTestFramePoseCopyAtTransition(BehaviorControlModeEnum controlMode)
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

   public FramePose2d getTestFramePose2dAtTransition(BehaviorControlModeEnum controlMode)
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

   public BehaviorStatus getBehaviorStatusAtTransition(BehaviorControlModeEnum controlMode)
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
