package us.ihmc.humanoidBehaviors.utilities;

import java.util.LinkedHashMap;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.communication.packets.behaviors.HumanoidBehaviorControlModePacket.HumanoidBehaviorControlModeEnum;
import us.ihmc.communication.subscribers.RobotDataReceiver;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface.BehaviorStatus;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;

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
         captureTransformToWorld(newRequestedControlMode);
      }
   }

   protected void captureTransformToWorld(HumanoidBehaviorControlModeEnum newControlMode)
   {
      testFrameTransformToWorld.get(newControlMode).set(getCurrentTestFrameTransformToWorld());
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

   public FramePose getTestFramePoseAtTransition(HumanoidBehaviorControlModeEnum controlMode)
   {
      FramePose ret = new FramePose(ReferenceFrame.getWorldFrame(), getTestFrameTransformToWorldAtTransition(controlMode));
      return ret;
   }

   public FramePose getCurrentTestFramePose()
   {
      FramePose ret = new FramePose(ReferenceFrame.getWorldFrame(), getCurrentTestFrameTransformToWorld());
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
