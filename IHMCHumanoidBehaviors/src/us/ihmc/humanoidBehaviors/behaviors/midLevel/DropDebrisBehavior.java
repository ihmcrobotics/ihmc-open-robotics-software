package us.ihmc.humanoidBehaviors.behaviors.midLevel;

import javax.vecmath.Quat4d;

import us.ihmc.communication.packets.dataobjects.FingerState;
import us.ihmc.communication.packets.manipulation.HandPosePacket.Frame;
import us.ihmc.communication.util.PacketControllerTools;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.taskExecutor.FingerStateTask;
import us.ihmc.humanoidBehaviors.taskExecutor.HandPoseTask;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.math.geometry.RotationFunctions;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.utilities.taskExecutor.TaskExecutor;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;

public class DropDebrisBehavior extends BehaviorInterface
{
   private final FullRobotModel fullRobotModel;

   private final TaskExecutor taskExecutor = new TaskExecutor();

   private final HandPoseBehavior handPoseBehavior;
   private final FingerStateBehavior fingerStateBehavior;

   private final BooleanYoVariable isDone;
   private final BooleanYoVariable haveInputsBeenSet;
   private final IntegerYoVariable behaviorIndex;
   private final DoubleYoVariable zHeightOffsetFromChestFrameForMidPoint;

   private final SideDependentList<ReferenceFrame> handReferenceFrames = new SideDependentList<ReferenceFrame>();

   private final ReferenceFrame chestFrame;
   private RobotSide robotSide;

   private double trajectoryTime = 3.5;

   private final FramePose midPose = new FramePose();
   private final FramePose dropLocationPose = new FramePose();

   private final DoubleYoVariable yoTime;

   public DropDebrisBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, FullRobotModel fullRobotModel, DoubleYoVariable yoTime)
   {
      super(outgoingCommunicationBridge);
      this.fullRobotModel = fullRobotModel;
      this.yoTime = yoTime;
      handPoseBehavior = new HandPoseBehavior(outgoingCommunicationBridge, yoTime);
      fingerStateBehavior = new FingerStateBehavior(outgoingCommunicationBridge, yoTime);

      zHeightOffsetFromChestFrameForMidPoint = new DoubleYoVariable("zHeightOffsetFromChestFrameForMidPoint", registry);
      zHeightOffsetFromChestFrameForMidPoint.set(0.27);

      haveInputsBeenSet = new BooleanYoVariable("haveInputsBeenSet", registry);
      behaviorIndex = new IntegerYoVariable("behaviorIndex", registry);
      isDone = new BooleanYoVariable("isDone", registry);

      handReferenceFrames.put(RobotSide.LEFT, fullRobotModel.getHandControlFrame(RobotSide.LEFT));
      handReferenceFrames.put(RobotSide.RIGHT, fullRobotModel.getHandControlFrame(RobotSide.RIGHT));

      chestFrame = fullRobotModel.getChest().getBodyFixedFrame();
   }

   private void getMidPose()
   {
      midPose.translate(0.0, 0.0, zHeightOffsetFromChestFrameForMidPoint.getDoubleValue());
      midPose.changeFrame(ReferenceFrame.getWorldFrame());
   }

   private void getDropLocation(RobotSide side)
   {
      Quat4d dropRotation = new Quat4d();
      if (side == RobotSide.LEFT)
      {
         dropLocationPose.translate(0.3, 0.7, -0.1);
         RotationFunctions.setQuaternionBasedOnYawPitchRoll(dropRotation, -Math.PI / 2, Math.PI / 2, 0);
      }
      else
      {
         dropLocationPose.translate(0.3, -0.7, -0.1);
         RotationFunctions.setQuaternionBasedOnYawPitchRoll(dropRotation, Math.PI / 2, Math.PI / 2, 0);
      }
      dropLocationPose.setOrientation(dropRotation);
      dropLocationPose.changeFrame(ReferenceFrame.getWorldFrame());
   }

   private void setPoses(RobotSide side)
   {
      RigidBodyTransform tempPose = new RigidBodyTransform();

      initializePoses(side);

      getMidPose();
      midPose.getPose(tempPose);
      taskExecutor.submit(new HandPoseTask(side, yoTime, handPoseBehavior, Frame.WORLD, tempPose, trajectoryTime));

      getDropLocation(side);
      dropLocationPose.getPose(tempPose);
      taskExecutor.submit(new HandPoseTask(side, yoTime, handPoseBehavior, Frame.WORLD, tempPose, trajectoryTime));

      taskExecutor.submit(new FingerStateTask(robotSide, FingerState.OPEN, fingerStateBehavior));

      taskExecutor.submit(new HandPoseTask(PacketControllerTools.createGoToHomePacket(side, 3.0), handPoseBehavior, yoTime));

   }

   private void initializePoses(RobotSide side)
   {
      midPose.setToZero(fullRobotModel.getHandControlFrame(side));
      midPose.changeFrame(chestFrame);
      dropLocationPose.changeFrame(chestFrame);
      dropLocationPose.setToZero(chestFrame);
   }

   @Override
   public void doControl()
   {
      taskExecutor.doControl();
   }

   public void setInputs(RobotSide side)
   {
      robotSide = side;
      setPoses(side);
      haveInputsBeenSet.set(true);
   }

   @Override
   protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
   {
      if (taskExecutor.getCurrentTask() instanceof HandPoseTask)
         handPoseBehavior.consumeObjectFromNetworkProcessor(object);
      if (taskExecutor.getCurrentTask() instanceof FingerStateTask)
         fingerStateBehavior.consumeObjectFromNetworkProcessor(object);
   }

   @Override
   protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {
      if (taskExecutor.getCurrentTask() instanceof HandPoseTask)
         handPoseBehavior.consumeObjectFromController(object);
      if (taskExecutor.getCurrentTask() instanceof FingerStateTask)
         fingerStateBehavior.consumeObjectFromController(object);
   }

   @Override
   public void stop()
   {
      handPoseBehavior.stop();
   }

   @Override
   public void enableActions()
   {
      handPoseBehavior.enableActions();
   }

   @Override
   public void pause()
   {
      handPoseBehavior.pause();
   }

   @Override
   public void resume()
   {
      handPoseBehavior.resume();
   }

   @Override
   public boolean isDone()
   {
      return (taskExecutor.isDone() && hasInputBeenSet());
   }

   @Override
   public void finalize()
   {
      haveInputsBeenSet.set(false);
      isDone.set(false);
      behaviorIndex.set(-1);
   }

   @Override
   public void initialize()
   {
      haveInputsBeenSet.set(false);
      isDone.set(false);
      behaviorIndex.set(0);
   }

   @Override
   public boolean hasInputBeenSet()
   {
      return haveInputsBeenSet.getBooleanValue();
   }
}
