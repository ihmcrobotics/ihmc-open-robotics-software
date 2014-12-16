package us.ihmc.humanoidBehaviors.behaviors.midLevel;

import java.util.ArrayList;

import javax.vecmath.Quat4d;

import us.ihmc.communication.packets.manipulation.HandPosePacket.Frame;
import us.ihmc.communication.util.PacketControllerTools;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.math.geometry.RotationFunctions;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;

public class DropDebrisBehavior extends BehaviorInterface
{
   private final FullRobotModel fullRobotModel;
   private BehaviorInterface currentBehavior;
   private final ArrayList<BehaviorInterface> behaviors = new ArrayList<BehaviorInterface>();
   private final HandPoseBehavior handPoseBehavior;
   private final OpenHandBehavior openHandBehavior;

   private final BooleanYoVariable isDone;
   private final BooleanYoVariable haveInputsBeenSet;
   private final BooleanYoVariable reachedMidPoint;
   private final IntegerYoVariable behaviorIndex;
   private final DoubleYoVariable zHeightOffsetFromChestFrameForMidPoint;

   private final SideDependentList<ReferenceFrame> handReferenceFrames = new SideDependentList<ReferenceFrame>();

   private final ReferenceFrame chestFrame;
   private RobotSide robotSide;

   private double trajectoryTime = 3.5;

   private final RigidBodyTransform[] posesForHandPoseBehavior = new RigidBodyTransform[4];

   private final FramePose midPose = new FramePose();
   private final FramePose dropLocationPose = new FramePose();

   public DropDebrisBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, FullRobotModel fullRobotModel, DoubleYoVariable yoTime)
   {
      super(outgoingCommunicationBridge);
      this.fullRobotModel = fullRobotModel;
      handPoseBehavior = new HandPoseBehavior(outgoingCommunicationBridge, yoTime);
      openHandBehavior = new OpenHandBehavior(outgoingCommunicationBridge);

      zHeightOffsetFromChestFrameForMidPoint = new DoubleYoVariable("zHeightOffsetFromChestFrameForMidPoint", registry);
      zHeightOffsetFromChestFrameForMidPoint.set(0.27);

      haveInputsBeenSet = new BooleanYoVariable("haveInputsBeenSet", registry);
      reachedMidPoint = new BooleanYoVariable("reachedMidPoint", registry);
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
      Quat4d leftDropRotation = new Quat4d();
      if (side == RobotSide.LEFT)
      {
         dropLocationPose.translate(0.3, 0.7, -0.1);
         RotationFunctions.setQuaternionBasedOnYawPitchRoll(leftDropRotation, -Math.PI / 2, 0, 0);
      }
      else
      {
         dropLocationPose.translate(0.3, -0.7, -0.1);
         RotationFunctions.setQuaternionBasedOnYawPitchRoll(leftDropRotation, Math.PI / 2, 0, 0);
      }
      dropLocationPose.setOrientation(leftDropRotation);
      dropLocationPose.changeFrame(ReferenceFrame.getWorldFrame());
   }

   private void setPoses(RobotSide side)
   {
      initializePoses(side);
      getMidPose();
      posesForHandPoseBehavior[0] = new RigidBodyTransform();
      midPose.getPose(posesForHandPoseBehavior[0]);

      getDropLocation(side);
      posesForHandPoseBehavior[1] = new RigidBodyTransform();
      dropLocationPose.getPose(posesForHandPoseBehavior[1]);

      posesForHandPoseBehavior[2] = null;
      posesForHandPoseBehavior[3] = null;

      handPoseBehavior.setInput(Frame.CHEST, posesForHandPoseBehavior[0], robotSide, trajectoryTime);
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
      if (!isDone.getBooleanValue() && currentBehavior != null)
      {
         checkTransitionCondition();
         currentBehavior.doControl();
      }
   }

   public void setInputs(RobotSide side)
   {
      robotSide = side;
      setPoses(side);
      haveInputsBeenSet.set(true);
   }

   private void checkTransitionCondition()
   {
      if (currentBehavior.isDone())
      {
         currentBehavior.finalize();

         if (!behaviors.isEmpty())
         {
            currentBehavior = behaviors.remove(0);
            behaviorIndex.add(1);
            currentBehavior.initialize();

            if (posesForHandPoseBehavior[behaviorIndex.getIntegerValue()] != null)
            {
               handPoseBehavior.setInput(Frame.CHEST, posesForHandPoseBehavior[behaviorIndex.getIntegerValue()], robotSide, trajectoryTime);
            }

            if (behaviors.size() == 1)
            {
               handPoseBehavior.setInput(PacketControllerTools.createGoToHomePacket(robotSide, trajectoryTime));
            }
         }
         else
         {
            isDone.set(true);
         }
      }
   }

   @Override
   protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
   {
      if (currentBehavior != null)
         currentBehavior.consumeObjectFromNetworkProcessor(object);
   }

   @Override
   protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {
      if (currentBehavior != null)
         currentBehavior.consumeObjectFromController(object);
   }

   @Override
   public void stop()
   {
      currentBehavior.stop();
   }

   @Override
   public void enableActions()
   {
      currentBehavior.enableActions();
   }

   @Override
   public void pause()
   {
      currentBehavior.pause();
   }

   @Override
   public void resume()
   {
      currentBehavior.resume();
   }

   @Override
   public boolean isDone()
   {
      return isDone.getBooleanValue();
   }

   @Override
   public void finalize()
   {
      currentBehavior = null;
      behaviors.clear();
      haveInputsBeenSet.set(false);
      isDone.set(false);
      reachedMidPoint.set(false);
      behaviorIndex.set(-1);
   }

   @Override
   public void initialize()
   {
      behaviors.clear();
      behaviors.add(handPoseBehavior);
      behaviors.add(handPoseBehavior);
      behaviors.add(openHandBehavior);
      behaviors.add(handPoseBehavior);

      currentBehavior = behaviors.remove(0);
      haveInputsBeenSet.set(false);
      isDone.set(false);
      reachedMidPoint.set(false);
      behaviorIndex.set(0);
   }

   @Override
   public boolean hasInputBeenSet()
   {
      return haveInputsBeenSet.getBooleanValue();
   }

}
