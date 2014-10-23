package us.ihmc.humanoidBehaviors.behaviors.midLevel;

import java.util.ArrayList;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packets.manipulation.HandPosePacket.Frame;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
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
   private final ReferenceFrame chest;
   private RobotSide robotSide;

   private long trajectoryTime;
   private final SideDependentList<RigidBodyTransform> midPoses = new SideDependentList<RigidBodyTransform>();
   private final SideDependentList<RigidBodyTransform> posesForDrop = new SideDependentList<RigidBodyTransform>();
   private final SideDependentList<RigidBodyTransform> finalPoses = new SideDependentList<RigidBodyTransform>();

   private final RigidBodyTransform[] posesForHandPoseBehavior = new RigidBodyTransform[4];

   public DropDebrisBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, FullRobotModel fullRobotModel,
         DoubleYoVariable yoTime)
   {
      super(outgoingCommunicationBridge);
      handPoseBehavior = new HandPoseBehavior(outgoingCommunicationBridge, yoTime);
      openHandBehavior = new OpenHandBehavior(outgoingCommunicationBridge);

      zHeightOffsetFromChestFrameForMidPoint = new DoubleYoVariable("zHeightOffsetFromChestFrameForMidPoint", registry);
      zHeightOffsetFromChestFrameForMidPoint.set(-0.27);

      haveInputsBeenSet = new BooleanYoVariable("haveInputsBeenSet", registry);
      reachedMidPoint = new BooleanYoVariable("reachedMidPoint", registry);
      behaviorIndex = new IntegerYoVariable("behaviorIndex", registry);
      isDone = new BooleanYoVariable("isDone", registry);

      handReferenceFrames.put(RobotSide.LEFT, fullRobotModel.getHandControlFrame(RobotSide.LEFT));
      handReferenceFrames.put(RobotSide.RIGHT, fullRobotModel.getHandControlFrame(RobotSide.RIGHT));

      chest = fullRobotModel.getChest().getBodyFixedFrame();
   }
   
   private RigidBodyTransform getDropLocation(RobotSide side)
   {
      if (side == RobotSide.LEFT)
      {
         Vector3d leftDropTranslation = new Vector3d(0, 1, 0);
         Quat4d leftDropRotation = new Quat4d();
         RotationFunctions.setQuaternionBasedOnYawPitchRoll(leftDropRotation, -Math.PI / 2, 0, 0);
         return new RigidBodyTransform(leftDropRotation, leftDropTranslation);
      }

      Vector3d rightDropTranslation = new Vector3d(0, -1, 0);
      Quat4d rightDropRotation = new Quat4d();
      RotationFunctions.setQuaternionBasedOnYawPitchRoll(rightDropRotation, Math.PI / 2, 0, 0);
      return new RigidBodyTransform(rightDropRotation, rightDropTranslation);
   }

   private RigidBodyTransform getEndLocation(RobotSide side)
   {
      if (side == RobotSide.LEFT)
      {
         Vector3d leftDropTranslation = new Vector3d(0.1, 0.3, -0.3);
         Quat4d leftDropRotation = new Quat4d();
         RotationFunctions.setQuaternionBasedOnYawPitchRoll(leftDropRotation, -Math.PI / 10, 0, 0);
         return new RigidBodyTransform(leftDropRotation, leftDropTranslation);
      }

      Vector3d rightDropTranslation = new Vector3d(0.1, -0.3, -0.3);
      Quat4d rightDropRotation = new Quat4d();
      RotationFunctions.setQuaternionBasedOnYawPitchRoll(rightDropRotation, Math.PI / 10, 0, 0);
      return new RigidBodyTransform(rightDropRotation, rightDropTranslation);
   }

   private RigidBodyTransform getMidPose(RobotSide side)
   {
      RigidBodyTransform pose = handReferenceFrames.get(side).getTransformToDesiredFrame(chest);
      Vector3d translation = new Vector3d();
      pose.getTranslation(translation);
      translation.setZ(zHeightOffsetFromChestFrameForMidPoint.getDoubleValue());
      pose.setTranslation(translation);
      return pose;
   }

   private void setPoses(RobotSide side)
   {

      posesForHandPoseBehavior[0] = getMidPose(side);
      posesForHandPoseBehavior[1] = getDropLocation(side);
      posesForHandPoseBehavior[2] = null;
      posesForHandPoseBehavior[3] = getEndLocation(side);
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
      haveInputsBeenSet.set(true);
      setPoses(side);
   }

   private void checkTransitionCondition()
   {
      if (currentBehavior.isDone())
      {
         currentBehavior.finalize();
         currentBehavior = behaviors.remove(0);
         behaviorIndex.add(1);
         if (currentBehavior != null)
         {
            currentBehavior.initialize();

            if (posesForHandPoseBehavior[behaviorIndex.getIntegerValue()] != null)
            {
               handPoseBehavior.setInput(Frame.CHEST, posesForHandPoseBehavior[behaviorIndex.getIntegerValue()], robotSide, trajectoryTime);
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
      currentBehavior.consumeObjectFromNetworkProcessor(object);
   }

   @Override
   protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {
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
