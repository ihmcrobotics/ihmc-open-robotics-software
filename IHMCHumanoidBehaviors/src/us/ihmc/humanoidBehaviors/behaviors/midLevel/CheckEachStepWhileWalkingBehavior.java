package us.ihmc.humanoidBehaviors.behaviors.midLevel;

import java.util.ArrayList;
import java.util.LinkedList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FootstepListBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.LookAtBehavior;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepData;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataList;
import us.ihmc.humanoidRobotics.communication.packets.walking.SnapFootstepPacket;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/** 
 * Use LookAtBehavior() to provide visual feedback about upcoming footsteps.
 * Input is updated whenever a new FootStepList() [Packet] is received
 * Ignore footsteps that are "too close" to the robot (inside the no_looky_radius).
 * 
 * @author Brandon McShrewsington Crunchberry
 *
 */

public class CheckEachStepWhileWalkingBehavior extends BehaviorInterface
{
   private static final double NO_LOOKY_RADIUS = 1.0;

   private final BooleanYoVariable isDone;
   private final BooleanYoVariable haveInputsBeenSet;

   private final ConcurrentListeningQueue<SnapFootstepPacket> SnapFootstepQueue;
   private final ConcurrentListeningQueue<FootstepDataList> footstepListQueue;
   private final FootstepListBehavior footstepListBehavior;
   private final LookAtBehavior lookAtBehavior;
   private final LinkedList<FootstepData> footStepToTake;
   private final LinkedList<FootstepData> footStepsToLookAt;
   private final HumanoidReferenceFrames referenceFrames;
   private final ReferenceFrame midZUpFrame;
   private final RigidBodyTransform midZUpTransform = new RigidBodyTransform();
   private final Vector3d midZUpTranslation = new Vector3d();
   private final Vector3d footStepToLookAtTranslation = new Vector3d();
   private final ArrayList<FootstepData> outgoingFootStepsForSnapping = new ArrayList<FootstepData>();

   private final FootstepDataList outgoingFootstepDataList;
   private FootstepDataList currentFootStepList;
   private FootstepData currentFootBeingLookedAt;
   
   public CheckEachStepWhileWalkingBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, HumanoidReferenceFrames referenceFrames, WalkingControllerParameters walkingControllerParameters, DoubleYoVariable yoTime)
   {
      super(outgoingCommunicationBridge);
      isDone = new BooleanYoVariable(behaviorName + "_isDone", registry);
      haveInputsBeenSet = new BooleanYoVariable(behaviorName + "_haveInputsBeenSet", registry);

      footstepListQueue = new ConcurrentListeningQueue<FootstepDataList>();
      SnapFootstepQueue = new ConcurrentListeningQueue<SnapFootstepPacket>();
      footstepListBehavior = new FootstepListBehavior(outgoingCommunicationBridge, walkingControllerParameters);
      lookAtBehavior = new LookAtBehavior(outgoingCommunicationBridge, walkingControllerParameters, yoTime);
      this.referenceFrames = referenceFrames;
      midZUpFrame = referenceFrames.getMidFeetZUpFrame();

      footStepToTake = new LinkedList<FootstepData>();
      footStepsToLookAt = new LinkedList<FootstepData>();
      outgoingFootstepDataList = new FootstepDataList();
   }

   @Override
   public void doControl()
   {
      if (footstepListQueue.isNewPacketAvailable())
      {
         setNewFootSteps(footstepListQueue.getNewestPacket());
      }

      if (SnapFootstepQueue.isNewPacketAvailable())
      {
         processSnapFootstepResponse(SnapFootstepQueue.getNewestPacket());
      }

      processLooks();
      lookAtBehavior.doControl();

      processNextStepToTake();
      footstepListBehavior.doControl();
   }

   private void processNextStepToTake()
   {
      if (!footstepListBehavior.isWalking() && !footStepToTake.isEmpty())
      {
         sendFootStepToAtlas(footStepToTake);
      }
   }

   private void processLooks()
   {
      if (lookAtBehavior.isDone())
      {
         outgoingFootStepsForSnapping.add(currentFootBeingLookedAt);
         sendPacketToNetworkProcessor(new SnapFootstepPacket(outgoingFootStepsForSnapping, new int[] {}, new byte[] {}));
         lookAtBehavior.initialize();
      }

      if (!lookAtBehavior.isLooking() && !footStepsToLookAt.isEmpty())
      {
         FootstepData currentFootStepData = footStepsToLookAt.peek();
         if (isTargetFarEnoughAwayToLookAt(currentFootStepData.getLocation()))
         {
            lookAtStep(currentFootStepData);
         }
         else
         {
            System.out.println("Taking Steps too close to look at");
            {
               processSnapFootstepResponse(currentFootStepData);
            }
         }
         footStepsToLookAt.removeFirst();
      }
   }

   private boolean isTargetFarEnoughAwayToLookAt(Point3d targetsLocation)
   {
      midZUpFrame.getTransformToDesiredFrame(midZUpTransform, ReferenceFrame.getWorldFrame());
      midZUpTransform.getTranslation(midZUpTranslation);
      footStepToLookAtTranslation.set(targetsLocation.getX(), targetsLocation.getY(), targetsLocation.getZ());
      if (Math.sqrt(midZUpTranslation.dot(footStepToLookAtTranslation)) < NO_LOOKY_RADIUS)
      {
         return false;
      }
      return true;
   }

   private void lookAtStep(FootstepData currentFootStepData)
   {
      lookAtBehavior.setLookAtLocation(currentFootStepData.getLocation());
   }

   private void sendFootStepToAtlas(LinkedList<FootstepData> footStepsToTake)
   {
      outgoingFootstepDataList.footstepDataList.addAll(footStepsToTake);
      footStepsToTake.clear();

      sendPacketToController(currentFootStepList);
   }

   private void processSnapFootstepResponse(FootstepData currentFootStepData)
   {
      footStepToTake.add(currentFootStepData);
   }

   private void processSnapFootstepResponse(SnapFootstepPacket snapFootstepPacket)
   {
      switch (snapFootstepPacket.getFlag()[0])
      {
      case 0://retchToLook(snapFootstepPacket.getFootstepData());
      case 1://SnapFootstepPacket.VALID_UNCHANGED_STEP:
      case 2://SnapFootstepPacket.VALID_SNAPPED_STEP:
            footStepToTake.addAll(snapFootstepPacket.getFootstepData());
      case 3://SnapFootstepPacket.BAD_STEP:
            System.out.println("need to replan!");
      }
   }

   private void stretchToLook(ArrayList<FootstepData> footsteps)
   {
      System.out.println("Not Implemented, just taking the step");
      footStepToTake.addAll(footsteps);
   }

   public void setNewFootSteps(FootstepDataList footStepDataList)
   {
      currentFootStepList = footStepDataList;
      footstepListBehavior.pause();
      footStepsToLookAt.clear();
      footStepToTake.clear();
      footStepsToLookAt.addAll(currentFootStepList.getDataList());
      haveInputsBeenSet.set(true);
   }

   @Override
   protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
   {

   }

   @Override
   protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {
      lookAtBehavior.consumeObjectFromController(object);
      footstepListBehavior.consumeObjectFromController(object);
   }

   @Override
   public void stop()
   {
      footstepListBehavior.stop();
      lookAtBehavior.stop();
   }

   @Override
   public void enableActions()
   {
      footstepListBehavior.enableActions();
      lookAtBehavior.enableActions();
   }

   @Override
   public void pause()
   {
      footstepListBehavior.pause();
      lookAtBehavior.pause();
   }

   @Override
   public void resume()
   {
      footstepListBehavior.resume();
      lookAtBehavior.resume();
   }

   @Override
   public boolean isDone()
   {
      isDone.set(footStepToTake.isEmpty() && footStepsToLookAt.isEmpty() && hasInputBeenSet());
      return isDone.getBooleanValue();
   }

   @Override
   public void doPostBehaviorCleanup()
   {
      footstepListBehavior.doPostBehaviorCleanup();
      lookAtBehavior.doPostBehaviorCleanup();
   }

   @Override
   public void initialize()
   {
      footstepListBehavior.initialize();
      lookAtBehavior.initialize();
   }

   @Override
   public boolean hasInputBeenSet()
   {
      return haveInputsBeenSet.getBooleanValue();
   }
}
