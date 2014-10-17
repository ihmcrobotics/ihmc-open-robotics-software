package us.ihmc.humanoidBehaviors.behaviors.midLevel;

import java.util.ArrayList;

import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class RemovePieceOfDebrisBehavior extends BehaviorInterface
{
   private final ArrayList<BehaviorInterface> behaviors = new ArrayList<BehaviorInterface>();

   private final ConcurrentListeningQueue<Packet> inputListeningQueue = new ConcurrentListeningQueue<Packet>();
   private final GraspObjectBehavior graspObject;
   private final DropDebrisBehavior dropMic;
   private final BooleanYoVariable isDone;
   private final BooleanYoVariable haveInputsBeenSet;
   private BehaviorInterface currentBehavior;

   public RemovePieceOfDebrisBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, FullRobotModel fullRobotModel,
         ReferenceFrames referenceFrame, DoubleYoVariable yoTime)
   {
      super(outgoingCommunicationBridge);

      graspObject = new GraspObjectBehavior(outgoingCommunicationBridge);
      dropMic = new DropDebrisBehavior(outgoingCommunicationBridge, fullRobotModel, yoTime);

      isDone = new BooleanYoVariable("isDone", registry);
      haveInputsBeenSet = new BooleanYoVariable("hasInputsBeenSet", registry);

      this.attachNetworkProcessorListeningQueue(inputListeningQueue, Packet.class);
   }

   @Override
   public void doControl()
   {
      if (!isDone.getBooleanValue())
      {
         checkForNewInputs();
         if (currentBehavior != null)
         {
            checkTransitionCondition();
            currentBehavior.doControl();
         }
      }
   }

   private void checkTransitionCondition()
   {
      if (currentBehavior.isDone())
      {
         currentBehavior.finalize();
         currentBehavior = behaviors.remove(0);
         if (currentBehavior != null)
         {
            currentBehavior.initialize();
         }
         else
         {
            isDone.set(true);
         }
      }
   }

   private void checkForNewInputs()
   {
      if (inputListeningQueue.isNewPacketAvailable())
      {
         //this.graspPose = inputListeningQueue.getNewestPacket().getPose;
         graspObject.setGraspPose(null);
         haveInputsBeenSet.set(true);
      }
   }

   public void setInputs(RigidBodyTransform graspPose)
   {
      graspObject.setGraspPose(graspPose);
      haveInputsBeenSet.set(true);
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
      isDone.set(false);
      currentBehavior = null;
      haveInputsBeenSet.set(false);
   }

   @Override
   public void initialize()
   {
      behaviors.clear();
      isDone.set(false);
      graspObject.initialize();
      dropMic.initialize();
      behaviors.add(graspObject);
      behaviors.add(dropMic);
      currentBehavior = behaviors.remove(0);
      haveInputsBeenSet.set(false);

   }

   @Override
   public boolean hasInputBeenSet()
   {
      return haveInputsBeenSet.getBooleanValue();
   }

}
