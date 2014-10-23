package us.ihmc.humanoidBehaviors.behaviors;

import java.util.ArrayList;

import us.ihmc.communication.packets.behaviors.DebrisData;
import us.ihmc.communication.packets.behaviors.HumanoidBehaviorDebrisPacket;
import us.ihmc.humanoidBehaviors.behaviors.midLevel.RemovePieceOfDebrisBehavior;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class RemoveDebrisBehavior extends BehaviorInterface
{
   private final RemovePieceOfDebrisBehavior removePieceOfDebrisBehavior;

   private final ConcurrentListeningQueue<HumanoidBehaviorDebrisPacket> inputListeningQueue = new ConcurrentListeningQueue<HumanoidBehaviorDebrisPacket>();
   private final BooleanYoVariable isDone;
   private final BooleanYoVariable haveInputsBeenSet;

   private final ArrayList<DebrisData> debrisDataList = new ArrayList<>();

   public RemoveDebrisBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, FullRobotModel fullRobotModel, ReferenceFrames referenceFrame,
         DoubleYoVariable yoTime)
   {
      super(outgoingCommunicationBridge);

      removePieceOfDebrisBehavior = new RemovePieceOfDebrisBehavior(outgoingCommunicationBridge, fullRobotModel, referenceFrame, yoTime);
      isDone = new BooleanYoVariable("isDone", registry);
      haveInputsBeenSet = new BooleanYoVariable("hasInputsBeenSet", registry);

      this.attachNetworkProcessorListeningQueue(inputListeningQueue, HumanoidBehaviorDebrisPacket.class);
   }

   @Override
   public void doControl()
   {
      if (!isDone.getBooleanValue())
         checkForNewInputs();
      if (removePieceOfDebrisBehavior.isDone())
      {
         removePieceOfDebrisBehavior.finalize();
         debrisDataList.remove(0);
         if (debrisDataList.isEmpty())
         {
            isDone.set(true);
            return;
         }
         removePieceOfDebrisBehavior.initialize();
         removePieceOfDebrisBehavior.setInputs(debrisDataList.get(0).getTransform(), debrisDataList.get(0).getPosition(), debrisDataList.get(0).getVector());
      }
      removePieceOfDebrisBehavior.doControl();
   }

   private void checkForNewInputs()
   {
      HumanoidBehaviorDebrisPacket newestPacket = inputListeningQueue.getNewestPacket();
      if (newestPacket != null)
      {        
         debrisDataList.addAll(newestPacket.getDebrisDataList());
         removePieceOfDebrisBehavior.initialize();
         removePieceOfDebrisBehavior.setInputs(debrisDataList.get(0).getTransform(), debrisDataList.get(0).getPosition(), debrisDataList.get(0).getVector());
         haveInputsBeenSet.set(true);
      }
   }

   @Override
   protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
   {
      if (removePieceOfDebrisBehavior != null)
         removePieceOfDebrisBehavior.consumeObjectFromNetworkProcessor(object);
   }

   @Override
   protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {
      if (removePieceOfDebrisBehavior != null)
         removePieceOfDebrisBehavior.consumeObjectFromController(object);
   }

   @Override
   public void stop()
   {
      removePieceOfDebrisBehavior.stop();
   }

   @Override
   public void enableActions()
   {
      removePieceOfDebrisBehavior.enableActions();
   }

   @Override
   public void pause()
   {
      removePieceOfDebrisBehavior.pause();
   }

   @Override
   public void resume()
   {
      removePieceOfDebrisBehavior.resume();
   }

   @Override
   public boolean isDone()
   {
      return isDone.getBooleanValue();
   }

   @Override
   public void finalize()
   {
      removePieceOfDebrisBehavior.finalize();
      debrisDataList.clear();
      isDone.set(false);
      haveInputsBeenSet.set(false);
   }

   @Override
   public void initialize()
   {
      debrisDataList.clear();
      isDone.set(false);
      haveInputsBeenSet.set(false);

   }

   @Override
   public boolean hasInputBeenSet()
   {
      return haveInputsBeenSet.getBooleanValue();
   }

}
