package us.ihmc.humanoidBehaviors.behaviors;

import java.util.ArrayList;
import java.util.LinkedHashMap;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.packets.behaviors.DebrisData;
import us.ihmc.communication.packets.behaviors.HumanoidBehaviorDebrisPacket;
import us.ihmc.humanoidBehaviors.behaviors.midLevel.RemoveSingleDebrisBehavior;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.utilities.WristForceSensorFilteredUpdatable;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class RemoveMultipleDebrisBehavior extends BehaviorInterface
{
   private final RemoveSingleDebrisBehavior removePieceOfDebrisBehavior;

   private final ConcurrentListeningQueue<HumanoidBehaviorDebrisPacket> inputListeningQueue = new ConcurrentListeningQueue<HumanoidBehaviorDebrisPacket>();
   private final BooleanYoVariable isDone;
   private final BooleanYoVariable haveInputsBeenSet;
   private final FullRobotModel fullRobotModel;

   private final ArrayList<DebrisData> debrisDataList = new ArrayList<>();
   private final ArrayList<DebrisData> sortedDebrisDataList = new ArrayList<>();
   private final LinkedHashMap<DebrisData, Double> debrisDistanceMap = new LinkedHashMap<>();

   private double currentDistanceToObject;
   private final FramePoint currentObjectPosition = new FramePoint();
   
   private final SideDependentList<WristForceSensorFilteredUpdatable> wristSensors;

   public RemoveMultipleDebrisBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, FullRobotModel fullRobotModel, ReferenceFrames referenceFrame,
         SideDependentList<WristForceSensorFilteredUpdatable> wristSensors, DoubleYoVariable yoTime, WalkingControllerParameters walkingControllerParameters)
   {
      super(outgoingCommunicationBridge);
      removePieceOfDebrisBehavior = new RemoveSingleDebrisBehavior(outgoingCommunicationBridge, fullRobotModel, referenceFrame, yoTime, walkingControllerParameters);
      isDone = new BooleanYoVariable("isDone", registry);
      haveInputsBeenSet = new BooleanYoVariable("hasInputsBeenSet", registry);

      this.fullRobotModel = fullRobotModel;
      this.attachNetworkProcessorListeningQueue(inputListeningQueue, HumanoidBehaviorDebrisPacket.class);

      this.wristSensors = wristSensors;
   }

   @Override
   public void doControl()
   {
      if (!isDone.getBooleanValue())
         checkForNewInputs();
      if (removePieceOfDebrisBehavior.isDone())
      {
         removePieceOfDebrisBehavior.finalize();
         sortedDebrisDataList.remove(0);
         if (sortedDebrisDataList.isEmpty())
         {
            isDone.set(true);
            return;
         }
         removePieceOfDebrisBehavior.initialize();
         removePieceOfDebrisBehavior.setInputs(sortedDebrisDataList.get(0).getTransform(), sortedDebrisDataList.get(0).getPosition(), sortedDebrisDataList.get(0).getVector());
      }
      removePieceOfDebrisBehavior.doControl();
   }

   private void checkForNewInputs()
   {
      HumanoidBehaviorDebrisPacket newestPacket = inputListeningQueue.getNewestPacket();
      if (newestPacket != null)
      {
         debrisDataList.addAll(newestPacket.getDebrisDataList());
         sortDebrisFromCloserToFarther();
         removePieceOfDebrisBehavior.initialize();
         removePieceOfDebrisBehavior.setInputs(sortedDebrisDataList.get(0).getTransform(), sortedDebrisDataList.get(0).getPosition(), sortedDebrisDataList.get(0).getVector());
         haveInputsBeenSet.set(true);
      }
   }

   private void sortDebrisFromCloserToFarther()
   {
      for (int i = 0; i < debrisDataList.size(); i++)
      {

         DebrisData currentDebrisData = debrisDataList.get(i);
         currentObjectPosition.changeFrame(ReferenceFrame.getWorldFrame());
         currentObjectPosition.set(currentDebrisData.getPosition());
         currentObjectPosition.changeFrame(fullRobotModel.getChest().getBodyFixedFrame());
         currentDistanceToObject = currentObjectPosition.getX();

         debrisDistanceMap.put(currentDebrisData, currentDistanceToObject);

      }

      sortedDebrisDataList.clear();
      sortedDebrisDataList.add(debrisDataList.get(0));
      for (int i = 1; i < debrisDataList.size(); i++)
      {
         currentDistanceToObject = debrisDistanceMap.get(debrisDataList.get(i));
         int j = 0;
         while (j < sortedDebrisDataList.size() && currentDistanceToObject > debrisDistanceMap.get(sortedDebrisDataList.get(j)))
         {
            j++;
         }
         if (j == sortedDebrisDataList.size())
            sortedDebrisDataList.add(debrisDataList.get(i));
         else
            sortedDebrisDataList.add(j, debrisDataList.get(i));
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
      sortedDebrisDataList.clear();
      isDone.set(false);
      haveInputsBeenSet.set(false);
   }

   @Override
   public void initialize()
   {
      debrisDataList.clear();
      sortedDebrisDataList.clear();
      isDone.set(false);
      haveInputsBeenSet.set(false);

   }

   @Override
   public boolean hasInputBeenSet()
   {
      return haveInputsBeenSet.getBooleanValue();
   }

}
