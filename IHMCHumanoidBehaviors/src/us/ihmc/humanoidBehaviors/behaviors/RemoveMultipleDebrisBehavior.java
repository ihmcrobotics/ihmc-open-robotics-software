package us.ihmc.humanoidBehaviors.behaviors;

import java.util.ArrayList;
import java.util.TreeMap;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.communication.packets.behaviors.DebrisData;
import us.ihmc.communication.packets.behaviors.HumanoidBehaviorDebrisPacket;
import us.ihmc.humanoidBehaviors.behaviors.midLevel.RemoveSingleDebrisBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.taskExecutor.HandPoseTask;
import us.ihmc.humanoidBehaviors.taskExecutor.RemovePieceOfDebrisTask;
import us.ihmc.humanoidBehaviors.utilities.WristForceSensorFilteredUpdatable;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.utilities.taskExecutor.PipeLine;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.wholeBodyController.parameters.DefaultArmConfigurations.ArmConfigurations;

public class RemoveMultipleDebrisBehavior extends BehaviorInterface
{
   private final PipeLine<BehaviorInterface> pipeLine = new PipeLine<>();

   private final HandPoseBehavior handPoseBehavior;
   private final RemoveSingleDebrisBehavior removePieceOfDebrisBehavior;

   private final ConcurrentListeningQueue<HumanoidBehaviorDebrisPacket> inputListeningQueue = new ConcurrentListeningQueue<HumanoidBehaviorDebrisPacket>();
   private final BooleanYoVariable isDone;
   private final BooleanYoVariable haveInputsBeenSet;
   private final SDFFullRobotModel fullRobotModel;

   private final ArrayList<DebrisData> debrisDataList = new ArrayList<>();
   private final ArrayList<DebrisData> sortedDebrisDataList = new ArrayList<>();
   private final TreeMap<Double, DebrisData> debrisDistanceMap = new TreeMap<>();

   private final DoubleYoVariable yoTime;
   private double trajectoryTime = 2.5;

   private final WholeBodyControllerParameters wholeBodyControllerParameters;
   private final SideDependentList<WristForceSensorFilteredUpdatable> wristSensors;

   public RemoveMultipleDebrisBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, SDFFullHumanoidRobotModel fullRobotModel,
         HumanoidReferenceFrames referenceFrame, SideDependentList<WristForceSensorFilteredUpdatable> wristSensors, DoubleYoVariable yoTime,
         WholeBodyControllerParameters wholeBodyControllerParameters)
   {
      super(outgoingCommunicationBridge);
      handPoseBehavior = new HandPoseBehavior(outgoingCommunicationBridge, yoTime);
      removePieceOfDebrisBehavior = new RemoveSingleDebrisBehavior(outgoingCommunicationBridge, fullRobotModel, referenceFrame, yoTime,
            wholeBodyControllerParameters);
      isDone = new BooleanYoVariable("isDone", registry);
      haveInputsBeenSet = new BooleanYoVariable("haveInputsBeenSet", registry);

      this.fullRobotModel = fullRobotModel;
      this.attachNetworkProcessorListeningQueue(inputListeningQueue, HumanoidBehaviorDebrisPacket.class);

      this.wristSensors = wristSensors;
      this.yoTime = yoTime;
      this.wholeBodyControllerParameters = wholeBodyControllerParameters;
   }

   @Override
   public void doControl()
   {
      if (!isDone.getBooleanValue() && !haveInputsBeenSet.getBooleanValue())
         checkForNewInputs();
      if (haveInputsBeenSet.getBooleanValue())
         pipeLine.doControl();
   }

   private void checkForNewInputs()
   {
      HumanoidBehaviorDebrisPacket newestPacket = inputListeningQueue.getNewestPacket();
      if (newestPacket != null)
      {
         debrisDataList.addAll(newestPacket.getDebrisDataList());
         setInputs(debrisDataList);
      }
   }

   public void setInputs(ArrayList<DebrisData> debrisDataList)
   {
      sortDebrisFromCloserToFarther(debrisDataList);
      submitArmsSafePosition();
      for (int i = 0; i < sortedDebrisDataList.size(); i++)
         submitRemoveDebrisTasks(sortedDebrisDataList.get(i));
      haveInputsBeenSet.set(true);
   }

   private void submitArmsSafePosition()
   {
      RobotSide robotSide = RobotSide.LEFT;
      double[] desiredArmJointAngles = wholeBodyControllerParameters.getDefaultArmConfigurations().getArmDefaultConfigurationJointAngles(
            ArmConfigurations.COMPACT_HOME, robotSide);

      pipeLine.submitTaskForPallelPipesStage(handPoseBehavior, new HandPoseTask(robotSide, desiredArmJointAngles, yoTime, handPoseBehavior, trajectoryTime));

      robotSide = RobotSide.RIGHT;
      desiredArmJointAngles = wholeBodyControllerParameters.getDefaultArmConfigurations().getArmDefaultConfigurationJointAngles(ArmConfigurations.COMPACT_HOME,
            robotSide);

      pipeLine.submitTaskForPallelPipesStage(handPoseBehavior, new HandPoseTask(robotSide, desiredArmJointAngles, yoTime, handPoseBehavior, trajectoryTime));

   }

   private void submitRemoveDebrisTasks(DebrisData debrisData)
   {
      pipeLine.submitSingleTaskStage(new RemovePieceOfDebrisTask(removePieceOfDebrisBehavior, debrisData.getDebrisTransform(), debrisData
            .getGraspVectorPosition(), debrisData.getGraspVector(), yoTime));
   }

   private void sortDebrisFromCloserToFarther(ArrayList<DebrisData> debrisDataListToBeSorted)
   {
      double currentDistanceToObject;
      FramePoint currentObjectPosition = new FramePoint();

      for (int i = 0; i < debrisDataListToBeSorted.size(); i++)
      {
         DebrisData currentDebrisData = debrisDataListToBeSorted.get(i);
         currentObjectPosition.changeFrame(ReferenceFrame.getWorldFrame());
         currentObjectPosition.set(currentDebrisData.getGraspVectorPosition());
         currentObjectPosition.changeFrame(fullRobotModel.getChest().getBodyFixedFrame());
         currentDistanceToObject = currentObjectPosition.getX();

         debrisDistanceMap.put(currentDistanceToObject, currentDebrisData);
      }

      sortedDebrisDataList.clear();
      sortedDebrisDataList.addAll(debrisDistanceMap.values());
   }

   @Override
   protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
   {
      handPoseBehavior.consumeObjectFromNetworkProcessor(object);
      removePieceOfDebrisBehavior.consumeObjectFromNetworkProcessor(object);
   }

   @Override
   protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {
      handPoseBehavior.consumeObjectFromController(object);
      removePieceOfDebrisBehavior.consumeObjectFromController(object);
   }

   @Override
   public void stop()
   {
      handPoseBehavior.stop();
      removePieceOfDebrisBehavior.stop();
   }
   
   @Override
   public void enableActions()
   {
      handPoseBehavior.enableActions();
      removePieceOfDebrisBehavior.enableActions();
   }
   
   @Override
   public void pause()
   {
      handPoseBehavior.pause();
      removePieceOfDebrisBehavior.pause();
   }

   @Override
   public void resume()
   {
      handPoseBehavior.resume();
      removePieceOfDebrisBehavior.resume();
   }

   @Override
   public boolean isDone()
   {
      if (pipeLine.isDone() && hasInputBeenSet())
         isDone.set(true);
      return isDone.getBooleanValue();
   }

   @Override
   public void doPostBehaviorCleanup()
   {
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
