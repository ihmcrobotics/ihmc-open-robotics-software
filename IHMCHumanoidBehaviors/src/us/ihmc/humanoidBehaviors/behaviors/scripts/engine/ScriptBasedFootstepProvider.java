package us.ihmc.humanoidBehaviors.behaviors.scripts.engine;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.concurrent.ConcurrentLinkedQueue;

import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PauseWalkingMessage;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;

public class ScriptBasedFootstepProvider implements Updatable
{
   private int footstepCounter = 0;
   private int completedFootstepCount = 0;

   private final SideDependentList<? extends ContactablePlaneBody> bipedFeet;
   private final ScriptFileLoader scriptFileLoader;
   private boolean loadedScriptFile = false;
   private final ConcurrentLinkedQueue<ScriptObject> scriptObjects = new ConcurrentLinkedQueue<ScriptObject>();

   private final ConcurrentLinkedQueue<Footstep> footstepQueue = new ConcurrentLinkedQueue<Footstep>();

   private final DoubleYoVariable time;
   private final DoubleYoVariable scriptEventStartTime, scriptEventDuration;

   private final FullRobotModel fullRobotModel;

   public ScriptBasedFootstepProvider(CommonHumanoidReferenceFrames referenceFrames, ScriptFileLoader scriptFileLoader, DoubleYoVariable time, SideDependentList<? extends ContactablePlaneBody> bipedFeet,
         FullHumanoidRobotModel fullRobotModel, WalkingControllerParameters walkingControllerParameters, YoVariableRegistry registry)
   {
      this.time = time;
      this.bipedFeet = bipedFeet;
      this.fullRobotModel = fullRobotModel;

      this.scriptEventStartTime = new DoubleYoVariable("scriptEventStartTime", registry);
      this.scriptEventDuration = new DoubleYoVariable("scriptEventDuration", registry);

      this.scriptFileLoader = scriptFileLoader;

   }

   private void loadScriptFileIfNecessary()
   {
      FramePoint soleInRootJointFrame = new FramePoint(bipedFeet.get(RobotSide.LEFT).getSoleFrame());
      soleInRootJointFrame.changeFrame(fullRobotModel.getRootJoint().getFrameAfterJoint());
      if (loadedScriptFile)
         return;

      // TODO: Get to work for more than just left foot frame.
      ReferenceFrame leftFootFrame = bipedFeet.get(RobotSide.LEFT).getSoleFrame();
      RigidBodyTransform transformFromLeftFootPlaneFrameToWorldFrame = leftFootFrame.getTransformToDesiredFrame(ReferenceFrame.getWorldFrame());

      ArrayList<ScriptObject> scriptObjectsList = scriptFileLoader.readIntoList(transformFromLeftFootPlaneFrameToWorldFrame);
      scriptObjects.addAll(scriptObjectsList);
      setupTimesForNewScriptEvent(0.1);

      loadedScriptFile = true;
   }

   public void grabNewScriptEventIfNecessary()
   {
      loadScriptFileIfNecessary();

      if (scriptObjects.isEmpty())
         return;
      if (!footstepQueue.isEmpty())
         return;
      if (completedFootstepCount != footstepCounter)
         return;
      if (time.getDoubleValue() < scriptEventStartTime.getDoubleValue() + scriptEventDuration.getDoubleValue())
         return;

      ScriptObject nextObject = scriptObjects.poll();
      Object scriptObject = nextObject.getScriptObject();

      if (scriptObject instanceof FootstepDataListMessage)
      {
         FootstepDataListMessage footstepDataList = (FootstepDataListMessage) scriptObject;
         this.addFootstepDataList(footstepDataList);
         setupTimesForNewScriptEvent(0.5); // Arbitrary half second duration. With footsteps, it waits till they are done before looking for a new command.
      }
//      else if (scriptObject instanceof FootTrajectoryMessage)
//      {
//         FootTrajectoryMessage message = (FootTrajectoryMessage) scriptObject;
//         footTrajectoryMessageSubscriber.receivedPacket(message);
//         setupTimesForNewScriptEvent(0.5);
//      }
//      else if (scriptObject instanceof HandTrajectoryMessage)
//      {
//         HandTrajectoryMessage handTrajectoryMessage = (HandTrajectoryMessage) scriptObject;
//         handTrajectoryMessageSubscriber.receivedPacket(handTrajectoryMessage);
//
//         setupTimesForNewScriptEvent(handTrajectoryMessage.getLastTrajectoryPoint().time);
//      }
//      else if (scriptObject instanceof ArmTrajectoryMessage)
//      {
//         ArmTrajectoryMessage armTrajectoryMessage = (ArmTrajectoryMessage) scriptObject;
//         armTrajectoryMessageSubscriber.receivedPacket(armTrajectoryMessage);
//         
//         setupTimesForNewScriptEvent(armTrajectoryMessage.getTrajectoryTime());
//      }
//      else if (scriptObject instanceof PelvisTrajectoryMessage)
//      {
//         PelvisTrajectoryMessage pelvisPosePacket = (PelvisTrajectoryMessage) scriptObject;
//         pelvisTrajectoryMessageSubscriber.receivedPacket(pelvisPosePacket);
//
//         setupTimesForNewScriptEvent(pelvisPosePacket.getTrajectoryTime());
//      }
      else if (scriptObject instanceof PauseWalkingMessage)
      {
         PauseWalkingMessage pauseCommand = (PauseWalkingMessage) scriptObject;
         setupTimesForNewScriptEvent(0.5);
      }

//      else if (scriptObject instanceof PelvisHeightTrajectoryMessage)
//      {
//         PelvisHeightTrajectoryMessage comHeightPacket = (PelvisHeightTrajectoryMessage) scriptObject;
//         pelvisHeightTrajectoryMessageSubscriber.receivedPacket(comHeightPacket);
//         setupTimesForNewScriptEvent(2.0); // Arbitrary two second duration to allow for changing the CoM height. Might be possible to lower this a little bit. 
//      }
   }

   private void setupTimesForNewScriptEvent(double scriptEventDuration)
   {
      scriptEventStartTime.set(time.getDoubleValue());
      this.scriptEventDuration.set(scriptEventDuration);
   }

   private void addFootstepDataList(FootstepDataListMessage footstepDataList)
   {
      ArrayList<FootstepDataMessage> footstepList = footstepDataList.getDataList();

      ArrayList<Footstep> footsteps = new ArrayList<Footstep>();
      for (FootstepDataMessage footstepData : footstepList)
      {
         RobotSide robotSide = footstepData.getRobotSide();
         ContactablePlaneBody contactableBody = bipedFeet.get(robotSide);

         FramePose footstepPose = new FramePose(ReferenceFrame.getWorldFrame(), footstepData.getLocation(), footstepData.getOrientation());
         PoseReferenceFrame footstepPoseFrame = new PoseReferenceFrame("footstepPoseFrame", footstepPose);
         String id = "scriptedFootstep_" + footstepCounter;
         Footstep footstep = new Footstep(id, contactableBody.getRigidBody(), robotSide, contactableBody.getSoleFrame(), footstepPoseFrame, true);

         footsteps.add(footstep);

         footstepCounter++;
      }

      footstepQueue.addAll(footsteps);
   }

   public Footstep poll()
   {
      return footstepQueue.poll();
   }

   public Footstep peek()
   {
      return footstepQueue.peek();
   }

   public Footstep peekPeek()
   {
      Iterator<Footstep> iterator = footstepQueue.iterator();

      if (iterator.hasNext())
      {
         iterator.next();
      }
      else
      {
         return null;
      }
      if (iterator.hasNext())
      {
         return iterator.next();
      }
      else
      {
         return null;
      }
   }

   public boolean isEmpty()
   {
      return footstepQueue.isEmpty();
   }

   public void notifyComplete(FramePose actualFootPoseInWorld)
   {
      completedFootstepCount++;
   }

   public void notifyWalkingComplete()
   {
   }

   public int getNumberOfFootstepsToProvide()
   {
      return footstepQueue.size();
   }

   public boolean isBlindWalking()
   {
      return false;
   }

   @Override
   public void update(double time)
   {
      grabNewScriptEventIfNecessary();
   }

   public boolean isPaused()
   {
      return false;
   }

   public void cancelPlan()
   {
      footstepQueue.clear();
   }
}
