package us.ihmc.humanoidBehaviors.behaviors.roughTerrain;

import us.ihmc.communication.packets.*;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlannerGoalType;
import us.ihmc.footstepPlanning.SimpleFootstep;
import us.ihmc.footstepPlanning.graphSearch.BipedalFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.PlanarRegionBipedalFootstepPlannerVisualizer;
import us.ihmc.footstepPlanning.graphSearch.SimplePlanarRegionBipedalAnytimeFootstepPlanner;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.behaviorServices.FiducialDetectorBehaviorService;
import us.ihmc.humanoidBehaviors.behaviors.behaviorServices.ObjectDetectorBehaviorService;
import us.ihmc.humanoidBehaviors.behaviors.goalLocation.GoalDetectorBehaviorService;
import us.ihmc.humanoidBehaviors.behaviors.goalLocation.LocateGoalBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SimpleDoNothingBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SleepBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessageConverter;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachines.StateTransitionCondition;
import us.ihmc.robotics.time.YoTimer;
import us.ihmc.tools.FormattingTools;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;

import java.util.ArrayList;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Tuple3d;

public class AnytimePlannerStateMachineBehavior extends StateMachineBehavior<AnytimePlannerStateMachineBehavior.AnytimePlanningState>
{
   private final String prefix = getClass().getSimpleName();
   private static final GoalDetectorType GOAL_DETECTOR_TYPE = GoalDetectorType.FIDUCIAL;

   private enum GoalDetectorType
   {
      FIDUCIAL, VALVE;
   }

   private final DoubleYoVariable yoTime;
   private final IntegerYoVariable maxNumberOfStepsToTake = new IntegerYoVariable(prefix + "NumberOfStepsToTake", registry);
   private final IntegerYoVariable indexOfNextFootstepToSendFromCurrentPlan = new IntegerYoVariable(prefix + "NextFootstepToSendFromCurrentPlan", registry);
   
   private final BipedalFootstepPlannerParameters footstepPlanningParameters;
   private final SimplePlanarRegionBipedalAnytimeFootstepPlanner footstepPlanner;

   private final BooleanYoVariable reachedGoal = new BooleanYoVariable(prefix + "ReachedGoal", registry);
   private final DoubleYoVariable swingTime = new DoubleYoVariable(prefix + "SwingTime", registry);
   private final DoubleYoVariable transferTime = new DoubleYoVariable(prefix + "TransferTime", registry);
   private final BooleanYoVariable receivedFootstepCompletedPacked = new BooleanYoVariable(prefix + "ReceivedFootstepCompletedPacket", registry);
   private final HumanoidReferenceFrames referenceFrames;
   private final BooleanYoVariable havePlanarRegionsBeenSet = new BooleanYoVariable(prefix + "HavePlanarRegionsBeenSet", registry);
   private FootstepPlan currentPlan;
   private final IntegerYoVariable numberOfFootstepsInCurrentPlan = new IntegerYoVariable(prefix + "NumberOfFootstepsInCurrentPlan", registry);

   private final LocateGoalBehavior locateGoalBehavior;
   private final RequestAndWaitForPlanarRegionsListBehavior requestAndWaitForPlanarRegionsListBehavior;
   private final SleepBehavior sleepBehavior;
   private final CheckForBestPlanBehavior checkForBestPlanBehavior;
   private final SendOverFootstepAndWaitForCompletionBehavior sendOverFootstepsAndUpdatePlannerBehavior;
   private final SimpleDoNothingBehavior reachedGoalBehavior;

   private final NoValidPlanCondition noValidPlanCondition = new NoValidPlanCondition();
   private final FoundAPlanCondition foundAPlanCondition = new FoundAPlanCondition();
   private final ContinueWalkingAfterCompletedStepCondition continueWalkingAfterCompletedStepCondition = new ContinueWalkingAfterCompletedStepCondition();
   private final ReachedGoalAfterCompletedStepCondition reachedGoalAfterCompletedStepCondition = new ReachedGoalAfterCompletedStepCondition();

   private final ConcurrentListeningQueue<PlanarRegionsListMessage> planarRegionsListQueue = new ConcurrentListeningQueue<>(10);
   private final ConcurrentListeningQueue<FootstepStatus> footstepStatusQueue = new ConcurrentListeningQueue<FootstepStatus>(10);

   public enum AnytimePlanningState
   {
      LOCATE_GOAL, REQUEST_AND_WAIT_FOR_PLANAR_REGIONS, SLEEP, CHECK_FOR_BEST_PLAN, SEND_OVER_FOOTSTEP_AND_WAIT_FOR_COMPLETION, REACHED_GOAL
   }

   public AnytimePlannerStateMachineBehavior(CommunicationBridge communicationBridge, DoubleYoVariable yoTime, HumanoidReferenceFrames referenceFrames,
                                             LogModelProvider logModelProvider, FullHumanoidRobotModel fullRobotModel, WholeBodyControllerParameters wholeBodyControllerParameters)
   {
      super("AnytimePlanner", AnytimePlanningState.class, yoTime, communicationBridge);
      
      footstepPlanningParameters = new BipedalFootstepPlannerParameters(registry);
      FootstepPlannerForBehaviorsHelper.setPlannerParametersForAnytimePlannerAndPlannerToolbox(footstepPlanningParameters);
      footstepPlanner = new SimplePlanarRegionBipedalAnytimeFootstepPlanner(footstepPlanningParameters, registry);
      SideDependentList<ConvexPolygon2d> footPolygonsInSoleFrame = FootstepPlannerForBehaviorsHelper.createDefaultFootPolygonsForAnytimePlannerAndPlannerToolbox(wholeBodyControllerParameters.getContactPointParameters());
      footstepPlanner.setFeetPolygons(footPolygonsInSoleFrame);
      footstepPlanner.setMaximumNumberOfNodesToExpand(500);

      this.yoTime = yoTime;
      maxNumberOfStepsToTake.set(1);
      this.referenceFrames = referenceFrames;

      GoalDetectorBehaviorService goalDetectorBehaviorService;
      switch(GOAL_DETECTOR_TYPE)
      {
      case FIDUCIAL:
         FiducialDetectorBehaviorService fiducialDetectorBehaviorService = new FiducialDetectorBehaviorService(communicationBridge, null);
         fiducialDetectorBehaviorService.setTargetIDToLocate(50);
         fiducialDetectorBehaviorService.setExpectedFiducialSize(0.50);
         goalDetectorBehaviorService = fiducialDetectorBehaviorService;
         break;
      case VALVE:
         try
         {
            goalDetectorBehaviorService = new ObjectDetectorBehaviorService(communicationBridge, null);
            break;
         }
         catch(Exception e)
         {
            e.printStackTrace();
            System.err.println("Cannot create valve detector service!");
         }
      default:
         throw new RuntimeException("Cannot create detector " + GOAL_DETECTOR_TYPE);
      }

      locateGoalBehavior = new LocateGoalBehavior(communicationBridge, goalDetectorBehaviorService);
      requestAndWaitForPlanarRegionsListBehavior = new RequestAndWaitForPlanarRegionsListBehavior(communicationBridge);
      sleepBehavior = new SleepBehavior(communicationBridge, yoTime);
      checkForBestPlanBehavior = new CheckForBestPlanBehavior(communicationBridge);
      sendOverFootstepsAndUpdatePlannerBehavior = new SendOverFootstepAndWaitForCompletionBehavior(communicationBridge);
      reachedGoalBehavior = new SimpleDoNothingBehavior(communicationBridge)
      {
         @Override
         public boolean isDone()
         {
            return false;
         }
      };

      createAndAttachYoVariableServerListenerToPlanner(logModelProvider, fullRobotModel);

      swingTime.set(1.5);
      transferTime.set(0.3);

      this.registry.addChild(requestAndWaitForPlanarRegionsListBehavior.getYoVariableRegistry());
      this.registry.addChild(sleepBehavior.getYoVariableRegistry());
      this.registry.addChild(checkForBestPlanBehavior.getYoVariableRegistry());
      this.registry.addChild(sendOverFootstepsAndUpdatePlannerBehavior.getYoVariableRegistry());
      //      this.registry.addChild(reachedGoalBehavior.getYoVariableRegistry());

      attachNetworkListeningQueue(footstepStatusQueue, FootstepStatus.class);
      attachNetworkListeningQueue(planarRegionsListQueue, PlanarRegionsListMessage.class);

      setupStateMachine();
   }

   private void setupStateMachine()
   {
      BehaviorAction<AnytimePlanningState> locateGoalAction = new BehaviorAction<AnytimePlanningState>(
            AnytimePlanningState.LOCATE_GOAL, locateGoalBehavior)
      {
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = new TextToSpeechPacket("Starting to locate goal");
            sendPacket(p1);
         }

         @Override
         public void doTransitionOutOfAction()
         {
            FramePose goalPose = new FramePose();
            locateGoalBehavior.getReportedGoalPoseWorldFrame(goalPose);
            setGoalPose(goalPose);
            super.doTransitionOutOfAction();

            sendPacketToUI(new UIPositionCheckerPacket(goalPose.getFramePointCopy().getPoint(), goalPose.getFrameOrientationCopy().getQuaternion()));

            PrintTools.info("got the goal: \n" + goalPose + " \n requesting planar regions...");
         }
      };

      BehaviorAction<AnytimePlanningState> requestPlanarRegionsAction = new BehaviorAction<AnytimePlanningState>(
            AnytimePlanningState.REQUEST_AND_WAIT_FOR_PLANAR_REGIONS, requestAndWaitForPlanarRegionsListBehavior)
      {
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = new TextToSpeechPacket("Requesting planar regions");
            sendPacket(p1);
         }
      };

      BehaviorAction<AnytimePlanningState> sleepAction = new BehaviorAction<AnytimePlanningState>(AnytimePlanningState.SLEEP, sleepBehavior)
      {
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = new TextToSpeechPacket("Received planar regions, giving planner time to think...");
            sendPacket(p1);
         }
      };

      BehaviorAction<AnytimePlanningState> checkForBestPlan = new BehaviorAction<AnytimePlanningState>(AnytimePlanningState.CHECK_FOR_BEST_PLAN,
                                                                                                       checkForBestPlanBehavior);

      BehaviorAction<AnytimePlanningState> sendFootstepAndWaitForCompletion = new BehaviorAction<AnytimePlanningState>(
            AnytimePlanningState.SEND_OVER_FOOTSTEP_AND_WAIT_FOR_COMPLETION, sendOverFootstepsAndUpdatePlannerBehavior);

      BehaviorAction<AnytimePlanningState> reachedGoalAction = new BehaviorAction<AnytimePlanningState>(AnytimePlanningState.REACHED_GOAL, reachedGoalBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = new TextToSpeechPacket("Reached Goal!");
            sendPacket(p1);
         }
      };

      statemachine.addStateWithDoneTransition(locateGoalAction, AnytimePlanningState.REQUEST_AND_WAIT_FOR_PLANAR_REGIONS);
      statemachine.addStateWithDoneTransition(requestPlanarRegionsAction, AnytimePlanningState.SLEEP);
      statemachine.addStateWithDoneTransition(sleepAction, AnytimePlanningState.CHECK_FOR_BEST_PLAN);
      statemachine.addState(checkForBestPlan);
      statemachine.addState(sendFootstepAndWaitForCompletion);
      statemachine.addState(reachedGoalAction);

      checkForBestPlan.addStateTransition(AnytimePlanningState.REQUEST_AND_WAIT_FOR_PLANAR_REGIONS, noValidPlanCondition);
      checkForBestPlan.addStateTransition(AnytimePlanningState.SEND_OVER_FOOTSTEP_AND_WAIT_FOR_COMPLETION, foundAPlanCondition);

      sendFootstepAndWaitForCompletion.addStateTransition(AnytimePlanningState.CHECK_FOR_BEST_PLAN, continueWalkingAfterCompletedStepCondition);
      sendFootstepAndWaitForCompletion.addStateTransition(AnytimePlanningState.REACHED_GOAL, reachedGoalAfterCompletedStepCondition);

      statemachine.setStartState(AnytimePlanningState.LOCATE_GOAL);
   }

   @Override
   public void onBehaviorEntered()
   {
      super.onBehaviorEntered();
      new Thread(footstepPlanner).start();
   }

   public void createAndAttachYoVariableServerListenerToPlanner(LogModelProvider logModelProvider, FullRobotModel fullRobotModel)
   {
      PlanarRegionBipedalFootstepPlannerVisualizer listener = PlanarRegionBipedalFootstepPlannerVisualizerFactory
            .createWithYoVariableServer(0.01, fullRobotModel, logModelProvider, footstepPlanner.getFootPolygonsInSoleFrame(), "Anytime_");

      footstepPlanner.setBipedalFootstepPlannerListener(listener);
   }


   @Override
   public void onBehaviorExited()
   {
      footstepPlanner.requestStop();
   }

   public void setGoalPose(FramePose goalPose)
   {
      Tuple3d goalPosition = new Point3d();
      goalPose.getPosition(goalPosition);

      String xString = FormattingTools.getFormattedToSignificantFigures(goalPosition.getX(), 3);
      String yString = FormattingTools.getFormattedToSignificantFigures(goalPosition.getY(), 3);

      FootstepPlannerGoal footstepPlannerGoal = new FootstepPlannerGoal();
      footstepPlannerGoal.setGoalPoseBetweenFeet(goalPose);

      Point2d xyGoal = new Point2d();
      xyGoal.setX(goalPose.getX());
      xyGoal.setY(goalPose.getY());
      double distanceFromXYGoal = 1.0;
      footstepPlannerGoal.setXYGoal(xyGoal, distanceFromXYGoal);
      footstepPlannerGoal.setFootstepPlannerGoalType(FootstepPlannerGoalType.CLOSE_TO_XY_POSITION);

      FramePose leftFootPose = new FramePose();
      leftFootPose.setToZero(referenceFrames.getSoleFrame(RobotSide.LEFT));
      leftFootPose.changeFrame(ReferenceFrame.getWorldFrame());
      sendPacketToUI(new UIPositionCheckerPacket(new Point3d(xyGoal.getX(), xyGoal.getY(), leftFootPose.getZ()), new Quat4d()));

      footstepPlanner.setGoal(footstepPlannerGoal);
      footstepPlanner.setInitialStanceFoot(leftFootPose, RobotSide.LEFT);
   }

   private class RequestAndWaitForPlanarRegionsListBehavior extends AbstractBehavior
   {
      private final BooleanYoVariable receivedPlanarRegionsList = new BooleanYoVariable(prefix + "ReceivedPlanarRegionsList", registry);
      private final YoTimer requestNewPlanarRegionsTimer = new YoTimer(yoTime);
      private final DoubleYoVariable planarRegionsResponseTimeout = new DoubleYoVariable(prefix + "PlanarRegionsResponseTimeout", registry);
      private PlanarRegionsList planarRegionsList;

      public RequestAndWaitForPlanarRegionsListBehavior(CommunicationBridgeInterface communicationBridge)
      {
         super(communicationBridge);
         planarRegionsResponseTimeout.set(2.0);
      }

      @Override
      public void doControl()
      {
         if (planarRegionsListQueue.isNewPacketAvailable())
         {
            PlanarRegionsListMessage planarRegionsListMessage = planarRegionsListQueue.getLatestPacket();
            planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage);
            footstepPlanner.setPlanarRegions(planarRegionsList);
            receivedPlanarRegionsList.set(true);
            havePlanarRegionsBeenSet.set(true);
         }
         else if (requestNewPlanarRegionsTimer.totalElapsed() > planarRegionsResponseTimeout.getDoubleValue())
         {
            requestPlanarRegions();
            requestNewPlanarRegionsTimer.reset();
         }
      }

      @Override
      public void onBehaviorEntered()
      {
         receivedPlanarRegionsList.set(false);
         requestNewPlanarRegionsTimer.reset();
         requestPlanarRegions();
      }

      @Override
      public void onBehaviorAborted()
      {
      }

      @Override
      public void onBehaviorPaused()
      {
      }

      @Override
      public void onBehaviorResumed()
      {
      }

      @Override
      public void onBehaviorExited()
      {
      }

      @Override
      public boolean isDone()
      {
         return receivedPlanarRegionsList.getBooleanValue();
      }
   }

   private class CheckForBestPlanBehavior extends AbstractBehavior
   {
      public CheckForBestPlanBehavior(CommunicationBridgeInterface communicationBridge)
      {
         super(communicationBridge);
      }

      @Override
      public void doControl()
      {
         FootstepPlan latestPlan = footstepPlanner.getBestPlanYet();

         if (latestPlan != null)
         {
            currentPlan = latestPlan;
            indexOfNextFootstepToSendFromCurrentPlan.set(0);
            numberOfFootstepsInCurrentPlan.set(currentPlan.getNumberOfSteps());
         }

         if (currentPlan != null && currentPlan.getNumberOfSteps() == indexOfNextFootstepToSendFromCurrentPlan.getIntegerValue() + 1)
         {
            currentPlan = null;
            indexOfNextFootstepToSendFromCurrentPlan.set(0);
            numberOfFootstepsInCurrentPlan.set(0);
         }
      }

      @Override
      public void onBehaviorEntered()
      {

      }

      @Override
      public void onBehaviorAborted()
      {

      }

      @Override
      public void onBehaviorPaused()
      {

      }

      @Override
      public void onBehaviorResumed()
      {

      }

      @Override
      public void onBehaviorExited()
      {

      }

      @Override
      public boolean isDone()
      {
         return false;
      }
   }

   private class SendOverFootstepAndWaitForCompletionBehavior extends AbstractBehavior
   {
      private final FramePose tempFirstFootstepPose = new FramePose();
      private final Point3d tempFootstepPosePosition = new Point3d();
      private final Quat4d tempFirstFootstepPoseOrientation = new Quat4d();
      private final EnumYoVariable<FootstepStatus.Status> latestFootstepStatus = new EnumYoVariable<>(prefix + "LatestFootstepStatus", registry,
                                                                                                      FootstepStatus.Status.class);

      public SendOverFootstepAndWaitForCompletionBehavior(CommunicationBridgeInterface communicationBridge)
      {
         super(communicationBridge);
      }

      @Override
      public void doControl()
      {
         if (footstepStatusQueue.isNewPacketAvailable())
         {
            FootstepStatus footstepStatus = footstepStatusQueue.getLatestPacket();
            latestFootstepStatus.set(footstepStatus.getStatus());

            if (footstepStatus.getStatus() == FootstepStatus.Status.COMPLETED)
            {
               receivedFootstepCompletedPacked.set(true);
            }
         }
      }

      @Override
      public void onBehaviorEntered()
      {
         // clear footstep status queue
         footstepStatusQueue.clear();

         // set newest planar regions
         PlanarRegionsListMessage planarRegionsListMessage = planarRegionsListQueue.getLatestPacket();
         PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage);
         if (planarRegionsList != null)
            footstepPlanner.setPlanarRegions(planarRegionsList);

         // notify planner footstep is being taken
         PrintTools.info("current plan size: " + currentPlan.getNumberOfSteps() + ", current footstep: " + indexOfNextFootstepToSendFromCurrentPlan
               .getIntegerValue());
         SimpleFootstep latestFootstep = currentPlan.getFootstep(1 + indexOfNextFootstepToSendFromCurrentPlan.getIntegerValue());
         footstepPlanner.executingFootstep(latestFootstep);

         // send over footstep
         FootstepDataListMessage footstepDataListMessage = createFootstepDataListFromPlan(currentPlan,
                                                                                          indexOfNextFootstepToSendFromCurrentPlan.getIntegerValue(), 1,
                                                                                          swingTime.getDoubleValue(), transferTime.getDoubleValue());
         indexOfNextFootstepToSendFromCurrentPlan.increment();

         FootstepDataListMessage footstepDataListMessageFull = FootstepDataMessageConverter.createFootstepDataListFromPlan(currentPlan, 0.0, 0.0, ExecutionMode.OVERRIDE);
         footstepDataListMessageFull.setDestination(PacketDestination.UI);
         sendPacket(footstepDataListMessageFull);

         footstepDataListMessage.setDestination(PacketDestination.CONTROLLER);
         sendPacketToController(footstepDataListMessage);
         receivedFootstepCompletedPacked.set(false);

         // request new planar regions
         requestPlanarRegions();

         // notify UI
         TextToSpeechPacket p1 = new TextToSpeechPacket(
               "Sending over footstep, " + indexOfNextFootstepToSendFromCurrentPlan.getIntegerValue() + " of current plan");
         sendPacket(p1);
      }

      @Override
      public void onBehaviorAborted()
      {

      }

      @Override
      public void onBehaviorPaused()
      {

      }

      @Override
      public void onBehaviorResumed()
      {

      }

      @Override
      public void onBehaviorExited()
      {

      }

      @Override
      public boolean isDone()
      {
         return false;
      }

      private FootstepDataListMessage createFootstepDataListFromPlan(FootstepPlan plan, int startIndex, int maxNumberOfStepsToTake, double swingTime,
                                                                     double transferTime)
      {
         FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
         footstepDataListMessage.setSwingTime(swingTime);
         footstepDataListMessage.setTransferTime(transferTime);
         int numSteps = plan.getNumberOfSteps();
         int lastStepIndex = Math.min(startIndex + maxNumberOfStepsToTake + 1, numSteps);
         for (int i = 1 + startIndex; i < lastStepIndex; i++)
         {
            SimpleFootstep footstep = plan.getFootstep(i);
            footstep.getSoleFramePose(tempFirstFootstepPose);
            tempFirstFootstepPose.getPosition(tempFootstepPosePosition);
            tempFirstFootstepPose.getOrientation(tempFirstFootstepPoseOrientation);

            FootstepDataMessage firstFootstepMessage = new FootstepDataMessage(footstep.getRobotSide(), new Point3d(tempFootstepPosePosition),
                                                                               new Quat4d(tempFirstFootstepPoseOrientation));
            firstFootstepMessage.setOrigin(FootstepDataMessage.FootstepOrigin.AT_SOLE_FRAME);
            System.out.println("sending footstep of side " + footstep.getRobotSide());

            footstepDataListMessage.add(firstFootstepMessage);
         }

         footstepDataListMessage.setExecutionMode(ExecutionMode.OVERRIDE);
         return footstepDataListMessage;
      }
   }

   private void requestPlanarRegions()
   {
      RequestPlanarRegionsListMessage requestPlanarRegionsListMessage = new RequestPlanarRegionsListMessage(
            RequestPlanarRegionsListMessage.RequestType.SINGLE_UPDATE);
      requestPlanarRegionsListMessage.setDestination(PacketDestination.REA_MODULE);
      sendPacket(requestPlanarRegionsListMessage);
      sendPacket(requestPlanarRegionsListMessage);
   }

   private class FoundAPlanCondition implements StateTransitionCondition
   {
      @Override
      public boolean checkCondition()
      {
         return currentPlan != null;
      }
   }

   private class NoValidPlanCondition implements StateTransitionCondition
   {
      @Override
      public boolean checkCondition()
      {
         return currentPlan == null || currentPlan.getNumberOfSteps() <= 1;
      }
   }

   private class ContinueWalkingAfterCompletedStepCondition implements StateTransitionCondition
   {
      @Override
      public boolean checkCondition()
      {
         return receivedFootstepCompletedPacked.getBooleanValue() && !reachedGoal.getBooleanValue();
      }
   }

   private class ReachedGoalAfterCompletedStepCondition implements StateTransitionCondition
   {
      @Override
      public boolean checkCondition()
      {
         return receivedFootstepCompletedPacked.getBooleanValue() && reachedGoal.getBooleanValue();
      }
   }
}
