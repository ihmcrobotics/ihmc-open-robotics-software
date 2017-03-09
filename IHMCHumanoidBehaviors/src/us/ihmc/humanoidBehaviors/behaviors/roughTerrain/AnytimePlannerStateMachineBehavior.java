package us.ihmc.humanoidBehaviors.behaviors.roughTerrain;

import java.util.Random;

import us.ihmc.commonWalkingControlModules.trajectories.SwingOverPlanarRegionsTrajectoryExpander;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.PlanarRegionsListMessage;
import us.ihmc.communication.packets.RequestPlanarRegionsListMessage;
import us.ihmc.communication.packets.RequestPlanarRegionsListMessage.RequestType;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.communication.packets.UIPositionCheckerPacket;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlannerGoalType;
import us.ihmc.footstepPlanning.SimpleFootstep;
import us.ihmc.footstepPlanning.graphSearch.BipedalFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.PlanarRegionBipedalFootstepPlannerVisualizer;
import us.ihmc.footstepPlanning.graphSearch.SimplePlanarRegionBipedalAnytimeFootstepPlanner;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
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
import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataClearCommand;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataClearCommand.DepthDataTree;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage.FootstepOrigin;
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
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.robotics.time.YoStopwatch;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;

public class AnytimePlannerStateMachineBehavior extends StateMachineBehavior<AnytimePlannerStateMachineBehavior.AnytimePlanningState>
{
   private final String prefix = getClass().getSimpleName();
   private static final GoalDetectorType GOAL_DETECTOR_TYPE = GoalDetectorType.FIDUCIAL;

   private enum GoalDetectorType
   {
      FIDUCIAL, VALVE, HARD_CODED;
   }

   private final DoubleYoVariable yoTime;
   private final IntegerYoVariable maxNumberOfStepsToTake = new IntegerYoVariable(prefix + "NumberOfStepsToTake", registry);
   private final IntegerYoVariable indexOfNextFootstepToSendFromCurrentPlan = new IntegerYoVariable(prefix + "NextFootstepToSendFromCurrentPlan", registry);

   private final BipedalFootstepPlannerParameters footstepPlanningParameters;
   private final SimplePlanarRegionBipedalAnytimeFootstepPlanner footstepPlanner;
   private final SwingOverPlanarRegionsTrajectoryExpander swingOverPlanarRegionsTrajectoryExpander;

   private final BooleanYoVariable reachedGoal = new BooleanYoVariable(prefix + "ReachedGoal", registry);
   private final BooleanYoVariable clearedLidar = new BooleanYoVariable(prefix + "ClearedLidar", registry);
   private final DoubleYoVariable reachedGoalThreshold = new DoubleYoVariable(prefix + "ReachedGoalThreshold", registry);
   private final DoubleYoVariable swingTime = new DoubleYoVariable(prefix + "SwingTime", registry);
   private final DoubleYoVariable transferTime = new DoubleYoVariable(prefix + "TransferTime", registry);
   private final BooleanYoVariable receivedFootstepCompletedPacket = new BooleanYoVariable(prefix + "ReceivedFootstepCompletedPacket", registry);
   private final HumanoidReferenceFrames referenceFrames;
   private final BooleanYoVariable havePlanarRegionsBeenSet = new BooleanYoVariable(prefix + "HavePlanarRegionsBeenSet", registry);
   private FootstepPlan currentPlan;
   private final IntegerYoVariable numberOfFootstepsInCurrentPlan = new IntegerYoVariable(prefix + "NumberOfFootstepsInCurrentPlan", registry);

   private final LocateGoalBehavior locateGoalBehavior;
   private final RequestAndWaitForPlanarRegionsListBehavior requestAndWaitForPlanarRegionsListBehavior;
   private final SleepBehavior sleepBehavior;
   private final CheckForBestPlanBehavior checkForBestPlanBehavior;
   private final SendOverFootstepAndWaitForCompletionBehavior sendOverFootstepsAndUpdatePlannerBehavior;
   private final SquareUpBehavior squareUpBehavior;
   private final SimpleDoNothingBehavior reachedGoalBehavior;

   private final NoValidPlanCondition noValidPlanCondition = new NoValidPlanCondition();
   private final FoundAPlanCondition foundAPlanCondition = new FoundAPlanCondition();
   private final ClearedLidarCondition clearedLidarCondition = new ClearedLidarCondition();
   private final ContinueWalkingAfterCompletedStepCondition continueWalkingAfterCompletedStepCondition = new ContinueWalkingAfterCompletedStepCondition();
   private final ReachedGoalAfterCompletedStepCondition reachedGoalAfterCompletedStepCondition = new ReachedGoalAfterCompletedStepCondition();

   private final ConcurrentListeningQueue<PlanarRegionsListMessage> planarRegionsListQueue = new ConcurrentListeningQueue<>(10);
   private final ConcurrentListeningQueue<FootstepStatus> footstepStatusQueue = new ConcurrentListeningQueue<FootstepStatus>(10);
   private SimpleFootstep lastFootstepSentForExecution;
   
   private final IntegerYoVariable stepCounterForClearingLidar = new IntegerYoVariable("StepCounterForClearingLidar", registry);
   private final IntegerYoVariable stepsBeforeClearingLidar = new IntegerYoVariable("StepsBeforeClearingLidar", registry);

   public enum AnytimePlanningState
   {
      LOCATE_GOAL, REQUEST_AND_WAIT_FOR_PLANAR_REGIONS, SLEEP, CHECK_FOR_BEST_PLAN, SEND_OVER_FOOTSTEP_AND_WAIT_FOR_COMPLETION, SQUARE_UP, REACHED_GOAL
   }

   public AnytimePlannerStateMachineBehavior(CommunicationBridge communicationBridge, DoubleYoVariable yoTime, HumanoidReferenceFrames referenceFrames,
                                             LogModelProvider logModelProvider, FullHumanoidRobotModel fullRobotModel,
                                             WholeBodyControllerParameters wholeBodyControllerParameters, YoGraphicsListRegistry yoGraphicsListRegistry,
                                             GoalDetectorBehaviorService goalDetectorBehaviorService, boolean createYoVariableServerForPlannerVisualizer)
   {
      super("AnytimePlanner", AnytimePlanningState.class, yoTime, communicationBridge);

      reachedGoalThreshold.set(0.5);

      footstepPlanningParameters = new BipedalFootstepPlannerParameters(registry);
      FootstepPlannerForBehaviorsHelper.setPlannerParametersForAnytimePlannerAndPlannerToolbox(footstepPlanningParameters);
      footstepPlanner = new SimplePlanarRegionBipedalAnytimeFootstepPlanner(footstepPlanningParameters, registry);
      RobotContactPointParameters contactPointParameters = wholeBodyControllerParameters.getContactPointParameters();
      SideDependentList<ConvexPolygon2d> footPolygonsInSoleFrame = FootstepPlannerForBehaviorsHelper.createDefaultFootPolygonsForAnytimePlannerAndPlannerToolbox(contactPointParameters);
      SideDependentList<ConvexPolygon2d> controlPolygonsInSoleFrame = FootstepPlannerForBehaviorsHelper.createDefaultFootPolygons(contactPointParameters, 1.0, 1.0);
      footstepPlanner.setFeetPolygons(footPolygonsInSoleFrame, controlPolygonsInSoleFrame);
      swingOverPlanarRegionsTrajectoryExpander = new SwingOverPlanarRegionsTrajectoryExpander(wholeBodyControllerParameters.getWalkingControllerParameters(),
                                                                                              registry, yoGraphicsListRegistry);

      this.yoTime = yoTime;
      maxNumberOfStepsToTake.set(1);
      this.referenceFrames = referenceFrames;

      locateGoalBehavior = new LocateGoalBehavior(communicationBridge, goalDetectorBehaviorService);
      requestAndWaitForPlanarRegionsListBehavior = new RequestAndWaitForPlanarRegionsListBehavior(communicationBridge);
      sleepBehavior = new ResettingSleepBehavior(communicationBridge, yoTime, 2.0);
            
      checkForBestPlanBehavior = new CheckForBestPlanBehavior(communicationBridge);
      sendOverFootstepsAndUpdatePlannerBehavior = new SendOverFootstepAndWaitForCompletionBehavior(communicationBridge);
      squareUpBehavior = new SquareUpBehavior(communicationBridge);
      reachedGoalBehavior = new SimpleDoNothingBehavior(communicationBridge)
      {
         @Override
         public boolean isDone()
         {
            return false;
         }
      };

      if(createYoVariableServerForPlannerVisualizer)
         createAndAttachYoVariableServerListenerToPlanner(logModelProvider, fullRobotModel);

      swingTime.set(0.5);
      transferTime.set(0.3);
      stepsBeforeClearingLidar.set(5);

      this.registry.addChild(requestAndWaitForPlanarRegionsListBehavior.getYoVariableRegistry());
      this.registry.addChild(sleepBehavior.getYoVariableRegistry());
      this.registry.addChild(checkForBestPlanBehavior.getYoVariableRegistry());
      this.registry.addChild(sendOverFootstepsAndUpdatePlannerBehavior.getYoVariableRegistry());

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

      BehaviorAction<AnytimePlanningState> squareUp = new BehaviorAction<AnytimePlanningState>(AnytimePlanningState.SQUARE_UP, squareUpBehavior);

      BehaviorAction<AnytimePlanningState> reachedGoalAction = new BehaviorAction<AnytimePlanningState>(AnytimePlanningState.REACHED_GOAL, reachedGoalBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            int randomInt = new Random().nextInt(5);
            switch (randomInt)
            {
            case 0:
               sendPacket(new TextToSpeechPacket("I am done. Do you want me to do this again?"));
               break;
            case 1:
               sendPacket(new TextToSpeechPacket("What is my next task?"));
               break;
            case 2:
               sendPacket(new TextToSpeechPacket("Can I crush those cinder block now human master?"));
               break;
            case 3:
               sendPacket(new TextToSpeechPacket("Urgh - I bet they make me do this again."));
               break;
            case 4:
               sendPacket(new TextToSpeechPacket("Can I go to the bathroom now please?"));
               break;
            default:
               sendPacket(new TextToSpeechPacket("Done with requested behavior."));
               break;
            }
         }
      };

      statemachine.addStateWithDoneTransition(locateGoalAction, AnytimePlanningState.REQUEST_AND_WAIT_FOR_PLANAR_REGIONS);
      statemachine.addStateWithDoneTransition(requestPlanarRegionsAction, AnytimePlanningState.SLEEP);
      statemachine.addStateWithDoneTransition(sleepAction, AnytimePlanningState.CHECK_FOR_BEST_PLAN);
      statemachine.addState(checkForBestPlan);
      statemachine.addState(sendFootstepAndWaitForCompletion);
      statemachine.addStateWithDoneTransition(squareUp, AnytimePlanningState.REACHED_GOAL);
      statemachine.addState(reachedGoalAction);

      checkForBestPlan.addStateTransition(AnytimePlanningState.REQUEST_AND_WAIT_FOR_PLANAR_REGIONS, clearedLidarCondition);
      checkForBestPlan.addStateTransition(AnytimePlanningState.REQUEST_AND_WAIT_FOR_PLANAR_REGIONS, noValidPlanCondition);
      checkForBestPlan.addStateTransition(AnytimePlanningState.SEND_OVER_FOOTSTEP_AND_WAIT_FOR_COMPLETION, foundAPlanCondition);

      sendFootstepAndWaitForCompletion.addStateTransition(AnytimePlanningState.CHECK_FOR_BEST_PLAN, continueWalkingAfterCompletedStepCondition);
      sendFootstepAndWaitForCompletion.addStateTransition(AnytimePlanningState.SQUARE_UP, reachedGoalAfterCompletedStepCondition);

      statemachine.setStartState(AnytimePlanningState.LOCATE_GOAL);
   }

   @Override
   public void onBehaviorEntered()
   {
      super.onBehaviorEntered();
      reachedGoal.set(false);
      stepCounterForClearingLidar.set(0);
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

   private final FramePoint2d goalPosition = new FramePoint2d();

   public void setGoalPose(FramePose goalPose)
   {
      goalPose.getPosition2dIncludingFrame(this.goalPosition);
      Tuple3DBasics goalPosition = new Point3D();
      goalPose.getPosition(goalPosition);

      FootstepPlannerGoal footstepPlannerGoal = new FootstepPlannerGoal();
      footstepPlannerGoal.setGoalPoseBetweenFeet(goalPose);

      Point2D xyGoal = new Point2D();
      xyGoal.setX(goalPose.getX());
      xyGoal.setY(goalPose.getY());
      double distanceFromXYGoal = 1.0;
      footstepPlannerGoal.setXYGoal(xyGoal, distanceFromXYGoal);
      footstepPlannerGoal.setFootstepPlannerGoalType(FootstepPlannerGoalType.CLOSE_TO_XY_POSITION);

      FramePose leftFootPose = new FramePose();
      leftFootPose.setToZero(referenceFrames.getSoleFrame(RobotSide.LEFT));
      leftFootPose.changeFrame(ReferenceFrame.getWorldFrame());
      sendPacketToUI(new UIPositionCheckerPacket(new Point3D(xyGoal.getX(), xyGoal.getY(), leftFootPose.getZ()), new Quaternion()));

      footstepPlanner.setGoal(footstepPlannerGoal);
      footstepPlanner.setInitialStanceFoot(leftFootPose, RobotSide.LEFT);
   }

   private class RequestAndWaitForPlanarRegionsListBehavior extends AbstractBehavior
   {
      private final BooleanYoVariable receivedPlanarRegionsList = new BooleanYoVariable(prefix + "ReceivedPlanarRegionsList", registry);
      private final YoStopwatch requestNewPlanarRegionsTimer = new YoStopwatch(yoTime);
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

   private class ResettingSleepBehavior extends SleepBehavior
   {
      private final double sleepTime;

      public ResettingSleepBehavior(CommunicationBridgeInterface outgoingCommunicationBridge, DoubleYoVariable yoTime, double sleepTime)
      {
         super(outgoingCommunicationBridge, yoTime, sleepTime);
         this.sleepTime = sleepTime;
      }

      @Override
      public void onBehaviorExited()
      {
         super.onBehaviorExited();
         this.setSleepTime(sleepTime);
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
         if (!footstepPlanner.isClear())
         {
            return;
         }

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
      private final Point3D tempFootstepPosePosition = new Point3D();
      private final Quaternion tempFirstFootstepPoseOrientation = new Quaternion();
      private final FramePose stanceFootPose = new FramePose();
      private final FramePose swingStartPose = new FramePose();
      private final FramePose swingEndPose = new FramePose();
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
               receivedFootstepCompletedPacket.set(true);
            }
         }

         ReferenceFrame midFeetZUpFrame = referenceFrames.getMidFeetZUpFrame();
         FramePoint2d goalPositionInMidFeetZUp = goalPosition.changeFrameAndProjectToXYPlaneCopy(midFeetZUpFrame);
         reachedGoal.set(goalPositionInMidFeetZUp.distanceFromZero() < reachedGoalThreshold.getDoubleValue());
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
         FootstepDataListMessage footstepDataListMessage = createFootstepDataListFromPlanOverPlanarRegions(currentPlan,
                                                                                          indexOfNextFootstepToSendFromCurrentPlan.getIntegerValue(), 1,
                                                                                          swingTime.getDoubleValue(), transferTime.getDoubleValue(), planarRegionsList);
         indexOfNextFootstepToSendFromCurrentPlan.increment();

         FootstepDataListMessage footstepDataListMessageFull = FootstepDataMessageConverter.createFootstepDataListFromPlan(currentPlan, 0.0, 0.0, ExecutionMode.OVERRIDE);
         footstepDataListMessageFull.setDestination(PacketDestination.UI);
         sendPacket(footstepDataListMessageFull);

         footstepDataListMessage.setDestination(PacketDestination.CONTROLLER);
         sendPacketToController(footstepDataListMessage);
         receivedFootstepCompletedPacket.set(false);

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
         stepCounterForClearingLidar.increment();
         
         if (stepCounterForClearingLidar.getIntegerValue() == stepsBeforeClearingLidar.getIntegerValue())
         {
            DepthDataClearCommand clearLidarPacket = new DepthDataClearCommand(DepthDataTree.DECAY_POINT_CLOUD);
            clearLidarPacket.setDestination(PacketDestination.NETWORK_PROCESSOR);
            sendPacket(clearLidarPacket);

            RequestPlanarRegionsListMessage requestPlanarRegionsListMessage = new RequestPlanarRegionsListMessage(RequestType.CLEAR);
            requestPlanarRegionsListMessage.setDestination(PacketDestination.REA_MODULE);
            sendPacket(requestPlanarRegionsListMessage);

            currentPlan = null;
            
            footstepPlanner.clear();
            sleepBehavior.setSleepTime(5.0);
            clearedLidar.set(true);
            stepCounterForClearingLidar.set(0);
         }
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
         footstepDataListMessage.setDefaultSwingDuration(swingTime);
         footstepDataListMessage.setDefaultTransferDuration(transferTime);
         int numSteps = plan.getNumberOfSteps();
         int lastStepIndex = Math.min(startIndex + maxNumberOfStepsToTake + 1, numSteps);
         for (int i = 1 + startIndex; i < lastStepIndex; i++)
         {
            SimpleFootstep footstep = plan.getFootstep(i);
            footstep.getSoleFramePose(tempFirstFootstepPose);
            tempFirstFootstepPose.getPosition(tempFootstepPosePosition);
            tempFirstFootstepPose.getOrientation(tempFirstFootstepPoseOrientation);

            FootstepDataMessage firstFootstepMessage = new FootstepDataMessage(footstep.getRobotSide(), new Point3D(tempFootstepPosePosition),
                                                                               new Quaternion(tempFirstFootstepPoseOrientation));
            firstFootstepMessage.setOrigin(FootstepDataMessage.FootstepOrigin.AT_SOLE_FRAME);
            System.out.println("sending footstep of side " + footstep.getRobotSide());

            footstepDataListMessage.add(firstFootstepMessage);

            lastFootstepSentForExecution = new SimpleFootstep(footstep.getRobotSide(), tempFirstFootstepPose);
         }

         footstepDataListMessage.setExecutionMode(ExecutionMode.OVERRIDE);
         return footstepDataListMessage;
      }
      
      private FootstepDataListMessage createFootstepDataListFromPlanOverPlanarRegions(FootstepPlan plan, int startIndex, int maxNumberOfStepsToTake, double swingTime,
                                                                     double transferTime, PlanarRegionsList planarRegionsList)
      {
         FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
         footstepDataListMessage.setDefaultSwingDuration(swingTime);
         footstepDataListMessage.setDefaultTransferDuration(transferTime);
         int numSteps = plan.getNumberOfSteps();
         int lastStepIndex = Math.min(startIndex + maxNumberOfStepsToTake + 1, numSteps);
         
         swingStartPose.setToZero(referenceFrames.getSoleFrame(plan.getFootstep(1 + startIndex).getRobotSide()));
         stanceFootPose.setToZero(referenceFrames.getSoleFrame(plan.getFootstep(1 + startIndex).getRobotSide().getOppositeSide()));
         
         for (int i = 1 + startIndex; i < lastStepIndex; i++)
         {
            SimpleFootstep footstep = plan.getFootstep(i);
            footstep.getSoleFramePose(swingEndPose);

            FootstepDataMessage firstFootstepMessage = new FootstepDataMessage(footstep.getRobotSide(), new Point3D(swingEndPose.getPosition()),
                                                                               new Quaternion(swingEndPose.getOrientation()));
            firstFootstepMessage.setOrigin(FootstepOrigin.AT_SOLE_FRAME);

            swingOverPlanarRegionsTrajectoryExpander.expandTrajectoryOverPlanarRegions(stanceFootPose, swingStartPose, swingEndPose, planarRegionsList);

            firstFootstepMessage.setTrajectoryType(TrajectoryType.CUSTOM);
            Point3D waypointOne = new Point3D();
            Point3D waypointTwo = new Point3D();
            swingOverPlanarRegionsTrajectoryExpander.getExpandedWaypoints().get(0).get(waypointOne);
            swingOverPlanarRegionsTrajectoryExpander.getExpandedWaypoints().get(1).get(waypointTwo);
            firstFootstepMessage.setTrajectoryWaypoints(new Point3D[] {waypointOne, waypointTwo});
            System.out.println("sending footstep of side " + footstep.getRobotSide());

            footstepDataListMessage.add(firstFootstepMessage);

            swingStartPose.setIncludingFrame(stanceFootPose);
            stanceFootPose.setIncludingFrame(swingEndPose);
            
            lastFootstepSentForExecution = new SimpleFootstep(footstep.getRobotSide(), swingEndPose);
         }

         footstepDataListMessage.setExecutionMode(ExecutionMode.OVERRIDE);
         return footstepDataListMessage;
      }
   }

   private class SquareUpBehavior extends AbstractBehavior
   {
      private boolean isDone = false;

      public SquareUpBehavior(CommunicationBridgeInterface communicationBridge)
      {
         super(communicationBridge);
      }

      @Override
      public void doControl()
      {
         if (footstepStatusQueue.isNewPacketAvailable())
         {
            FootstepStatus footstepStatus = footstepStatusQueue.getLatestPacket();
            if (footstepStatus.getStatus() == FootstepStatus.Status.COMPLETED)
               isDone = true;
         }
      }

      @Override
      public void onBehaviorEntered()
      {
         // clear footstep status queue
         footstepStatusQueue.clear();
         isDone = false;

         // assemble square up step
         RobotSide stanceSide = lastFootstepSentForExecution.getRobotSide();
         double width = footstepPlanningParameters.getIdealFootstepWidth();
         ReferenceFrame stanceFootFrame = referenceFrames.getSoleFrame(stanceSide);
         FramePose stepPose = new FramePose(stanceFootFrame);
         stepPose.setY(stanceSide == RobotSide.LEFT ? -width : width);
         stepPose.changeFrame(ReferenceFrame.getWorldFrame());

         // make footstep data message
         FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
         footstepDataListMessage.setDefaultSwingDuration(swingTime.getDoubleValue());
         footstepDataListMessage.setDefaultTransferDuration(transferTime.getDoubleValue());
         Point3D position = new Point3D();
         Quaternion orientation = new Quaternion();
         stepPose.getPosition(position);
         stepPose.getOrientation(orientation);
         FootstepDataMessage firstFootstepMessage = new FootstepDataMessage(stanceSide.getOppositeSide(), position, orientation);
         firstFootstepMessage.setOrigin(FootstepDataMessage.FootstepOrigin.AT_SOLE_FRAME);
         footstepDataListMessage.add(firstFootstepMessage);
         footstepDataListMessage.setExecutionMode(ExecutionMode.OVERRIDE);

         // send it to the controller
         footstepDataListMessage.setDestination(PacketDestination.CONTROLLER);
         sendPacketToController(footstepDataListMessage);

         // notify UI
         TextToSpeechPacket p1 = new TextToSpeechPacket("Reached goal! Squaring up.");
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
         return isDone;
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

   private class ClearedLidarCondition implements StateTransitionCondition
   {
      @Override
      public boolean checkCondition()
      {
         boolean isTrueIf = clearedLidar.getBooleanValue();

         if (isTrueIf)
         {
            clearedLidar.set(false);
         }

         return isTrueIf;
      }
   }

   private class ContinueWalkingAfterCompletedStepCondition implements StateTransitionCondition
   {
      @Override
      public boolean checkCondition()
      {
         return receivedFootstepCompletedPacket.getBooleanValue() && !reachedGoal.getBooleanValue();
      }
   }

   private class ReachedGoalAfterCompletedStepCondition implements StateTransitionCondition
   {
      @Override
      public boolean checkCondition()
      {
         return receivedFootstepCompletedPacket.getBooleanValue() && reachedGoal.getBooleanValue();
      }
   }
}
