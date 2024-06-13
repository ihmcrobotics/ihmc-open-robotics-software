package us.ihmc.behaviors.activeMapping;

import behavior_msgs.msg.dds.ContinuousWalkingCommandMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.QueuedFootstepStatusMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.MonteCarloFootstepPlannerParameters;
import us.ihmc.footstepPlanning.PlannedFootstep;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.footstepPlanning.monteCarloPlanning.MonteCarloFootstepPlanner;
import us.ihmc.footstepPlanning.monteCarloPlanning.MonteCarloFootstepPlannerRequest;
import us.ihmc.footstepPlanning.monteCarloPlanning.MonteCarloPlannerTools;
import us.ihmc.footstepPlanning.monteCarloPlanning.TerrainPlanningDebugger;
import us.ihmc.footstepPlanning.swing.CollisionFreeSwingCalculator;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerType;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.robotics.math.trajectories.interfaces.PolynomialReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class ContinuousPlanner
{
   public enum PlanningMode
   {
      EXECUTE_AND_PAUSE, FRONTIER_EXPANSION, ACTIVE_SEARCH, FAST_HIKING, WALK_TO_GOAL
   }

   private ContinuousGoalGenerator goalGenerator = new ContinuousGoalGenerator(0.0, 5.0, 0.0, 5.0);
   private final SideDependentList<FramePose3D> startingStancePose = new SideDependentList<>(new FramePose3D(), new FramePose3D());
   private final SideDependentList<FramePose3D> goalStancePose = new SideDependentList<>(new FramePose3D(), new FramePose3D());
   private final HumanoidReferenceFrames referenceFrames;

   private AtomicReference<FootstepPlan> monteCarloFootstepPlan = new AtomicReference<>(null);
   private FramePose3D walkingStartMidPose = new FramePose3D();
   private FramePose3D imminentFootstepPose = new FramePose3D();
   private RobotSide imminentFootstepSide = RobotSide.LEFT;
   private ContinuousWalkingCommandMessage command;
   private PlanningMode mode;

   private List<EnumMap<Axis3D, List<PolynomialReadOnly>>> latestSwingTrajectories;
   private MonteCarloFootstepPlannerParameters monteCarloFootstepPlannerParameters;
   private MonteCarloFootstepPlannerRequest monteCarloFootstepPlannerRequest;
   private CollisionFreeSwingCalculator collisionFreeSwingCalculator;
   private ContinuousHikingParameters continuousHikingParameters;
   private MonteCarloFootstepPlanner monteCarloFootstepPlanner;
   private SwingPlannerParametersBasics swingPlannerParameters;
   private FootstepPlanningResult footstepPlanningResult;
   private ContinuousPlannerStatistics statistics;
   private FootstepPlanningModule footstepPlanner;
   private FootstepPlan monteCarloReferencePlan;
   private TerrainMapData latestTerrainMapData;
   private FootstepPlan previousFootstepPlan;
   private HeightMapData latestHeightMapData;
   private TerrainPlanningDebugger debugger;
   private FootstepPlan latestFootstepPlan;
   private FootstepPlannerLogger logger;

   private boolean initialized = false;
   private boolean planAvailable = false;
   private boolean resetMonteCarloFootstepPlanner = false;
   private boolean active;
   private double previousContinuousHikingSwingTime = 0.0;

   private List<SideDependentList<Pose3D>> walkToGoalWayPointList = new ArrayList<>();
   private final Point3D robotLocation = new Point3D();

   float xRandomMargin = 0.2f;
   float nominalStanceWidth = 0.22f;

   public ContinuousPlanner(DRCRobotModel robotModel,
                            HumanoidReferenceFrames humanoidReferenceFrames,
                            PlanningMode mode,
                            ContinuousHikingParameters continuousHikingParameters,
                            MonteCarloFootstepPlannerParameters monteCarloPlannerParameters,
                            TerrainPlanningDebugger debugger)
   {
      this.continuousHikingParameters = continuousHikingParameters;
      this.referenceFrames = humanoidReferenceFrames;
      this.debugger = debugger;
      this.active = true;
      this.mode = mode;

      this.monteCarloFootstepPlannerParameters = monteCarloPlannerParameters;
      footstepPlanner = FootstepPlanningModuleLauncher.createModule(robotModel, "ForContinuousWalking");
      footstepPlanner.getSwingPlannerParameters().set(robotModel.getSwingPlannerParameters());
      swingPlannerParameters = footstepPlanner.getSwingPlannerParameters();
      this.logger = new FootstepPlannerLogger(footstepPlanner);
      this.monteCarloFootstepPlanner = new MonteCarloFootstepPlanner(monteCarloFootstepPlannerParameters,
                                                                     FootstepPlanningModuleLauncher.createFootPolygons(robotModel),
                                                                     debugger);
      this.collisionFreeSwingCalculator = new CollisionFreeSwingCalculator(robotModel.getFootstepPlannerParameters("ForContinuousWalking"),
                                                                           swingPlannerParameters,
                                                                           robotModel.getWalkingControllerParameters(),
                                                                           FootstepPlanningModuleLauncher.createFootPolygons(robotModel));
   }

   public void initialize()
   {
      footstepPlanner.clearCustomTerminationConditions();
      footstepPlanner.addCustomTerminationCondition((time, iterations, finalStep, secondToFinalStep, pathSize) ->
                                                    {
                                                       return pathSize >= continuousHikingParameters.getNumberOfStepsToSend();
                                                    });

      for (RobotSide side : RobotSide.values)
      {
         startingStancePose.get(side).setFromReferenceFrame(referenceFrames.getSoleFrame(side));
         goalStancePose.get(side).setFromReferenceFrame(referenceFrames.getSoleFrame(side));
      }

      FramePose3D finalGoalMidPose = new FramePose3D();
      finalGoalMidPose.interpolate(startingStancePose.get(RobotSide.LEFT), startingStancePose.get(RobotSide.RIGHT), 0.5);

      walkingStartMidPose.getPosition().setX(finalGoalMidPose.getPosition().getX());
      walkingStartMidPose.getPosition().setY(finalGoalMidPose.getPosition().getY());
      walkingStartMidPose.getPosition().setZ(finalGoalMidPose.getPosition().getZ());
      walkingStartMidPose.getOrientation().setToYawOrientation(finalGoalMidPose.getRotation().getYaw());

      initialized = true;
   }

   public void planToGoal(ContinuousWalkingCommandMessage command)
   {
      this.command = command;

      if (command.getUseAstarFootstepPlanner())
      {
         planToGoalWithAStar(command.getUsePreviousPlanAsReference(), false);
      }
      else if (command.getUseHybridPlanner())
      {
         planToGoalWithHybridPlanner(command.getUsePreviousPlanAsReference(), command.getUseMonteCarloPlanAsReference());
      }
      else if (command.getUseMonteCarloFootstepPlanner())
      {
         planToGoalWithMonteCarlo();
      }
      else
      {
         planToGoalWithAStar(true, false);
      }
   }

   public void planToGoalWithHybridPlanner(boolean usePreviousPlanAsReference, boolean useMonteCarloPlanAsReference)
   {
      long startTimeForStatistics = System.currentTimeMillis();
      generateMonteCarloFootstepPlan();
      generateAStarFootstepPlan(latestHeightMapData, latestTerrainMapData, usePreviousPlanAsReference, useMonteCarloPlanAsReference);
      statistics.setLastAndTotalPlanningTimes((float) (System.currentTimeMillis() - startTimeForStatistics) / 1000.0f);
   }

   public void planToGoalWithAStar(boolean usePreviousPlanAsReference, boolean useMonteCarloPlanAsReference)
   {
      long startTimeForStatistics = System.currentTimeMillis();
      generateAStarFootstepPlan(latestHeightMapData, latestTerrainMapData, usePreviousPlanAsReference, useMonteCarloPlanAsReference);
      statistics.setLastAndTotalPlanningTimes((float) (System.currentTimeMillis() - startTimeForStatistics) / 1000.0f);
   }

   public void planToGoalWithMonteCarlo()
   {
      long startTimeForStatistics = System.currentTimeMillis();

      latestFootstepPlan = generateMonteCarloFootstepPlan();
      latestSwingTrajectories = computeSwingTrajectories(latestHeightMapData, latestFootstepPlan); // this also packs the footstep plan with optimized waypoints

      statistics.setLastAndTotalPlanningTimes((float) (System.currentTimeMillis() - startTimeForStatistics) / 1000.0f);
   }

   public FootstepPlan generateMonteCarloFootstepPlan()
   {
      monteCarloFootstepPlannerRequest = new MonteCarloFootstepPlannerRequest();
      monteCarloFootstepPlannerRequest.setTimeout(monteCarloFootstepPlannerParameters.getTimeoutDuration());
      monteCarloFootstepPlannerRequest.setStartFootPose(RobotSide.LEFT, startingStancePose.get(RobotSide.LEFT));
      monteCarloFootstepPlannerRequest.setStartFootPose(RobotSide.RIGHT, startingStancePose.get(RobotSide.RIGHT));
      monteCarloFootstepPlannerRequest.setGoalFootPose(RobotSide.LEFT, goalStancePose.get(RobotSide.LEFT));
      monteCarloFootstepPlannerRequest.setGoalFootPose(RobotSide.RIGHT, goalStancePose.get(RobotSide.RIGHT));
      monteCarloFootstepPlannerRequest.setRequestedInitialStanceSide(imminentFootstepSide);
      monteCarloFootstepPlannerRequest.setTerrainMapData(latestTerrainMapData);
      monteCarloFootstepPlannerRequest.setHeightMapData(latestHeightMapData);

      long timeStart = System.nanoTime();

      if (resetMonteCarloFootstepPlanner)
      {
         monteCarloFootstepPlanner.reset(monteCarloFootstepPlannerRequest);
      }

      FootstepPlan latestMonteCarloPlan = monteCarloFootstepPlanner.generateFootstepPlan(monteCarloFootstepPlannerRequest);

      monteCarloFootstepPlan.set(latestMonteCarloPlan);
      footstepPlanningResult = FootstepPlanningResult.FOUND_SOLUTION;
      planAvailable = latestMonteCarloPlan.getNumberOfSteps() > 0;

      LogTools.warn(monteCarloFootstepPlannerRequest);
      LogTools.warn("Monte-Carlo: {}", latestMonteCarloPlan);

      long timeEnd = System.nanoTime();

      statistics.appendString(String.format("Total Time: %.3f ms, Plan Size: %d, Visited: %d, Layer Counts: %s",
                                            (timeEnd - timeStart) / 1e6,
                                            latestMonteCarloPlan.getNumberOfSteps(),
                                            monteCarloFootstepPlanner.getVisitedNodes().size(),
                                            MonteCarloPlannerTools.getLayerCountsString(monteCarloFootstepPlanner.getRoot())));

      //debugger.plotFootstepPlan(latestMonteCarloPlan);
      return latestMonteCarloPlan;
   }

   public void generateAStarFootstepPlan(HeightMapData heightMapData, TerrainMapData terrainMapData, boolean usePreviousPlanAsReference, boolean useMonteCarloPlanAsReference)
   {
      if (footstepPlanner.isPlanning())
      {
         LogTools.warn("Footstep Planner is Busy!");
         return;
      }

      FootstepPlannerRequest request = createFootstepPlannerRequest(startingStancePose, goalStancePose);
      request.setRequestedInitialStanceSide(imminentFootstepSide);
      request.setHeightMapData(heightMapData);
      request.setTerrainMapData(terrainMapData);
      request.setSnapGoalSteps(true);
      request.setAbortIfGoalStepSnappingFails(true);
      LogTools.info("AStar {}", request);

      if (useMonteCarloPlanAsReference && monteCarloFootstepPlan.get() != null && monteCarloFootstepPlan.get().getNumberOfSteps() > 0)
      {
         monteCarloReferencePlan = new FootstepPlan(monteCarloFootstepPlan.getAndSet(null));
         request.setReferencePlan(monteCarloReferencePlan);
         statistics.appendString("Using Monte-Carlo Plan As Reference: Total Steps: " + monteCarloReferencePlan.getNumberOfSteps());
         statistics.appendString("Monte-Carlo Footstep Plan: " + monteCarloReferencePlan);

         if (previousFootstepPlan != null && previousFootstepPlan.getNumberOfSteps() > 0)
            this.previousFootstepPlan.remove(0);
      }
      else if (usePreviousPlanAsReference && previousFootstepPlan != null)
      {
         statistics.appendString("Using Previous Plan As Reference: Total Steps: " + previousFootstepPlan.getNumberOfSteps());

         // Sets the previous footstep plan to be a reference for the current plan
         if (latestFootstepPlan.getNumberOfSteps() >= continuousHikingParameters.getNumberOfStepsToSend())
            previousFootstepPlan = new FootstepPlan(latestFootstepPlan);

         if (previousFootstepPlan.getNumberOfSteps() < continuousHikingParameters.getNumberOfStepsToSend())
         {
            statistics.appendString("[ERROR]: Previous Plan for Reference: Not Enough Steps: " + previousFootstepPlan.getNumberOfSteps() + "!");
         }
         else
         {
            // These are steps that are considered to be at the start of the plan, don't want to use them as reference
            this.previousFootstepPlan.remove(0);

            if (!continuousHikingParameters.getOverrideEntireQueueEachStep())
               this.previousFootstepPlan.remove(1);

            request.setReferencePlan(this.previousFootstepPlan);

            double stepDuration = continuousHikingParameters.getSwingTime() + continuousHikingParameters.getTransferTime();
            double referencePlanTimeout = stepDuration * continuousHikingParameters.getPlannerTimeoutFraction();

            statistics.appendString("Using Reference Plan: " + this.previousFootstepPlan.getNumberOfSteps() + "Timeout: " + referencePlanTimeout);
            statistics.appendString("Previous Footstep Plan: " + previousFootstepPlan);
            request.setTimeout(referencePlanTimeout);
         }
      }
      else
      {
         statistics.appendString("[PLANNER] No Reference Plan");
         request.setTimeout(continuousHikingParameters.getPlanningWithoutReferenceTimeout());
      }

      FootstepPlannerOutput plannerOutput = footstepPlanner.handleRequest(request);

      if (plannerOutput != null)
      {
         footstepPlanningResult = plannerOutput.getFootstepPlanningResult();
         planAvailable = footstepPlanner.getOutput().getFootstepPlan().getNumberOfSteps() > 0;
         latestFootstepPlan = plannerOutput.getFootstepPlan();
         latestSwingTrajectories = plannerOutput.getSwingTrajectories();

         String message = String.format("Plan Result: %s, Steps: %d, Result: %s, Initial Stance: %s",
                                        footstepPlanningResult,
                                        footstepPlanner.getOutput().getFootstepPlan().getNumberOfSteps(),
                                        planAvailable,
                                        imminentFootstepSide);
         LogTools.info(message);
         statistics.appendString(message);
      }

      assert plannerOutput != null;
      statistics.setTotalStepsPlanned(plannerOutput.getFootstepPlan().getNumberOfSteps());
   }

   public void logFootStePlan()
   {
      if (continuousHikingParameters.getLogFootstepPlans())
      {
          logger.logSession();
      }
   }

   public void setGoalWaypointPoses()
   {
      switch (this.mode)
      {
         case FAST_HIKING:
            ContinuousPlanningTools.setRandomizedStraightGoalPoses(walkingStartMidPose,
                                                                   startingStancePose,
                                                                   goalStancePose,
                                                                   (float) continuousHikingParameters.getGoalPoseForwardDistance(),
                                                                   xRandomMargin,
                                                                   (float) continuousHikingParameters.getGoalPoseUpDistance(), nominalStanceWidth);
            break;
         case WALK_TO_GOAL:
            if (walkToGoalWayPointList.isEmpty())
            {
               mode = PlanningMode.FAST_HIKING;
               return;
            }

            goalStancePose.get(RobotSide.LEFT).set(walkToGoalWayPointList.get(0).get(RobotSide.LEFT));
            goalStancePose.get(RobotSide.RIGHT).set(walkToGoalWayPointList.get(0).get(RobotSide.RIGHT));

            Vector3DBasics robotLocationVector = referenceFrames.getMidFeetZUpFrame().getTransformToWorldFrame().getTranslation();
            robotLocation.set(robotLocationVector);
            double distanceToGoalPose = ContinuousPlanningTools.getDistanceFromRobotToGoalPoseOnXYPlane(robotLocation, goalStancePose);

            if (distanceToGoalPose < continuousHikingParameters.getNextWaypointDistanceMargin())
            {
               LogTools.info("Removed goal from list... ready to go to the next one");
               walkToGoalWayPointList.remove(0);
               continuousHikingParameters.setEnableContinuousWalking(false);

               if (!walkToGoalWayPointList.isEmpty())
               {
                  goalStancePose.get(RobotSide.LEFT).set(walkToGoalWayPointList.get(0).get(RobotSide.LEFT));
                  goalStancePose.get(RobotSide.RIGHT).set(walkToGoalWayPointList.get(0).get(RobotSide.RIGHT));
                  debugger.publishStartAndGoalForVisualization(getStartingStancePose(), getGoalStancePose());
               }
            }
            break;
      }
   }

   public FootstepPlannerRequest createFootstepPlannerRequest(SideDependentList<FramePose3D> startPose, SideDependentList<FramePose3D> goalPose)
   {
      FootstepPlannerRequest request = new FootstepPlannerRequest();
      request.setStartFootPoses(startPose.get(RobotSide.LEFT), startPose.get(RobotSide.RIGHT));
      request.setGoalFootPoses(goalPose.get(RobotSide.LEFT), goalPose.get(RobotSide.RIGHT));
      request.setSwingPlannerType(SwingPlannerType.MULTI_WAYPOINT_POSITION);
      request.setPerformAStarSearch(true);
      request.setAssumeFlatGround(false);
      request.setPlanBodyPath(false);
      return request;
   }

   public FootstepDataListMessage getLimitedFootstepDataListMessage(ContinuousHikingParameters parameters, List<QueuedFootstepStatusMessage> controllerQueue)
   {
      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      footstepDataListMessage.setDefaultSwingDuration(parameters.getSwingTime());
      footstepDataListMessage.setDefaultTransferDuration(parameters.getTransferTime());

      // We expect the plannerOutput to contain this number of steps we ask for
      int index = 0;
      if (!controllerQueue.isEmpty() && !continuousHikingParameters.getOverrideEntireQueueEachStep())
      {
         PlannedFootstep stepToNotOverride = new PlannedFootstep(RobotSide.fromByte(controllerQueue.get(1).getRobotSide()),
                                                                 new Pose3D(controllerQueue.get(1).getLocation(), controllerQueue.get(1).getOrientation()));
         footstepDataListMessage.getFootstepDataList().add().set(stepToNotOverride.getAsMessage());
         index = 1;
      }

      int totalNumberOfSteps = Math.min(latestFootstepPlan.getNumberOfSteps(), parameters.getNumberOfStepsToSend());
      for (int i = index; i < totalNumberOfSteps; i++)
      {
         PlannedFootstep footstep = latestFootstepPlan.getFootstep(i);
         footstep.limitFootholdVertices();
         footstepDataListMessage.getFootstepDataList().add().set(footstep.getAsMessage());
      }

      return footstepDataListMessage;
   }

   public FootstepDataListMessage getMonteCarloFootstepDataListMessage()
   {
      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();

      if (monteCarloReferencePlan != null)
      {
         for (int i = 0; i < monteCarloReferencePlan.getNumberOfSteps(); i++)
         {
            PlannedFootstep footstep = monteCarloReferencePlan.getFootstep(i);
            footstep.limitFootholdVertices();
            footstepDataListMessage.getFootstepDataList().add().set(footstep.getAsMessage());
         }
      }

      return footstepDataListMessage;
   }

   /**
    * This method gets called when we need a new footstep plan, this gets the latest information in the controller footstep queue
    */
   public void getImminentStanceFromLatestStatus(AtomicReference<FootstepStatusMessage> footstepStatusMessage,
                                                 List<QueuedFootstepStatusMessage> controllerQueue)
   {
      RobotSide imminentFootSide = RobotSide.fromByte(footstepStatusMessage.get().getRobotSide());
      assert imminentFootSide != null;

      int index = getNextIndexOnOppositeSide(RobotSide.fromByte(footstepStatusMessage.get().getRobotSide()), controllerQueue);

      FramePose3D imminentFootstepPose = new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                         footstepStatusMessage.get().getDesiredFootPositionInWorld(),
                                                         footstepStatusMessage.get().getDesiredFootOrientationInWorld());

      FramePose3D nextRobotStepAfterCurrent;

      if (continuousHikingParameters.getOverrideEntireQueueEachStep())
      {
         nextRobotStepAfterCurrent = new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                     referenceFrames.getSoleFrame(imminentFootSide.getOppositeSide()).getTransformToWorldFrame());
      }
      else
      {
         nextRobotStepAfterCurrent = new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                     controllerQueue.get(index).getLocation(),
                                                     controllerQueue.get(index).getOrientation());
      }

      updateImminentStance(nextRobotStepAfterCurrent, imminentFootstepPose, imminentFootSide);
   }

   private int getNextIndexOnOppositeSide(RobotSide side, List<QueuedFootstepStatusMessage> controllerQueue)
   {
      int i = 0;
      while (i < controllerQueue.size() && RobotSide.fromByte(controllerQueue.get(i).getRobotSide()) == side)
         i++;

      return i;
   }

   public boolean updateImminentStance(FramePose3D nextRobotStepAfterCurrent, FramePose3D imminentFootstepPose, RobotSide imminentFootstepSide)
   {
      if (!this.imminentFootstepSide.equals(imminentFootstepSide))
         LogTools.warn("Updating Imminent Stance: From:{}-to-{} {}", this.imminentFootstepSide, imminentFootstepSide, imminentFootstepPose);

      FramePose3D oldLeftPose = new FramePose3D();
      FramePose3D oldRightPose = new FramePose3D();
      oldLeftPose.set(startingStancePose.get(RobotSide.LEFT));
      oldRightPose.set(startingStancePose.get(RobotSide.RIGHT));

      this.imminentFootstepPose.set(imminentFootstepPose);
      this.imminentFootstepSide = imminentFootstepSide;

      startingStancePose.get(imminentFootstepSide).set(imminentFootstepPose);
      startingStancePose.get(imminentFootstepSide.getOppositeSide()).set(nextRobotStepAfterCurrent);

      if (startingStancePose.get(RobotSide.LEFT).equals(oldLeftPose) && startingStancePose.get(RobotSide.RIGHT).equals(oldRightPose))
      {
         return false;
      }
      else
      {
         return true;
      }
   }

   public RobotSide getCloserSideToGoal()
   {
      double leftDistance = startingStancePose.get(RobotSide.LEFT).getPosition().distance(goalStancePose.get(RobotSide.LEFT).getPosition());
      double rightDistance = startingStancePose.get(RobotSide.RIGHT).getPosition().distance(goalStancePose.get(RobotSide.RIGHT).getPosition());

      if (leftDistance < rightDistance + 0.01)
         return RobotSide.LEFT;
      else
         return RobotSide.RIGHT;
   }

   public void transitionCallback()
   {
      statistics.appendString("[TRANSITION]: Resetting Previous Plan Reference");
      this.previousFootstepPlan = new FootstepPlan(latestFootstepPlan);

      if (command.getUseMonteCarloFootstepPlanner())
      {
         monteCarloFootstepPlanner.transitionToOptimal();
      }
   }

   public void syncParametersCallback()
   {
      if (previousContinuousHikingSwingTime != continuousHikingParameters.getSwingTime())
      {
         footstepPlanner.getSwingPlannerParameters().setMinimumSwingTime(continuousHikingParameters.getSwingTime());
         footstepPlanner.getSwingPlannerParameters().setMaximumSwingTime(continuousHikingParameters.getSwingTime());
         previousContinuousHikingSwingTime = continuousHikingParameters.getSwingTime();
         swingPlannerParameters.set(footstepPlanner.getSwingPlannerParameters());
      }
   }

   /*
    * This method both generates the swing trajectories and packs the footstep plan with optimized waypoints
    */
   public List<EnumMap<Axis3D, List<PolynomialReadOnly>>> computeSwingTrajectories(HeightMapData heightMapData, FootstepPlan footstepPlan)
   {
      collisionFreeSwingCalculator.setHeightMapData(heightMapData);

      // this also packs the footstep plan with optimized waypoints
      collisionFreeSwingCalculator.computeSwingTrajectories(startingStancePose, footstepPlan);

      List<EnumMap<Axis3D, List<PolynomialReadOnly>>> swingTrajectories = collisionFreeSwingCalculator.getSwingTrajectories();
      return swingTrajectories;
   }

   public MonteCarloFootstepPlanner getMonteCarloFootstepPlanner()
   {
      return monteCarloFootstepPlanner;
   }

   public FootstepPlanningResult getFootstepPlanningResult()
   {
      return footstepPlanningResult;
   }

   public FootstepPlan getLatestFootstepPlan()
   {
      return latestFootstepPlan;
   }

   public List<EnumMap<Axis3D, List<PolynomialReadOnly>>> getLatestSwingTrajectories()
   {
      return latestSwingTrajectories;
   }

   public void setPlanAvailable(boolean planAvailable)
   {
      this.planAvailable = planAvailable;
   }

   public boolean isPlanAvailable()
   {
      return planAvailable;
   }

   public boolean isActive()
   {
      return active;
   }

   public void setActive(boolean active)
   {
      this.active = active;
   }

   public void setInitialized(boolean initialized)
   {
      this.initialized = initialized;
   }

   public boolean isInitialized()
   {
      return initialized;
   }

   public FootstepPlannerParametersBasics getFootstepPlannerParameters()
   {
      return footstepPlanner.getFootstepPlannerParameters();
   }

   public MonteCarloFootstepPlannerParameters getMonteCarloFootstepPlannerParameters()
   {
      return monteCarloFootstepPlannerParameters;
   }

   public SwingPlannerParametersBasics getSwingPlannerParameters()
   {
      return swingPlannerParameters;
   }


   public FramePose3D getImminentFootstepPose()
   {
      return imminentFootstepPose;
   }

   public RobotSide getImminentFootstepSide()
   {
      return imminentFootstepSide;
   }

   public SideDependentList<FramePose3D> getGoalStancePose()
   {
      return goalStancePose;
   }

   public SideDependentList<FramePose3D> getStartingStancePose()
   {
      return startingStancePose;
   }

   public void setContinuousPlannerStatistics(ContinuousPlannerStatistics continuousPlannerStatistics)
   {
      this.statistics = continuousPlannerStatistics;
   }

   public PlanningMode getMode()
   {
      return mode;
   }

   public void setLatestHeightMapData(HeightMapData heightMapData)
   {
      this.latestHeightMapData = heightMapData;
   }

   public void setLatestTerrainMapData(TerrainMapData terrainMapData)
   {
      this.latestTerrainMapData = terrainMapData;
   }

   public void requestMonteCarloPlannerReset()
   {
      resetMonteCarloFootstepPlanner = true;
   }

   public void addWayPointToList(Pose3D leftFootGoalPose, Pose3D rightFootGoalPose)
   {
      mode = PlanningMode.WALK_TO_GOAL;
      //TODO make sure we don't add the same values twice
      SideDependentList<Pose3D> latestWayPoint = new SideDependentList<>();
      latestWayPoint.put(RobotSide.LEFT, leftFootGoalPose);
      latestWayPoint.put(RobotSide.RIGHT, rightFootGoalPose);

      LogTools.info("Added waypoint for WALK_TO_GOAL");
      walkToGoalWayPointList.add(latestWayPoint);

      // Until the first waypoint is removed from this list, that is the current goal
      goalStancePose.get(RobotSide.LEFT).set(walkToGoalWayPointList.get(0).get(RobotSide.LEFT));
      goalStancePose.get(RobotSide.RIGHT).set(walkToGoalWayPointList.get(0).get(RobotSide.RIGHT));
   }
}

