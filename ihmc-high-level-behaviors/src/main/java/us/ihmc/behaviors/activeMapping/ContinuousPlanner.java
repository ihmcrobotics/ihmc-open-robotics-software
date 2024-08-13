package us.ihmc.behaviors.activeMapping;

import behavior_msgs.msg.dds.ContinuousWalkingCommandMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.QueuedFootstepStatusMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.MonteCarloFootstepPlannerParameters;
import us.ihmc.footstepPlanning.PlannedFootstep;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.footstepPlanning.monteCarloPlanning.MonteCarloFootstepPlanner;
import us.ihmc.footstepPlanning.monteCarloPlanning.MonteCarloFootstepPlannerRequest;
import us.ihmc.footstepPlanning.monteCarloPlanning.MonteCarloPlannerTools;
import us.ihmc.footstepPlanning.swing.CollisionFreeSwingCalculator;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerType;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class ContinuousPlanner
{
   private static final boolean DEBUG_WITH_MORE_PRINTS = true;
   private final HumanoidReferenceFrames referenceFrames;
   private final TerrainPlanningDebugger debugger;
   private final ContinuousPlannerStatistics statistics;
   private final CollisionFreeSwingCalculator collisionFreeSwingCalculator;
   private final FootstepPlannerLogger logger;

   private final FootstepPlanningModule footstepPlanner;
   private final MonteCarloFootstepPlanner monteCarloFootstepPlanner;
   private final AtomicReference<FootstepPlan> monteCarloFootstepPlan = new AtomicReference<>(null);
   private FootstepPlan monteCarloReferencePlan;
   private FootstepPlan previousFootstepPlan;
   private FootstepPlan latestFootstepPlan;
   private FootstepPlanningResult footstepPlanningResult;

   private final ContinuousHikingParameters continuousHikingParameters;
   private final MonteCarloFootstepPlannerParameters monteCarloPlannerParameters;
   private final DefaultFootstepPlannerParametersBasics footstepPlannerParameters;
   private final SwingPlannerParametersBasics swingPlannerParameters;

   private final SideDependentList<FramePose3D> startStancePose = new SideDependentList<>(new FramePose3D(), new FramePose3D());
   private final SideDependentList<FramePose3D> goalStancePose = new SideDependentList<>(new FramePose3D(), new FramePose3D());
   private final FramePose3D walkingStartMidPose = new FramePose3D();
   private final FramePose3D imminentFootstepPose = new FramePose3D();
   private RobotSide imminentFootstepSide = RobotSide.LEFT;

   private ContinuousWalkingCommandMessage command;
   private AtomicReference<FootstepStatusMessage> latestFootstepStatusMessage = new AtomicReference<>(new FootstepStatusMessage());
   private List<QueuedFootstepStatusMessage> controllerQueue = new ArrayList<>();

   private TerrainMapData latestTerrainMapData;
   private HeightMapData latestHeightMapData;

   private boolean initialized = false;
   private boolean planAvailable = false;
   private boolean resetMonteCarloFootstepPlanner = false;

   public ContinuousPlanner(DRCRobotModel robotModel,
                            HumanoidReferenceFrames referenceFrames,
                            ContinuousHikingParameters continuousHikingParameters,
                            MonteCarloFootstepPlannerParameters monteCarloPlannerParameters,
                            DefaultFootstepPlannerParametersBasics footstepPlannerParameters,
                            SwingPlannerParametersBasics swingPlannerParameters,
                            TerrainPlanningDebugger debugger,
                            ContinuousPlannerStatistics statistics)
   {
      this.referenceFrames = referenceFrames;
      this.continuousHikingParameters = continuousHikingParameters;
      this.monteCarloPlannerParameters = monteCarloPlannerParameters;
      this.footstepPlannerParameters = footstepPlannerParameters;
      this.swingPlannerParameters = swingPlannerParameters;
      this.debugger = debugger;
      this.statistics = statistics;

      footstepPlanner = FootstepPlanningModuleLauncher.createModule(robotModel, "ForContinuousWalking");

      this.logger = new FootstepPlannerLogger(footstepPlanner);
      this.monteCarloFootstepPlanner = new MonteCarloFootstepPlanner(this.monteCarloPlannerParameters,
                                                                     FootstepPlanningModuleLauncher.createFootPolygons(robotModel));
      this.collisionFreeSwingCalculator = new CollisionFreeSwingCalculator(footstepPlannerParameters,
                                                                           swingPlannerParameters,
                                                                           robotModel.getWalkingControllerParameters(),
                                                                           FootstepPlanningModuleLauncher.createFootPolygons(robotModel));
   }

   public void initialize()
   {
      footstepPlanner.clearCustomTerminationConditions();
      footstepPlanner.addCustomTerminationCondition((time, iterations, finalStep, secondToFinalStep, pathSize) -> pathSize
                                                                                                                  >= continuousHikingParameters.getNumberOfStepsToSend());

      for (RobotSide side : RobotSide.values)
      {
         startStancePose.get(side).setFromReferenceFrame(referenceFrames.getSoleFrame(side));
         goalStancePose.get(side).setFromReferenceFrame(referenceFrames.getSoleFrame(side));
      }

      FramePose3D finalGoalMidPose = new FramePose3D();
      finalGoalMidPose.interpolate(startStancePose.get(RobotSide.LEFT), startStancePose.get(RobotSide.RIGHT), 0.5);

      // This pose represents the starting position where the robot started walking from
      walkingStartMidPose.getPosition().setX(finalGoalMidPose.getPosition().getX());
      walkingStartMidPose.getPosition().setY(finalGoalMidPose.getPosition().getY());
      walkingStartMidPose.getPosition().setZ(finalGoalMidPose.getPosition().getZ());
      walkingStartMidPose.getOrientation().setToYawOrientation(finalGoalMidPose.getRotation().getYaw());

      // Getting ready to use the continuous planner, since things are just getting initialized, plan from the robot because it should be standing
      initialized = true;
   }

   public void planToGoal(ContinuousWalkingCommandMessage command)
   {
      this.command = command;

      long startTimeForStatistics = System.currentTimeMillis();

      if (command.getUseAstarFootstepPlanner())
      {
         generateAStarFootstepPlan(latestHeightMapData, latestTerrainMapData, command.getUsePreviousPlanAsReference(), false);
      }
      else if (command.getUseMonteCarloFootstepPlanner())
      {
         latestFootstepPlan = generateMonteCarloFootstepPlan();
         computeSwingTrajectories(latestHeightMapData, latestFootstepPlan); // this also packs the footstep plan with optimized waypoints
      }
      else if (command.getUseHybridPlanner())
      {
         generateMonteCarloFootstepPlan();
         generateAStarFootstepPlan(latestHeightMapData,
                                   latestTerrainMapData,
                                   command.getUsePreviousPlanAsReference(),
                                   command.getUseMonteCarloPlanAsReference());
         statistics.setLastAndTotalPlanningTimes((float) (System.currentTimeMillis() - startTimeForStatistics) / 1000.0f);
      }
      else
      {
         generateAStarFootstepPlan(latestHeightMapData, latestTerrainMapData, true, false);
      }

      statistics.setLastAndTotalPlanningTimes((float) (System.currentTimeMillis() - startTimeForStatistics) / 1000.0f);
   }

   public void generateAStarFootstepPlan(HeightMapData heightMapData,
                                         TerrainMapData terrainMapData,
                                         boolean usePreviousPlanAsReference,
                                         boolean useMonteCarloPlanAsReference)
   {
      if (footstepPlanner.isPlanning())
      {
         LogTools.warn("Footstep Planner is Busy!");
         return;
      }

      FootstepPlannerRequest request = new FootstepPlannerRequest();
      request.setStartFootPoses(startStancePose.get(RobotSide.LEFT), startStancePose.get(RobotSide.RIGHT));
      request.setGoalFootPoses(goalStancePose.get(RobotSide.LEFT), goalStancePose.get(RobotSide.RIGHT));
      request.setSwingPlannerType(SwingPlannerType.MULTI_WAYPOINT_POSITION);
      request.setPerformAStarSearch(true);
      request.setAssumeFlatGround(false);
      request.setPlanBodyPath(false);
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
         // We are trying to use the previous plan as a reference for the next planning session so its faster
         // However
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
            if (continuousHikingParameters.getStepPublisherEnabled())
               this.previousFootstepPlan.remove(0);

            // If the step publisher isn't enabled, we don't want to remove a step because we aren't walking, and that will mess up the reference plan
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

      // Setting these parameters right before using them ensures that we have the most recent parameters
      // We want the swing time from ContinuousHikingParameters to be used, so we need to set the swing parameters accordingly
      swingPlannerParameters.setMinimumSwingTime(continuousHikingParameters.getSwingTime());
      swingPlannerParameters.setMaximumSwingTime(continuousHikingParameters.getSwingTime());
      footstepPlanner.getFootstepPlannerParameters().set(footstepPlannerParameters);
      footstepPlanner.getSwingPlannerParameters().set(swingPlannerParameters);

      if (DEBUG_WITH_MORE_PRINTS)
      {
         LogTools.info("Swing Planner: {}", swingPlannerParameters);
      }

      FootstepPlannerOutput plannerOutput = footstepPlanner.handleRequest(request);

      if (plannerOutput != null)
      {
         footstepPlanningResult = plannerOutput.getFootstepPlanningResult();
         planAvailable = footstepPlanner.getOutput().getFootstepPlan().getNumberOfSteps() > 0;
         latestFootstepPlan = plannerOutput.getFootstepPlan();

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
      // In case logging footstep plans becomes a problem, we have this feature where we can not log plans if we want too
      if (continuousHikingParameters.getLogFootstepPlans())
      {
         logger.logSession();
      }
   }

   public FootstepPlan generateMonteCarloFootstepPlan()
   {
      MonteCarloFootstepPlannerRequest monteCarloFootstepPlannerRequest = new MonteCarloFootstepPlannerRequest();
      monteCarloFootstepPlannerRequest.setTimeout(monteCarloPlannerParameters.getTimeoutDuration());
      monteCarloFootstepPlannerRequest.setStartFootPose(RobotSide.LEFT, startStancePose.get(RobotSide.LEFT));
      monteCarloFootstepPlannerRequest.setStartFootPose(RobotSide.RIGHT, startStancePose.get(RobotSide.RIGHT));
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
      debugger.setRequest(monteCarloFootstepPlannerRequest);
      debugger.refresh(monteCarloFootstepPlannerRequest.getTerrainMapData());

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

      return latestMonteCarloPlan;
   }

   public void setGoalWaypointPoses(FramePose3DReadOnly leftGoalPose, FramePose3DReadOnly rightGoalPose)
   {
      goalStancePose.get(RobotSide.LEFT).set(leftGoalPose);
      goalStancePose.get(RobotSide.RIGHT).set(rightGoalPose);
   }

   public FootstepDataListMessage getLimitedFootstepDataListMessage(ContinuousHikingParameters continuousHIkingParameters,
                                                                    List<QueuedFootstepStatusMessage> controllerQueue)
   {
      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      footstepDataListMessage.setDefaultSwingDuration(continuousHIkingParameters.getSwingTime());
      footstepDataListMessage.setDefaultTransferDuration(continuousHIkingParameters.getTransferTime());

      // We expect the plannerOutput to contain this number of steps we ask for
      int index = 0;
      if (!controllerQueue.isEmpty() && !continuousHikingParameters.getOverrideEntireQueueEachStep())
      {
         PlannedFootstep stepToNotOverride = new PlannedFootstep(RobotSide.fromByte(controllerQueue.get(1).getRobotSide()),
                                                                 new Pose3D(controllerQueue.get(1).getLocation(), controllerQueue.get(1).getOrientation()));
         footstepDataListMessage.getFootstepDataList().add().set(stepToNotOverride.getAsMessage());
         index = 1;
      }

      int totalNumberOfSteps = Math.min(latestFootstepPlan.getNumberOfSteps(), continuousHIkingParameters.getNumberOfStepsToSend());
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

   public void setImminentStanceToPlanFrom()
   {
      // This means the controller queue is not empty, and we want to plan from where we are going to be, not where we currently are or we are going to step in place
      if (latestFootstepStatusMessage != null && !controllerQueue.isEmpty())
      {
         getImminentStanceFromLatestStatus(latestFootstepStatusMessage, controllerQueue);
      }
      else
      {
         // Set the starting stance to be the feet of the robot as we want to plan from where the robot is
         FramePose3D leftSolePose = new FramePose3D(ReferenceFrame.getWorldFrame(), referenceFrames.getSoleFrame(RobotSide.LEFT).getTransformToWorldFrame());
         FramePose3D rightSolePose = new FramePose3D(ReferenceFrame.getWorldFrame(), referenceFrames.getSoleFrame(RobotSide.RIGHT).getTransformToWorldFrame());

         double leftSolePositionInXFromRobot = referenceFrames.getSoleFrame(RobotSide.LEFT)
                                                              .getTransformToDesiredFrame(referenceFrames.getMidFeetUnderPelvisFrame())
                                                              .getTranslationX();
         double rightSolePositionInXFromRobot = referenceFrames.getSoleFrame(RobotSide.RIGHT)
                                                               .getTransformToDesiredFrame(referenceFrames.getMidFeetUnderPelvisFrame())
                                                               .getTranslationX();
         RobotSide robotSide = leftSolePositionInXFromRobot > rightSolePositionInXFromRobot ? RobotSide.LEFT : RobotSide.RIGHT;

         if (robotSide == RobotSide.LEFT)
         {
            updateImminentStance(rightSolePose, leftSolePose, robotSide);
         }
         else
         {
            updateImminentStance(leftSolePose, rightSolePose, robotSide);
         }
      }
   }

   public void getImminentStanceFromLatestStatus(AtomicReference<FootstepStatusMessage> footstepStatusMessage,
                                                 List<QueuedFootstepStatusMessage> controllerQueue)
   {
      // Sometimes no message exists, by default ignore the message if its null and use what ever the imminent side was last time
      RobotSide imminentFootSide = imminentFootstepSide;
      RobotSide sideFromMessage = RobotSide.fromByte(footstepStatusMessage.get().getRobotSide());
      if (sideFromMessage != null)
      {
         imminentFootSide = sideFromMessage;
      }

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

   public void updateImminentStance(FramePose3D nextRobotStepAfterCurrent, FramePose3D imminentFootstepPose, RobotSide imminentFootstepSide)
   {
      if (!this.imminentFootstepSide.equals(imminentFootstepSide))
         LogTools.info("Updating Imminent Stance: From:{}-to-{} Imminent Stance is ( {} )",
                       this.imminentFootstepSide,
                       imminentFootstepSide,
                       imminentFootstepPose);

      FramePose3D oldLeftPose = new FramePose3D();
      FramePose3D oldRightPose = new FramePose3D();
      oldLeftPose.set(startStancePose.get(RobotSide.LEFT));
      oldRightPose.set(startStancePose.get(RobotSide.RIGHT));

      this.imminentFootstepPose.set(imminentFootstepPose);
      this.imminentFootstepSide = imminentFootstepSide;

      startStancePose.get(imminentFootstepSide).set(imminentFootstepPose);
      startStancePose.get(imminentFootstepSide.getOppositeSide()).set(nextRobotStepAfterCurrent);
   }

   private int getNextIndexOnOppositeSide(RobotSide side, List<QueuedFootstepStatusMessage> controllerQueue)
   {
      int i = 0;
      while (i < controllerQueue.size() && RobotSide.fromByte(controllerQueue.get(i).getRobotSide()) == side)
         i++;

      return i;
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

   /**
    * This method both generates the swing trajectories and packs the footstep plan with optimized waypoints
    */
   public void computeSwingTrajectories(HeightMapData heightMapData, FootstepPlan footstepPlan)
   {
      collisionFreeSwingCalculator.setHeightMapData(heightMapData);

      // This also packs the footstep plan with optimized waypoints
      collisionFreeSwingCalculator.computeSwingTrajectories(startStancePose, footstepPlan);
   }

   public MonteCarloFootstepPlanner getMonteCarloFootstepPlanner()
   {
      return monteCarloFootstepPlanner;
   }

   public FootstepPlan getLatestFootstepPlan()
   {
      return latestFootstepPlan;
   }

   public void setLatestFootstepPlan(FootstepPlan latestFootstepPlan)
   {
      this.previousFootstepPlan = latestFootstepPlan;
   }

   public boolean isPlanAvailable()
   {
      return planAvailable;
   }

   public void setPlanAvailable(boolean planAvailable)
   {
      this.planAvailable = planAvailable;
   }

   public void setLatestFootstepStatusMessage(AtomicReference<FootstepStatusMessage> latestFootstepStatusMessage)
   {
      this.latestFootstepStatusMessage = latestFootstepStatusMessage;
   }

   public void setLatestControllerQueue(List<QueuedFootstepStatusMessage> controllerQueue)
   {
      this.controllerQueue = controllerQueue;
   }

   public boolean isInitialized()
   {
      return initialized;
   }

   public void setInitialized(boolean initialized)
   {
      this.initialized = initialized;
   }

   public DefaultFootstepPlannerParametersBasics getFootstepPlannerParameters()
   {
      return footstepPlanner.getFootstepPlannerParameters();
   }

   public SideDependentList<FramePose3D> getGoalStancePose()
   {
      return goalStancePose;
   }

   public SideDependentList<FramePose3D> getStartStancePose()
   {
      return startStancePose;
   }

   public FramePose3D getWalkingStartMidPose()
   {
      return walkingStartMidPose;
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
}
