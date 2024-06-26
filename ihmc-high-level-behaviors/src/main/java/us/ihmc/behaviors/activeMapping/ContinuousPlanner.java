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
   private final SideDependentList<FramePose3D> startingStancePose = new SideDependentList<>(new FramePose3D(), new FramePose3D());
   private final SideDependentList<FramePose3D> goalStancePose = new SideDependentList<>(new FramePose3D(), new FramePose3D());
   private final HumanoidReferenceFrames referenceFrames;

   private final AtomicReference<FootstepPlan> monteCarloFootstepPlan = new AtomicReference<>(null);
   private final FramePose3D walkingStartMidPose = new FramePose3D();
   private final FramePose3D imminentFootstepPose = new FramePose3D();
   private RobotSide imminentFootstepSide = RobotSide.LEFT;
   private ContinuousWalkingCommandMessage command;

   private AtomicReference<FootstepStatusMessage> latestFootstepStatusMessage = new AtomicReference<>(new FootstepStatusMessage());
   private List<QueuedFootstepStatusMessage> controllerQueue = new ArrayList<>();

   private List<EnumMap<Axis3D, List<PolynomialReadOnly>>> latestSwingTrajectories;
   private final MonteCarloFootstepPlannerParameters monteCarloFootstepPlannerParameters;
   private final CollisionFreeSwingCalculator collisionFreeSwingCalculator;
   private final ContinuousHikingParameters continuousHikingParameters;
   private final MonteCarloFootstepPlanner monteCarloFootstepPlanner;
   private final SwingPlannerParametersBasics swingPlannerParameters;
   private FootstepPlanningResult footstepPlanningResult;
   private ContinuousPlannerStatistics statistics;
   private final FootstepPlanningModule footstepPlanner;
   private FootstepPlan monteCarloReferencePlan;
   private TerrainMapData latestTerrainMapData;
   private HeightMapData latestHeightMapData;
   private FootstepPlan previousFootstepPlan;
   private FootstepPlan latestFootstepPlan;
   private final FootstepPlannerLogger logger;

   private boolean initialized = false;
   private boolean planFromRobot = false;
   private boolean planAvailable = false;
   private boolean resetMonteCarloFootstepPlanner = false;
   private boolean active;
   private double previousContinuousHikingSwingTime = 0.0;

   public ContinuousPlanner(DRCRobotModel robotModel,
                            HumanoidReferenceFrames humanoidReferenceFrames,
                            ContinuousHikingParameters continuousHikingParameters,
                            MonteCarloFootstepPlannerParameters monteCarloPlannerParameters,
                            TerrainPlanningDebugger debugger)
   {
      this.continuousHikingParameters = continuousHikingParameters;
      this.referenceFrames = humanoidReferenceFrames;
      this.active = true;

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

      // This pose represents the starting position where the robot started walking from
      walkingStartMidPose.getPosition().setX(finalGoalMidPose.getPosition().getX());
      walkingStartMidPose.getPosition().setY(finalGoalMidPose.getPosition().getY());
      walkingStartMidPose.getPosition().setZ(finalGoalMidPose.getPosition().getZ());
      walkingStartMidPose.getOrientation().setToYawOrientation(finalGoalMidPose.getRotation().getYaw());

      // Getting ready to use the continuous planner, since things are just getting initialized, plan from the robot because it should be standing
      planFromRobot = true;
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
      MonteCarloFootstepPlannerRequest monteCarloFootstepPlannerRequest = new MonteCarloFootstepPlannerRequest();
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
      // In case logging footstep plans becomes a problem, we have this feature where we can not log plans if we want too
      if (continuousHikingParameters.getLogFootstepPlans())
      {
         logger.logSession();
      }
   }

   public void setGoalWaypointPoses(FramePose3DReadOnly leftGoalPose, FramePose3DReadOnly rightGoalPose)
   {
      goalStancePose.get(RobotSide.LEFT).set(leftGoalPose);
      goalStancePose.get(RobotSide.RIGHT).set(rightGoalPose);
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
      if (planFromRobot)
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

         planFromRobot = false;
      }
      else if (latestFootstepStatusMessage != null)
      {
         // Maybe we already have steps in the queue and want to plan from where the robot is going to be, and not where it currently is
         getImminentStanceFromLatestStatus(latestFootstepStatusMessage, controllerQueue);
      }
   }

   public void updateImminentStance(FramePose3D nextRobotStepAfterCurrent, FramePose3D imminentFootstepPose, RobotSide imminentFootstepSide)
   {
      if (!this.imminentFootstepSide.equals(imminentFootstepSide))
         LogTools.info("Updating Imminent Stance: From:{}-to-{} Imminent Stance is ( {} )", this.imminentFootstepSide, imminentFootstepSide, imminentFootstepPose);

      FramePose3D oldLeftPose = new FramePose3D();
      FramePose3D oldRightPose = new FramePose3D();
      oldLeftPose.set(startingStancePose.get(RobotSide.LEFT));
      oldRightPose.set(startingStancePose.get(RobotSide.RIGHT));

      this.imminentFootstepPose.set(imminentFootstepPose);
      this.imminentFootstepSide = imminentFootstepSide;

      startingStancePose.get(imminentFootstepSide).set(imminentFootstepPose);
      startingStancePose.get(imminentFootstepSide.getOppositeSide()).set(nextRobotStepAfterCurrent);
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

   private int getNextIndexOnOppositeSide(RobotSide side, List<QueuedFootstepStatusMessage> controllerQueue)
   {
      int i = 0;
      while (i < controllerQueue.size() && RobotSide.fromByte(controllerQueue.get(i).getRobotSide()) == side)
         i++;

      return i;
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

      return collisionFreeSwingCalculator.getSwingTrajectories();
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

   public void setLatestFootstepPlan(FootstepPlan latestFootstepPlan)
   {
      this.previousFootstepPlan = latestFootstepPlan;
   }

   public void setLatestFootstepStatusMessage(AtomicReference<FootstepStatusMessage> latestFootstepStatusMessage)
   {
      this.latestFootstepStatusMessage = latestFootstepStatusMessage;
   }

   public void setLatestControllerQueue(List<QueuedFootstepStatusMessage> controllerQueue)
   {
      this.controllerQueue = controllerQueue;
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

   public DefaultFootstepPlannerParametersBasics getFootstepPlannerParameters()
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

   public FramePose3D getWalkingStartMidPose()
   {
      return walkingStartMidPose;
   }


   public void setContinuousPlannerStatistics(ContinuousPlannerStatistics continuousPlannerStatistics)
   {
      this.statistics = continuousPlannerStatistics;
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

