package us.ihmc.footstepPlanning.graphSearch.stepCost;

import org.jetbrains.annotations.NotNull;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanHeading;
import us.ihmc.footstepPlanning.PlannedFootstep;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerEnvironmentHandler;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerHeuristicCalculator;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapDataReadOnly;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraphNode;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstepTools;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.graphSearch.stepChecking.FootstepCheckerInterface;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersReadOnly;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlanHolder;
import us.ihmc.pathPlanning.graph.structure.DirectedGraph;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.HashMap;
import java.util.List;
import java.util.function.ToDoubleFunction;

public class FootstepCostCalculator implements FootstepCostCalculatorInterface
{
   private enum IdealStepMode
   {
      GOAL, ON_PATH, TOWARDS_PATH;
   }

   // TODO extract these to parameters once they're stable
   private static final double idealStepLengthWhenUpOrDownMultiplier = 0.7;
   private static final double maxDistanceAdjustmentTowardsPath = 0.15;
   private static final double maxYawAdjustmentTowardsPath = Math.toRadians(20.0);

   private final HashMap<DiscreteFootstep, DiscreteFootstep> idealStepMap = new HashMap<>();
   private final WaypointDefinedBodyPathPlanHolder bodyPathPlanHolder;
   private final FootstepPlannerEnvironmentHandler environmentHandler;
   private final FootstepCheckerInterface nodeChecker;

   private SideDependentList<DiscreteFootstep> goalSteps;
   private final YoDouble desiredRelativeHeading;

   private final SideDependentList<YoDouble> idealStepLengths = new SideDependentList<>();
   private final SideDependentList<YoDouble> idealStepWidths = new SideDependentList<>();

   private final YoDouble correctiveDistanceX;
   private final YoDouble correctiveDistanceY;
   private final YoDouble correctiveYaw;
   private final YoDouble idealStepYaw;
   private final YoEnum<FootstepCostCalculator.IdealStepMode> yawMode;
   private final YoEnum<FootstepCostCalculator.IdealStepMode> stepMode;

   private final Pose2D goalMidFootPose = new Pose2D();
   private final Pose2D stanceFootPose = new Pose2D();
   private final Pose2D idealStep = new Pose2D();
   private final Pose3D projectionPose = new Pose3D();
   private final Pose2D idealMidFootPose = new Pose2D();
   private double pathLength;

   private final YoBoolean stepSideIncorrect;
   // This will be used to query what reference alpha to use.
   private FootstepPlan referenceFootstepPlan;
   private DirectedGraph<FootstepGraphNode> footstepGraph;
   private DiscreteFootstep nominalIdealStep;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final DefaultFootstepPlannerParametersBasics footstepPlannerParameters;
   private final FootstepSnapperReadOnly snapper;
   private final ToDoubleFunction<FootstepGraphNode> heuristics;

   private final RigidBodyTransform stanceStepTransform = new RigidBodyTransform();
   private final RigidBodyTransform idealStepTransform = new RigidBodyTransform();
   private final RigidBodyTransform candidateStepTransform = new RigidBodyTransform();
   private final YoDouble edgeCost = new YoDouble("edgeCost", registry);
   private final YoDouble totalCost = new YoDouble("totalCost", registry);
   private final YoDouble heuristicCost = new YoDouble("heuristicCost", registry);
   private final YoDouble idealStepHeuristicCost = new YoDouble("idealStepHeuristicCost", registry);

   private final YoDouble xOffset = new YoDouble("xOffset", registry);
   private final YoDouble yOffset = new YoDouble("yOffset", registry);
   private final YoDouble zOffset = new YoDouble("zOffset", registry);
   private final YoDouble yawOffset = new YoDouble("yawOffset", registry);
   private final YoDouble pitchOffset = new YoDouble("pitchOffset", registry);
   private final YoDouble rollOffset = new YoDouble("rollOffset", registry);

   public FootstepCostCalculator(DefaultFootstepPlannerParametersBasics parameters,
                                 FootstepSnapperReadOnly snapper,
                                 ToDoubleFunction<FootstepGraphNode> heuristics,
                                 YoRegistry parentRegistry,
                                 FootstepCheckerInterface nodeChecker,
                                 WaypointDefinedBodyPathPlanHolder bodyPathPlanHolder,
                                 FootstepPlannerEnvironmentHandler environmentHandler)
   {
      this.footstepPlannerParameters = parameters;
      this.snapper = snapper;
      this.heuristics = heuristics;

      parentRegistry.addChild(registry);

      this.nodeChecker = nodeChecker;
      this.bodyPathPlanHolder = bodyPathPlanHolder;
      this.environmentHandler = environmentHandler;

      for (RobotSide robotSide : RobotSide.values)
      {
         idealStepLengths.put(robotSide, new YoDouble(robotSide.getShortLowerCaseName() + "_IdealStepLength", registry));
         idealStepWidths.put(robotSide, new YoDouble(robotSide.getShortLowerCaseName() + "_IdealStepWidth", registry));
      }

      correctiveDistanceX = new YoDouble("correctiveDistanceX", registry);
      correctiveDistanceY = new YoDouble("correctiveDistanceY", registry);
      correctiveYaw = new YoDouble("correctiveYaw", registry);

      idealStepYaw = new YoDouble("idealStepYaw", registry);
      yawMode = new YoEnum<>("idealStepYawMode", registry, FootstepCostCalculator.IdealStepMode.class);
      stepMode = new YoEnum<>("idealStepPositionMode", registry, FootstepCostCalculator.IdealStepMode.class);
      desiredRelativeHeading = new YoDouble("desiredRelativeHeading", registry);

      stepSideIncorrect = new YoBoolean("stepSideIncorrect", registry);
   }

   public void initialize(SideDependentList<DiscreteFootstep> goalSteps)
   {
      this.goalSteps = goalSteps;

      idealStepMap.clear();
      pathLength = bodyPathPlanHolder.computePathLength(0.0);

      Pose2D leftGoalPose = new Pose2D(goalSteps.get(RobotSide.LEFT).getX(), goalSteps.get(RobotSide.LEFT).getY(), goalSteps.get(RobotSide.LEFT).getYaw());
      Pose2D rightGoalPose = new Pose2D(goalSteps.get(RobotSide.RIGHT).getX(), goalSteps.get(RobotSide.RIGHT).getY(), goalSteps.get(RobotSide.RIGHT).getYaw());
      goalMidFootPose.interpolate(leftGoalPose, rightGoalPose, 0.5);
      computeAdjustedIdealStepParameters();
   }

   private void computeAdjustedIdealStepParameters()
   {
      FootstepPlanHeading primaryDirection = FootstepPlanHeading.FORWARD;
      double closestAngle = Double.MAX_VALUE;
      for (FootstepPlanHeading heading : FootstepPlanHeading.values())
      {
         double deltaAngle = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(desiredRelativeHeading.getValue(), heading.getYawOffset()));
         if (Math.abs(deltaAngle) < closestAngle)
         {
            closestAngle = deltaAngle;
            primaryDirection = heading;
         }
      }

      FootstepPlanHeading secondaryDirection = FootstepPlanHeading.FORWARD;
      double secondClosestAngle = Double.MAX_VALUE;
      for (FootstepPlanHeading heading : FootstepPlanHeading.values())
      {
         if (heading == primaryDirection)
            continue;

         double deltaAngle = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(desiredRelativeHeading.getValue(), heading.getYawOffset()));
         if (Math.abs(deltaAngle) < secondClosestAngle)
         {
            secondClosestAngle = deltaAngle;
            secondaryDirection = heading;
         }
      }

      double thetaFromPrimary = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(desiredRelativeHeading.getValue(), primaryDirection.getYawOffset()));
      double percentageSecondary = thetaFromPrimary / (0.5 * Math.PI);
      double percentagePrimary = 1.0 - percentageSecondary;

      for (RobotSide stanceSide : RobotSide.values)
      {
         double stepLength0 = getIdealStepLength(footstepPlannerParameters, primaryDirection);
         double stepWidth0 = getIdealStepWidth(footstepPlannerParameters, primaryDirection, stanceSide);
         double stepLength1 = getIdealStepLength(footstepPlannerParameters, secondaryDirection);
         double stepWidth1 = getIdealStepWidth(footstepPlannerParameters, secondaryDirection, stanceSide);

         idealStepLengths.get(stanceSide).set(percentagePrimary * stepLength0 + percentageSecondary * stepLength1);
         idealStepWidths.get(stanceSide).set(percentagePrimary * stepWidth0 + percentageSecondary * stepWidth1);
      }
   }

   private static double getIdealStepLength(DefaultFootstepPlannerParametersReadOnly parameters, FootstepPlanHeading heading)
   {
      switch (heading)
      {
         case LEFT:
         case RIGHT:
            return 0.0;
         case BACKWARD:
            return - parameters.getIdealBackStepLength();
         case FORWARD:
         default:
            return parameters.getIdealFootstepLength();
      }
   }

   private static double getIdealStepWidth(DefaultFootstepPlannerParametersReadOnly parameters, FootstepPlanHeading heading, RobotSide stanceSide)
   {
      switch (heading)
      {
         case LEFT:
            return stanceSide == RobotSide.LEFT ? - parameters.getMaxStepWidth() : parameters.getMinStepWidth();
         case RIGHT:
            return stanceSide == RobotSide.LEFT ? - parameters.getMinStepWidth() : parameters.getMaxStepWidth();
         case BACKWARD:
         case FORWARD:
         default:
            return stanceSide.negateIfLeftSide(parameters.getIdealSideStepWidth());
      }
   }

   @Override
   public double computeCost(DiscreteFootstep candidateStep, DiscreteFootstep stanceStep, DiscreteFootstep startOfSwing)
   {
      DiscreteFootstep idealStep = computeIdealStep(stanceStep, startOfSwing);

      DiscreteFootstepTools.getSnappedStepTransform(stanceStep, snapper.snapFootstep(stanceStep).getSnapTransform(), stanceStepTransform);
      FootstepSnapDataReadOnly candidateSnapData = snapper.snapFootstep(candidateStep);
      DiscreteFootstepTools.getSnappedStepTransform(candidateStep, candidateSnapData.getSnapTransform(), candidateStepTransform);
      idealStepTransform.getTranslation().set(idealStep.getX(), idealStep.getY(), stanceStepTransform.getTranslationZ());
      idealStepTransform.getRotation().setToYawOrientation(idealStep.getYaw());

      // calculate offset from ideal in a z-up frame
      stanceStepTransform.getRotation().setToYawOrientation(stanceStepTransform.getRotation().getYaw());
      idealStepTransform.preMultiplyInvertOther(stanceStepTransform);
      candidateStepTransform.preMultiplyInvertOther(stanceStepTransform);

      xOffset.set(candidateStepTransform.getTranslationX() - idealStepTransform.getTranslationX());
      yOffset.set(candidateStepTransform.getTranslationY() - idealStepTransform.getTranslationY());
      zOffset.set(candidateStepTransform.getTranslationZ() - idealStepTransform.getTranslationZ());
      yawOffset.set(AngleTools.computeAngleDifferenceMinusPiToPi(candidateStepTransform.getRotation().getYaw(), idealStepTransform.getRotation().getYaw()));
      pitchOffset.set(AngleTools.computeAngleDifferenceMinusPiToPi(candidateStepTransform.getRotation().getPitch(),
                                                                   idealStepTransform.getRotation().getPitch()));
      rollOffset.set(AngleTools.computeAngleDifferenceMinusPiToPi(candidateStepTransform.getRotation().getRoll(), idealStepTransform.getRotation().getRoll()));

      edgeCost.set(0.0);
      edgeCost.add(Math.abs(xOffset.getValue() * footstepPlannerParameters.getForwardWeight()));
      edgeCost.add(Math.abs(yOffset.getValue() * footstepPlannerParameters.getLateralWeight()));
      edgeCost.add(Math.abs(
            zOffset.getValue() * (zOffset.getValue() > 0.0 ? footstepPlannerParameters.getStepUpWeight() : footstepPlannerParameters.getStepDownWeight())));
      edgeCost.add(Math.abs(yawOffset.getValue() * footstepPlannerParameters.getYawWeight()));
      edgeCost.add(Math.abs(pitchOffset.getValue() * footstepPlannerParameters.getPitchWeight()));
      edgeCost.add(Math.abs(rollOffset.getValue() * footstepPlannerParameters.getRollWeight()));

      double rmsError = candidateSnapData.getSnapRMSError();
      if (!Double.isNaN(rmsError))
      {
         double rmsAlpha = EuclidCoreTools.clamp(
               (rmsError - footstepPlannerParameters.getRMSMinErrorToPenalize()) / (footstepPlannerParameters.getRMSErrorThreshold()
                                                                                    - footstepPlannerParameters.getRMSMinErrorToPenalize()), 0.0, 1.0);
         edgeCost.add(rmsAlpha * footstepPlannerParameters.getRMSErrorCost());
      }

      edgeCost.add(computeAreaCost(candidateStep));
      edgeCost.add(footstepPlannerParameters.getCostPerStep());

      // subtract off heuristic cost difference - i.e. ignore difference in goal proximity due to step adjustment
      idealStepHeuristicCost.set(heuristics.applyAsDouble(new FootstepGraphNode(stanceStep, idealStep)));
      heuristicCost.set(heuristics.applyAsDouble(new FootstepGraphNode(stanceStep, candidateStep)));
      double deltaHeuristics = idealStepHeuristicCost.getDoubleValue() - heuristicCost.getDoubleValue();

      if (deltaHeuristics > 0.0)
      {
         edgeCost.add(deltaHeuristics);
      }
      else
      {
         // TODO
         edgeCost.set(Math.max(0.0, edgeCost.getValue() - deltaHeuristics));
      }

      totalCost.set(edgeCost.getDoubleValue() + heuristicCost.getDoubleValue());
      return edgeCost.getValue();
   }

   private double computeAreaCost(DiscreteFootstep footstep)
   {
      FootstepSnapDataReadOnly snapData = snapper.snapFootstep(footstep);
      if (snapData != null)
      {
         double areaFraction = snapData.getSnapAreaFraction();
         if (Double.isNaN(areaFraction))
         {
            return 0.0;
         }
         double percentAreaUnoccupied = Math.max(0.0, 1.0 - areaFraction);
         return percentAreaUnoccupied * footstepPlannerParameters.getFootholdAreaWeight();
      }
      else
      {
         return 0.0;
      }
   }

   public DiscreteFootstep computeIdealStep(DiscreteFootstep stanceNode, DiscreteFootstep startOfSwing)
   {
      nominalIdealStep = idealStepMap.computeIfAbsent(stanceNode, this::computeIdealStanceInternal);

      double referencePlanAlpha = footstepPlannerParameters.getReferencePlanAlpha();
      PlannedFootstep referenceFootstep = getReferenceStep(new FootstepGraphNode(startOfSwing, stanceNode));
      if (referenceFootstep == null || referencePlanAlpha == 0.0)
      {
         return nominalIdealStep;
      }
      else
      {
         return computeIdealStepWithReferencePlan(stanceNode, referenceFootstep, referencePlanAlpha);
      }
   }

   public PlannedFootstep getReferenceStep(FootstepGraphNode graphNode)
   {
      if (referenceFootstepPlan == null || referenceFootstepPlan.isEmpty())
         return null;

      List<FootstepGraphNode> pathFromStart = footstepGraph.getPathFromStart(graphNode);
      int stepIndexInPlan = pathFromStart.size() - 1;
      if (stepIndexInPlan < referenceFootstepPlan.getNumberOfSteps())
      {
         return referenceFootstepPlan.getFootstep(stepIndexInPlan);
      }
      else
      {
         return null;
      }
   }

   @NotNull
   private DiscreteFootstep computeIdealStepWithReferencePlan(DiscreteFootstep stanceNode, PlannedFootstep referenceFootstep, double referencePlanAlpha)
   {
      // TODO: Indexing should match so that referencedStep's side is equal to stance Node's opposite side.
      // In case this is not true, we will try to find the closest step from either previous or next step in the reference plan.
      stepSideIncorrect.set(referenceFootstep.getRobotSide() != stanceNode.getRobotSide().getOppositeSide());
      if (stepSideIncorrect.getBooleanValue())
      {
         throw new RuntimeException("Invalid side on reference plan");
      }

      Pose2D nominalStepPose = new Pose2D(nominalIdealStep.getX(), nominalIdealStep.getY(), nominalIdealStep.getYaw());
      Pose2D referenceStepPose = new Pose2D(referenceFootstep.getFootstepPose());
      Pose2D interpolatedPose = new Pose2D(nominalStepPose);
      interpolatedPose.interpolate(referenceStepPose, referencePlanAlpha);
      return new DiscreteFootstep(interpolatedPose.getX(), interpolatedPose.getY(), interpolatedPose.getYaw(), referenceFootstep.getRobotSide());
   }

   private DiscreteFootstep computeIdealStanceInternal(DiscreteFootstep stanceStep)
   {
      // If goal node is reachable, it's the ideal step
      DiscreteFootstep goalStep = goalSteps.get(stanceStep.getRobotSide().getOppositeSide());
      if (!environmentHandler.flatGroundMode() && nodeChecker.isStepValid(goalStep, stanceStep, null))
      {
         return goalStep;
      }

      Point2D midFootPoint = stanceStep.getOrComputeMidFootPoint(footstepPlannerParameters.getIdealFootstepWidth());
      RobotSide stanceSide = stanceStep.getRobotSide();
      double alphaMidFoot = bodyPathPlanHolder.getClosestPoint(midFootPoint, projectionPose);
      int segmentIndex = bodyPathPlanHolder.getSegmentIndexFromAlpha(alphaMidFoot);
      bodyPathPlanHolder.getPointAlongPath(alphaMidFoot, projectionPose);

      double desiredRobotPostureHeading = bodyPathPlanHolder.getBodyPathPlan().getWaypoint(segmentIndex).getOrientation().getYaw();
      double desiredRobotMotionHeading = bodyPathPlanHolder.getSegmentYaw(segmentIndex);
      desiredRelativeHeading.set(EuclidCoreTools.angleDifferenceMinusPiToPi(desiredRobotPostureHeading, desiredRobotMotionHeading));

      double distanceFromGoalSquared = midFootPoint.distanceSquared(goalMidFootPose.getPosition());
      double distanceFromPathSquared = midFootPoint.distanceXYSquared(projectionPose.getPosition());
      double finalTurnProximity = footstepPlannerParameters.getFinalTurnProximity();

      // Calculate target yaw:
      // 1) Match goal if close to goal
      // 2) Match body path if close to path
      // 3) Turn towards body path if off path

      double desiredYaw;
      if (distanceFromGoalSquared < MathTools.square(finalTurnProximity))
      {
         yawMode.set(IdealStepMode.GOAL);
         desiredYaw = goalMidFootPose.getYaw();
      }
      else if (distanceFromPathSquared < MathTools.square(footstepPlannerParameters.getDistanceFromPathTolerance()))
      {
         yawMode.set(IdealStepMode.ON_PATH);
         desiredYaw = EuclidCoreTools.trimAngleMinusPiToPi(bodyPathPlanHolder.getSegmentYaw(segmentIndex) + desiredRelativeHeading.getValue());
      }
      else
      {
         yawMode.set(IdealStepMode.TOWARDS_PATH);

         int numberOfCorrectiveStepsWhenOffPath = 2;
         double alphaLookAhead = MathTools.clamp(
               alphaMidFoot + numberOfCorrectiveStepsWhenOffPath * footstepPlannerParameters.getIdealFootstepLength() / pathLength, 0.0, 1.0);
         bodyPathPlanHolder.getPointAlongPath(alphaLookAhead, projectionPose);
         desiredYaw = Math.atan2(projectionPose.getY() - midFootPoint.getY(), projectionPose.getX() - midFootPoint.getX());
         desiredYaw = EuclidCoreTools.trimAngleMinusPiToPi(desiredYaw + desiredRelativeHeading.getValue());
      }

      // Clamp target yaw to what's achievable by step footstepPlannerParameters
      idealStepYaw.set(desiredYaw);
      double deltaYaw = AngleTools.computeAngleDifferenceMinusPiToPi(desiredYaw, stanceStep.getYaw());
      RobotSide stepSide = stanceSide.getOppositeSide();
      double yawLowerLimit = stepSide == RobotSide.LEFT ? footstepPlannerParameters.getMinStepYaw() : -footstepPlannerParameters.getMaxStepYaw();
      double yawUpperLimit = stepSide == RobotSide.LEFT ? footstepPlannerParameters.getMaxStepYaw() : -footstepPlannerParameters.getMinStepYaw();
      double achievableStepYaw = MathTools.clamp(deltaYaw, yawLowerLimit, yawUpperLimit);

      // Calculate step positions:
      // 1) If close to goal, match the goal's mid-foot position and turn towards goal orientation
      // 2) If facing too far from body path orientation, turn in place to match body path heading
      //          (this avoids the blended step and turn that is unnatural if yaw difference is too large)
      // 3) Otherwise take a step given the ideal step footstepPlannerParameters. Append a correction so that ideal step shifts towards path

      if (distanceFromGoalSquared <= MathTools.square(finalTurnProximity))
      {
         // turn in place at goal
         stepMode.set(IdealStepMode.GOAL);
         return turnInPlaceStep(stanceStep,
                                goalMidFootPose.getPosition(),
                                stanceSide,
                                0.5 * footstepPlannerParameters.getIdealFootstepWidth(),
                                achievableStepYaw);
      }
      else if (Math.abs(deltaYaw) > footstepPlannerParameters.getDeltaYawFromReferenceTolerance())
      {
         // turn in place towards goal
         stepMode.set(IdealStepMode.TOWARDS_PATH);
         return turnInPlaceStep(stanceStep, midFootPoint, stanceSide, 0.5 * footstepPlannerParameters.getIdealFootstepWidth(), achievableStepYaw);
      }
      else
      {
         stepMode.set(IdealStepMode.ON_PATH);
         double idealStepLength = idealStepLengths.get(stanceSide).getDoubleValue();
         double idealStepWidth = idealStepWidths.get(stanceSide).getDoubleValue();

         stanceFootPose.set(stanceStep.getX(), stanceStep.getY(), stanceStep.getYaw());
         idealStep.set(stanceFootPose);
         idealStep.appendTranslation(idealStepLength, idealStepWidth);

         idealMidFootPose.set(idealStep);
         idealMidFootPose.appendTranslation(0.0, stepSide.negateIfLeftSide(0.5 * footstepPlannerParameters.getIdealFootstepWidth()));
         calculateCorrectiveValues(idealMidFootPose);

         idealStep.getPosition().add(correctiveDistanceX.getDoubleValue(), correctiveDistanceY.getDoubleValue());
         idealStep.appendRotation(correctiveYaw.getDoubleValue());

         return new DiscreteFootstep(idealStep.getX(), idealStep.getY(), idealStep.getYaw(), stanceStep.getRobotSide().getOppositeSide());
      }
   }

   private static DiscreteFootstep turnInPlaceStep(DiscreteFootstep startStep,
                                                   Point2DBasics midFootPoint,
                                                   RobotSide stanceSide,
                                                   double idealFootstepWidth,
                                                   double turnYaw)
   {
      Pose2D idealStep = new Pose2D(midFootPoint, startStep.getYaw());
      idealStep.appendRotation(turnYaw);
      idealStep.appendTranslation(0.0, stanceSide.negateIfLeftSide(idealFootstepWidth));
      return new DiscreteFootstep(idealStep.getX(), idealStep.getY(), idealStep.getYaw(), startStep.getRobotSide().getOppositeSide());
   }

   private void calculateCorrectiveValues(Pose2D midFootPose)
   {
      double alpha = bodyPathPlanHolder.getClosestPoint(midFootPose.getPosition(), projectionPose);

      Vector2D toClosestPointOnPath = new Vector2D();
      toClosestPointOnPath.set(projectionPose.getX(), projectionPose.getY());
      toClosestPointOnPath.sub(midFootPose.getX(), midFootPose.getY());

      double distanceToPath = toClosestPointOnPath.length();
      if (distanceToPath > maxDistanceAdjustmentTowardsPath)
      {
         toClosestPointOnPath.scale(maxDistanceAdjustmentTowardsPath / distanceToPath);
      }

      correctiveDistanceX.set(toClosestPointOnPath.getX());
      correctiveDistanceY.set(toClosestPointOnPath.getY());

      double pathYaw = bodyPathPlanHolder.getSegmentYaw(bodyPathPlanHolder.getSegmentIndexFromAlpha(alpha)) + desiredRelativeHeading.getValue();
      double currentYaw = midFootPose.getYaw();
      correctiveYaw.set(MathTools.clamp(AngleTools.computeAngleDifferenceMinusPiToPi(pathYaw, currentYaw), maxYawAdjustmentTowardsPath));
   }

   public void setFootstepGraph(DirectedGraph<FootstepGraphNode> footstepGraph)
   {
      this.footstepGraph = footstepGraph;
   }

   public DiscreteFootstep getNominalIdealStep()
   {
      return nominalIdealStep;
   }

   public void setReferenceFootstepPlan(FootstepPlan referenceFootstepPlan)
   {
      this.referenceFootstepPlan = referenceFootstepPlan;
   }

   public void resetLoggedVariables()
   {
      edgeCost.setToNaN();
      totalCost.setToNaN();
      heuristicCost.setToNaN();
   }
}
