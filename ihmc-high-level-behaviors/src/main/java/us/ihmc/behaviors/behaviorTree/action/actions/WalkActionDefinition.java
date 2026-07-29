package us.ihmc.behaviors.behaviorTree.action.actions;

import static behavior_msgs.WalkActionDefinitionMessage.*;

import behavior_msgs.WalkActionDefinitionMessage;
import behavior_msgs.WalkActionFootstepDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.BooleanNode;
import com.fasterxml.jackson.databind.node.IntNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import com.fasterxml.jackson.databind.node.TextNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeDefinition;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeDefinition;
import us.ihmc.behaviors.tools.BehaviorStoredPropertySetDefinition;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.crdt.*;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.graphSearch.parameters.InitialStanceSide;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.JSONTools;

import java.util.ArrayList;

public class WalkActionDefinition extends ActionNodeDefinition
{
   private final CRDTBidirectionalDouble swingDuration;
   private final CRDTBidirectionalDouble transferDuration;
   private final CRDTBidirectionalEnumField<ExecutionMode> executionMode;
   private final CRDTBidirectionalString parentFrameName;
   private final CRDTBidirectionalBoolean isManuallyPlaced;
   private final CRDTBidirectionalRecyclingArrayList<Pose3D> waypoints;
   private final CRDTBidirectionalRecyclingArrayList<WalkActionFootstepDefinition> manuallyPlacedFootsteps;
   private final CRDTBidirectionalPoint3D goalStancePoint;
   private final CRDTBidirectionalPoint3D goalFocalPoint;
   private final SideDependentList<CRDTBidirectionalDouble> goalFootstepToGoalXs;
   private final SideDependentList<CRDTBidirectionalDouble> goalFootstepToGoalYs;
   private final SideDependentList<CRDTBidirectionalDouble> goalFootstepToGoalYaws;
   private final CRDTBidirectionalImmutableField<InitialStanceSide> plannerInitialStanceSide;
   private final CRDTBidirectionalInteger plannerType;
   private final CRDTBidirectionalBoolean plannerWalkWithGoalOrientation;
   private final CRDTBidirectionalBoolean plannerPlanWithBodyPath;
   private final BehaviorStoredPropertySetDefinition plannerParameters;
   private final CRDTBidirectionalBoolean quickWaypointOnly;
   private final CRDTBidirectionalBoolean useRRTPathPlanner;
   private final CRDTBidirectionalDouble obstacleClearanceRadius;
   private final CRDTBidirectionalDouble quickHipWidth;
   private final CRDTBidirectionalDouble quickStepLength;
   private final CRDTBidirectionalDouble quickNextPelvisYawLimit;
   private final CRDTBidirectionalDouble quickInwardLimit;
   private final CRDTBidirectionalDouble quickOutwardLimit;
   private final CRDTBidirectionalDouble quickStepAngleLimit;
   private final CRDTBidirectionalDouble quickSwingTimeDistanceLower;
   private final CRDTBidirectionalDouble quickSwingTimeDistanceUpper;
   private final CRDTBidirectionalDouble quickMinSwingTime;
   private final CRDTBidirectionalDouble quickMaxSwingTime;

   // On disk fields
   private double onDiskSwingDuration;
   private double onDiskTransferDuration;
   private ExecutionMode onDiskExecutionMode;
   private String onDiskParentFrameName;
   private boolean onDiskIsManuallyPlaced;
   private final ArrayList<Pose3D> onDiskWaypoints = new ArrayList<>();
   private int onDiskNumberOfFootsteps;
   private final Point3D onDiskGoalStancePoint = new Point3D();
   private final Point3D onDiskGoalFocalPoint = new Point3D();
   private final SideDependentList<Double> onDiskGoalFootstepToGoalXs = new SideDependentList<>(() -> 0.0);
   private final SideDependentList<Double> onDiskGoalFootstepToGoalYs = new SideDependentList<>(() -> 0.0);
   private final SideDependentList<Double> onDiskGoalFootstepToGoalYaws = new SideDependentList<>(() -> 0.0);
   private InitialStanceSide onDiskPlannerInitialStanceSide;
   private int onDiskPlannerType;
   private boolean onDiskPlannerWalkWithGoalOrientation;
   private boolean onDiskPlannerPlanWithBodyPath;
   private boolean onDiskQuickWaypointOnly;
   private boolean onDiskUseRRTPathPlanner;
   private double onDiskObstacleClearanceRadius;
   private double onDiskQuickHipWidth;
   private double onDiskQuickStepLength;
   private double onDiskQuickNextPelvisYawLimit;
   private double onDiskQuickInwardLimit;
   private double onDiskQuickOutwardLimit;
   private double onDiskQuickStepAngleLimit;
   private double onDiskQuickSwingTimeDistanceLower;
   private double onDiskQuickSwingTimeDistanceUpper;
   private double onDiskQuickMinSwingTime;
   private double onDiskQuickMaxSwingTime;

   public WalkActionDefinition(BehaviorTreeRootNodeDefinition rootNode)
   {
      super(rootNode);

      swingDuration = new CRDTBidirectionalDouble(this, 0.8);
      transferDuration = new CRDTBidirectionalDouble(this, 0.5);
      executionMode = new CRDTBidirectionalEnumField<>(this, ExecutionMode.OVERRIDE);
      parentFrameName = new CRDTBidirectionalString(this, "Walking");
      isManuallyPlaced = new CRDTBidirectionalBoolean(this, false);
      waypoints = new CRDTBidirectionalRecyclingArrayList<>(this, new RecyclingArrayList<>(Pose3D::new));
      manuallyPlacedFootsteps = new CRDTBidirectionalRecyclingArrayList<>(this, new RecyclingArrayList<>(() -> new WalkActionFootstepDefinition(this)));
      goalStancePoint = new CRDTBidirectionalPoint3D(this);
      goalFocalPoint = new CRDTBidirectionalPoint3D(this);
      goalFootstepToGoalXs = new SideDependentList<>(() -> new CRDTBidirectionalDouble(this, 0.0));
      goalFootstepToGoalYs = new SideDependentList<>(() -> new CRDTBidirectionalDouble(this, 0.0));
      goalFootstepToGoalYaws = new SideDependentList<>(() -> new CRDTBidirectionalDouble(this, 0.0));
      plannerInitialStanceSide = new CRDTBidirectionalImmutableField<>(this, InitialStanceSide.AUTO);
      plannerType = new CRDTBidirectionalInteger(this, QUICK);
      plannerWalkWithGoalOrientation = new CRDTBidirectionalBoolean(this, true);
      plannerPlanWithBodyPath = new CRDTBidirectionalBoolean(this, false);
      plannerParameters = new BehaviorStoredPropertySetDefinition(this, "plannerParameters", robotModel.getFootstepPlannerParameters());
      quickWaypointOnly = new CRDTBidirectionalBoolean(this, false);
      useRRTPathPlanner = new CRDTBidirectionalBoolean(this, false);
      obstacleClearanceRadius = new CRDTBidirectionalDouble(this, 0.5);
      quickHipWidth = new CRDTBidirectionalDouble(this, 0.12);
      quickStepLength = new CRDTBidirectionalDouble(this, 0.28);
      quickNextPelvisYawLimit = new CRDTBidirectionalDouble(this, Math.toRadians(35.0));
      quickInwardLimit = new CRDTBidirectionalDouble(this, Math.toRadians(10.0));
      quickOutwardLimit = new CRDTBidirectionalDouble(this, Math.toRadians(50.0));
      quickStepAngleLimit = new CRDTBidirectionalDouble(this, Math.toRadians(115.0));
      quickSwingTimeDistanceLower = new CRDTBidirectionalDouble(this, 0.3);
      quickSwingTimeDistanceUpper = new CRDTBidirectionalDouble(this, 0.7);
      quickMinSwingTime = new CRDTBidirectionalDouble(this, 0.8);
      quickMaxSwingTime = new CRDTBidirectionalDouble(this, 1.2);
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("swingDuration", swingDuration.getValue());
      jsonNode.put("transferDuration", transferDuration.getValue());
      jsonNode.put("executionMode", executionMode.getValue().name());
      jsonNode.put("parentFrame", parentFrameName.getValue());

      if (isManuallyPlaced.getValue())
      {
         ArrayNode foostepsArrayNode = jsonNode.putArray("footsteps");
         for (int i = 0; i < manuallyPlacedFootsteps.getSize(); i++)
         {
            ObjectNode footstepNode = foostepsArrayNode.addObject();
            manuallyPlacedFootsteps.getValueReadOnly(i).saveToFile(footstepNode);
         }
      }
      else
      {
         JSONTools.toJSON(jsonNode.putObject("goalStancePoint"), goalStancePoint.getValueReadOnly());
         JSONTools.toJSON(jsonNode.putObject("goalFocalPoint"), goalFocalPoint.getValueReadOnly());

         for (RobotSide side : RobotSide.values)
         {
            ObjectNode goalFootNode = jsonNode.putObject(side.getCamelCaseName() + "GoalFootToGoal");
            goalFootNode.put("x", JSONTools.toJsonMeters(goalFootstepToGoalXs.get(side).getValue()));
            goalFootNode.put("y", JSONTools.toJsonMeters(goalFootstepToGoalYs.get(side).getValue()));
            goalFootNode.put("yawInDegrees", JSONTools.toJsonRadians(goalFootstepToGoalYaws.get(side).getValue()));
         }
         jsonNode.put("planner", switch (plannerType.getValue())
         {
            case TURN_WALK_TURN -> "TURN_WALK_TURN";
            case A_STAR -> "A_STAR";
            default -> "QUICK";
         });
         if (plannerType.getValue() == QUICK)
         {
            if (waypoints.getSize() > 0)
            {
               ArrayNode waypointsArray = jsonNode.putArray("waypoints");
               for (int i = 0; i < waypoints.getSize(); i++)
                  JSONTools.toJSON(waypointsArray.addObject(), waypoints.getValueReadOnly(i));
            }
            if (quickWaypointOnly.getValue())
               jsonNode.put("quickWaypointOnly", true);
            if (useRRTPathPlanner.getValue())
            {
               jsonNode.put("useRrtPathPlanner", true);
               if (Math.abs(obstacleClearanceRadius.getValue() - 0.5) > 0.005)
                  jsonNode.put("obstacleClearanceRadius", obstacleClearanceRadius.getValue());
            }
            if (Math.abs(quickHipWidth.getValue() - 0.12) > 0.005)
               jsonNode.put("quickHipWidth", quickHipWidth.getValue());
            if (Math.abs(quickStepLength.getValue() - 0.28) > 0.005)
               jsonNode.put("quickStepLength", quickStepLength.getValue());
            if (Math.abs(quickNextPelvisYawLimit.getValue() - Math.toRadians(35.0)) > Math.toRadians(0.5))
               jsonNode.put("quickNextPelvisYawLimit", Math.toDegrees(quickNextPelvisYawLimit.getValue()));
            if (Math.abs(quickInwardLimit.getValue() - Math.toRadians(10.0)) > Math.toRadians(0.5))
               jsonNode.put("quickInwardLimit", Math.toDegrees(quickInwardLimit.getValue()));
            if (Math.abs(quickOutwardLimit.getValue() - Math.toRadians(50.0)) > Math.toRadians(0.5))
               jsonNode.put("quickOutwardLimit", Math.toDegrees(quickOutwardLimit.getValue()));
            if (Math.abs(quickStepAngleLimit.getValue() - Math.toRadians(115.0)) > Math.toRadians(0.5))
               jsonNode.put("quickStepAngleLimit", Math.toDegrees(quickStepAngleLimit.getValue()));
            if (Math.abs(quickSwingTimeDistanceLower.getValue() - 0.3) > 0.005)
               jsonNode.put("quickSwingTimeDistanceLower", quickSwingTimeDistanceLower.getValue());
            if (Math.abs(quickSwingTimeDistanceUpper.getValue() - 0.7) > 0.005)
               jsonNode.put("quickSwingTimeDistanceUpper", quickSwingTimeDistanceUpper.getValue());
            if (Math.abs(quickMinSwingTime.getValue() - 0.8) > 1.0e-3)
               jsonNode.put("quickMinSwingTime", quickMinSwingTime.getValue());
            if (Math.abs(quickMaxSwingTime.getValue() - 1.2) > 1.0e-3)
               jsonNode.put("quickMaxSwingTime", quickMaxSwingTime.getValue());
         }
         else
         {
            jsonNode.put("plannerInitialStanceSide", plannerInitialStanceSide.getValue().name());
            jsonNode.put("plannerWalkWithGoalOrientation", plannerWalkWithGoalOrientation.getValue());
            jsonNode.put("plannerPlanWithBodyPath", plannerPlanWithBodyPath.getValue());
         }
         plannerParameters.toJSON(jsonNode);
      }
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      swingDuration.setValue(jsonNode.get("swingDuration").asDouble());
      transferDuration.setValue(jsonNode.get("transferDuration").asDouble());
      executionMode.setValue(ExecutionMode.valueOf(jsonNode.get("executionMode").textValue()));
      parentFrameName.setValue(jsonNode.get("parentFrame").textValue());
      isManuallyPlaced.setValue(jsonNode.get("footsteps") != null);

      manuallyPlacedFootsteps.getValueAndModify().clear();
      waypoints.getValueAndModify().clear();
      if (isManuallyPlaced.getValue())
         JSONTools.forEachArrayElement(jsonNode, "footsteps", footstepNode -> manuallyPlacedFootsteps.getValueAndModify().add().loadFromFile(footstepNode));
      else
      {
         if (jsonNode.get("waypoints") != null)
            JSONTools.forEachArrayElement(jsonNode, "waypoints", waypointNode ->
            {
               JSONTools.toEuclid(waypointNode, waypoints.getValueAndModify().add());
            });
         if (jsonNode.get("goalStancePoint") instanceof ObjectNode objectNode)
            JSONTools.toEuclid(objectNode, goalStancePoint.getValueAndModify());
         if (jsonNode.get("goalFocalPoint") instanceof ObjectNode objectNode)
            JSONTools.toEuclid(objectNode, goalFocalPoint.getValueAndModify());

         for (RobotSide side : RobotSide.values)
         {
            ObjectNode goalFootNode = (ObjectNode) jsonNode.get(side.getCamelCaseName() + "GoalFootToGoal");
            goalFootstepToGoalXs.get(side).setValue(goalFootNode.get("x").asDouble());
            goalFootstepToGoalYs.get(side).setValue(goalFootNode.get("y").asDouble());
            goalFootstepToGoalYaws.get(side).setValue(Math.toRadians(goalFootNode.get("yawInDegrees").asDouble()));
         }
         if (jsonNode.get("plannerInitialStanceSide") instanceof TextNode textNode)
            plannerInitialStanceSide.setValue(InitialStanceSide.valueOf(textNode.textValue()));
         if (jsonNode.get("plannerPerformAStarSearch") instanceof BooleanNode booleanNode)
            plannerType.setValue(booleanNode.booleanValue() ? A_STAR : QUICK);
         if (jsonNode.get("planner") != null)
         {
            if (jsonNode.get("planner") instanceof IntNode intNode)
               plannerType.setValue(intNode.intValue());
            else if (jsonNode.get("planner") instanceof TextNode textNode)
            {
               if (textNode.textValue().equals("QUICK"))
                  plannerType.setValue(QUICK);
               if (textNode.textValue().equals("TURN_WALK_TURN"))
                  plannerType.setValue(TURN_WALK_TURN);
               if (textNode.textValue().equals("A_STAR"))
                  plannerType.setValue(A_STAR);
            }
         }
         if (jsonNode.get("plannerWalkWithGoalOrientation") instanceof BooleanNode booleanNode)
            plannerWalkWithGoalOrientation.setValue(booleanNode.booleanValue());
         if (jsonNode.get("plannerPlanWithBodyPath") instanceof BooleanNode booleanNode)
            plannerPlanWithBodyPath.setValue(booleanNode.booleanValue());
         if (plannerType.getValue() == QUICK)
         {
            if (jsonNode.get("quickWaypointOnly") != null)
               quickWaypointOnly.setValue(jsonNode.get("quickWaypointOnly").asBoolean());
            if (jsonNode.get("useRrtPathPlanner") != null)
               useRRTPathPlanner.setValue(jsonNode.get("useRrtPathPlanner").asBoolean());
            if (jsonNode.get("obstacleClearanceRadius") != null)
               obstacleClearanceRadius.setValue(jsonNode.get("obstacleClearanceRadius").asDouble());
            if (jsonNode.get("quickHipWidth") != null)
               quickHipWidth.setValue(jsonNode.get("quickHipWidth").asDouble());
            if (jsonNode.get("quickStepLength") != null)
               quickStepLength.setValue(jsonNode.get("quickStepLength").asDouble());
            if (jsonNode.get("quickNextPelvisYawLimit") != null)
               quickNextPelvisYawLimit.setValue(Math.toRadians(jsonNode.get("quickNextPelvisYawLimit").asDouble()));
            if (jsonNode.get("quickInwardLimit") != null)
               quickInwardLimit.setValue(Math.toRadians(jsonNode.get("quickInwardLimit").asDouble()));
            if (jsonNode.get("quickOutwardLimit") != null)
               quickOutwardLimit.setValue(Math.toRadians(jsonNode.get("quickOutwardLimit").asDouble()));
            if (jsonNode.get("quickStepAngleLimit") != null)
               quickStepAngleLimit.setValue(Math.toRadians(jsonNode.get("quickStepAngleLimit").asDouble()));
            if (jsonNode.get("quickSwingTimeDistanceLower") != null)
               quickSwingTimeDistanceLower.setValue(jsonNode.get("quickSwingTimeDistanceLower").asDouble());
            if (jsonNode.get("quickSwingTimeDistanceUpper") != null)
               quickSwingTimeDistanceUpper.setValue(jsonNode.get("quickSwingTimeDistanceUpper").asDouble());
            if (jsonNode.get("quickMinSwingTime") != null)
               quickMinSwingTime.setValue(jsonNode.get("quickMinSwingTime").asDouble());
            if (jsonNode.get("quickMaxSwingTime") != null)
               quickMaxSwingTime.setValue(jsonNode.get("quickMaxSwingTime").asDouble());
         }
         plannerParameters.fromJSON(jsonNode);
      }
   }

   @Override
   public void setOnDiskFields()
   {
      super.setOnDiskFields();

      onDiskSwingDuration = swingDuration.getValue();
      onDiskTransferDuration = transferDuration.getValue();
      onDiskExecutionMode = executionMode.getValue();
      onDiskParentFrameName = parentFrameName.getValue();
      onDiskIsManuallyPlaced = isManuallyPlaced.getValue();
      onDiskWaypoints.clear();
      for (int i = 0; i < waypoints.getSize(); i++)
         onDiskWaypoints.add(new Pose3D(waypoints.getValueReadOnly(i)));
      onDiskNumberOfFootsteps = manuallyPlacedFootsteps.getSize();
      onDiskGoalStancePoint.set(goalStancePoint.getValueReadOnly());
      onDiskGoalFocalPoint.set(goalFocalPoint.getValueReadOnly());
      for (RobotSide side : goalFootstepToGoalXs.sides())
      {
         onDiskGoalFootstepToGoalXs.put(side, goalFootstepToGoalXs.get(side).getValue());
         onDiskGoalFootstepToGoalYs.put(side, goalFootstepToGoalYs.get(side).getValue());
         onDiskGoalFootstepToGoalYaws.put(side, goalFootstepToGoalYaws.get(side).getValue());
      }

      for (int i = 0; i < manuallyPlacedFootsteps.getSize(); i++)
         manuallyPlacedFootsteps.getValueReadOnly(i).setOnDiskFields();
      onDiskPlannerInitialStanceSide = plannerInitialStanceSide.getValue();
      onDiskPlannerType = plannerType.getValue();
      onDiskPlannerWalkWithGoalOrientation = plannerWalkWithGoalOrientation.getValue();
      onDiskPlannerPlanWithBodyPath = plannerPlanWithBodyPath.getValue();
      onDiskQuickWaypointOnly = quickWaypointOnly.getValue();
      onDiskUseRRTPathPlanner = useRRTPathPlanner.getValue();
      onDiskObstacleClearanceRadius = obstacleClearanceRadius.getValue();
      onDiskQuickHipWidth = quickHipWidth.getValue();
      onDiskQuickStepLength = quickStepLength.getValue();
      onDiskQuickNextPelvisYawLimit = quickNextPelvisYawLimit.getValue();
      onDiskQuickInwardLimit = quickInwardLimit.getValue();
      onDiskQuickOutwardLimit = quickOutwardLimit.getValue();
      onDiskQuickStepAngleLimit = quickStepAngleLimit.getValue();
      onDiskQuickSwingTimeDistanceLower = quickSwingTimeDistanceLower.getValue();
      onDiskQuickSwingTimeDistanceUpper = quickSwingTimeDistanceUpper.getValue();
      onDiskQuickMinSwingTime = quickMinSwingTime.getValue();
      onDiskQuickMaxSwingTime = quickMaxSwingTime.getValue();
      plannerParameters.setOnDiskFields();
   }

   @Override
   public void undoAllNontopologicalChanges()
   {
      super.undoAllNontopologicalChanges();

      if (isUndoAvailable())
      {
         swingDuration.setValue(onDiskSwingDuration);
         transferDuration.setValue(onDiskTransferDuration);
        executionMode.setValue(onDiskExecutionMode);
        parentFrameName.setValue(onDiskParentFrameName);
        isManuallyPlaced.setValue(onDiskIsManuallyPlaced);
         waypoints.getValueAndModify().clear();
         for (Pose3D waypoint : onDiskWaypoints)
            waypoints.getValueAndModify().add().set(waypoint);
        manuallyPlacedFootsteps.getValueAndModify().clear();
         for (int i = 0; i < onDiskNumberOfFootsteps; i++)
            manuallyPlacedFootsteps.getValueAndModify().add();
         goalStancePoint.getValueAndModify().set(onDiskGoalStancePoint);
         goalFocalPoint.getValueAndModify().set(onDiskGoalFocalPoint);
         for (RobotSide side : onDiskGoalFootstepToGoalXs.sides())
         {
            goalFootstepToGoalXs.get(side).setValue(onDiskGoalFootstepToGoalXs.get(side));
            goalFootstepToGoalYs.get(side).setValue(onDiskGoalFootstepToGoalYs.get(side));
            goalFootstepToGoalYaws.get(side).setValue(onDiskGoalFootstepToGoalYaws.get(side));
         }

         for (int i = 0; i < manuallyPlacedFootsteps.getSize(); i++)
            manuallyPlacedFootsteps.getValueAndModify().get(i).undoAllNontopologicalChanges();
         plannerInitialStanceSide.setValue(onDiskPlannerInitialStanceSide);
         plannerType.setValue(onDiskPlannerType);
         plannerWalkWithGoalOrientation.setValue(onDiskPlannerWalkWithGoalOrientation);
         plannerPlanWithBodyPath.setValue(onDiskPlannerPlanWithBodyPath);
         quickWaypointOnly.setValue(onDiskQuickWaypointOnly);
         useRRTPathPlanner.setValue(onDiskUseRRTPathPlanner);
         obstacleClearanceRadius.setValue(onDiskObstacleClearanceRadius);
         quickHipWidth.setValue(onDiskQuickHipWidth);
         quickStepLength.setValue(onDiskQuickStepLength);
         quickNextPelvisYawLimit.setValue(onDiskQuickNextPelvisYawLimit);
         quickInwardLimit.setValue(onDiskQuickInwardLimit);
         quickOutwardLimit.setValue(onDiskQuickOutwardLimit);
         quickStepAngleLimit.setValue(onDiskQuickStepAngleLimit);
         quickSwingTimeDistanceLower.setValue(onDiskQuickSwingTimeDistanceLower);
         quickSwingTimeDistanceUpper.setValue(onDiskQuickSwingTimeDistanceUpper);
         quickMinSwingTime.setValue(onDiskQuickMinSwingTime);
         quickMaxSwingTime.setValue(onDiskQuickMaxSwingTime);
         plannerParameters.undoAllNontopologicalChanges();
      }
   }

   @Override
   public boolean hasChanges()
   {
      boolean unchanged = !super.hasChanges();

      unchanged &= swingDuration.getValue() == onDiskSwingDuration;
      unchanged &= transferDuration.getValue() == onDiskTransferDuration;
      unchanged &= executionMode.getValue() == onDiskExecutionMode;
      unchanged &= parentFrameName.getValue().equals(onDiskParentFrameName);
      unchanged &= isManuallyPlaced.getValue() == onDiskIsManuallyPlaced;
      unchanged &= waypoints.getSize() == onDiskWaypoints.size();
      if (unchanged)
         for (int i = 0; i < waypoints.getSize(); i++)
            if (!waypoints.getValueReadOnly(i).equals(onDiskWaypoints.get(i)))
            {
               unchanged = false;
               break;
            }
      unchanged &= goalStancePoint.getValueReadOnly().equals(onDiskGoalStancePoint);
      unchanged &= goalFocalPoint.getValueReadOnly().equals(onDiskGoalFocalPoint);
      for (RobotSide side : goalFootstepToGoalXs.sides())
      {
         unchanged &= goalFootstepToGoalXs.get(side).getValue() == onDiskGoalFootstepToGoalXs.get(side);
         unchanged &= goalFootstepToGoalYs.get(side).getValue() == onDiskGoalFootstepToGoalYs.get(side);
         unchanged &= goalFootstepToGoalYaws.get(side).getValue() == onDiskGoalFootstepToGoalYaws.get(side);
      }

      boolean sameNumberOfFootsteps = manuallyPlacedFootsteps.getSize() == onDiskNumberOfFootsteps;
      unchanged &= sameNumberOfFootsteps;

      if (sameNumberOfFootsteps)
         for (int i = 0; i < manuallyPlacedFootsteps.getSize(); i++)
            unchanged &= !manuallyPlacedFootsteps.getValueReadOnly(i).hasChanges();
      unchanged &= plannerInitialStanceSide.getValue() == onDiskPlannerInitialStanceSide;
      unchanged &= plannerType.getValue() == onDiskPlannerType;
      unchanged &= plannerWalkWithGoalOrientation.getValue() == onDiskPlannerWalkWithGoalOrientation;
      unchanged &= plannerPlanWithBodyPath.getValue() == onDiskPlannerPlanWithBodyPath;
      unchanged &= quickWaypointOnly.getValue() == onDiskQuickWaypointOnly;
      unchanged &= useRRTPathPlanner.getValue() == onDiskUseRRTPathPlanner;
      unchanged &= obstacleClearanceRadius.getValue() == onDiskObstacleClearanceRadius;
      unchanged &= quickHipWidth.getValue() == onDiskQuickHipWidth;
      unchanged &= quickStepLength.getValue() == onDiskQuickStepLength;
      unchanged &= quickNextPelvisYawLimit.getValue() == onDiskQuickNextPelvisYawLimit;
      unchanged &= quickInwardLimit.getValue() == onDiskQuickInwardLimit;
      unchanged &= quickOutwardLimit.getValue() == onDiskQuickOutwardLimit;
      unchanged &= quickStepAngleLimit.getValue() == onDiskQuickStepAngleLimit;
      unchanged &= quickSwingTimeDistanceLower.getValue() == onDiskQuickSwingTimeDistanceLower;
      unchanged &= quickSwingTimeDistanceUpper.getValue() == onDiskQuickSwingTimeDistanceUpper;
      unchanged &= quickMinSwingTime.getValue() == onDiskQuickMinSwingTime;
      unchanged &= quickMaxSwingTime.getValue() == onDiskQuickMaxSwingTime;
      unchanged &= plannerParameters.isUnchanged();

      return !unchanged;
   }

   public void toMessage(WalkActionDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setSwingDuration(swingDuration.toMessage());
      message.setTransferDuration(transferDuration.toMessage());
      message.setExecutionMode(executionMode.toMessageOrdinal());
      message.setParentFrameName(parentFrameName.toMessage());
      message.setIsManuallyPlaced(isManuallyPlaced.toMessage());

      message.getWaypoints().clear();
      for (int i = 0; i < waypoints.getSize(); i++)
         message.getWaypoints().add().set(waypoints.getValueReadOnly(i));
      message.getFootsteps().clear();
      for (int i = 0; i < manuallyPlacedFootsteps.getSize(); i++)
         manuallyPlacedFootsteps.getValueReadOnly(i).toMessage(message.getFootsteps().add());
      message.getGoalStancePoint().set(goalStancePoint.getValueReadOnly());
      message.getGoalFocalPoint().set(goalFocalPoint.getValueReadOnly());
      message.setLeftGoalFootXToGizmo(goalFootstepToGoalXs.get(RobotSide.LEFT).toMessage());
      message.setLeftGoalFootYToGizmo(goalFootstepToGoalYs.get(RobotSide.LEFT).toMessage());
      message.setLeftGoalFootYawToGizmo(goalFootstepToGoalYaws.get(RobotSide.LEFT).toMessage());
      message.setRightGoalFootXToGizmo(goalFootstepToGoalXs.get(RobotSide.RIGHT).toMessage());
      message.setRightGoalFootYToGizmo(goalFootstepToGoalYs.get(RobotSide.RIGHT).toMessage());
      message.setRightGoalFootYawToGizmo(goalFootstepToGoalYaws.get(RobotSide.RIGHT).toMessage());
      message.setPlannerInitialStanceSide(plannerInitialStanceSide.toMessage().toByte());
      message.setPlanner((byte) plannerType.toMessage());
      message.setPlannerWalkWithGoalOrientation(plannerWalkWithGoalOrientation.toMessage());
      message.setPlannerPlanWithBodyPath(plannerPlanWithBodyPath.toMessage());
      plannerParameters.toMessage(message.getPlannerParameters());
      message.setQuickWaypointOnly(quickWaypointOnly.toMessage());
      message.setUseRrtPathPlanner(useRRTPathPlanner.toMessage());
      message.setObstacleClearanceRadius(obstacleClearanceRadius.toMessage());
      message.setQuickHipWidth(quickHipWidth.toMessage());
      message.setQuickStepLength(quickStepLength.toMessage());
      message.setQuickNextPelvisYawLimit(quickNextPelvisYawLimit.toMessage());
      message.setQuickInwardLimit(quickInwardLimit.toMessage());
      message.setQuickOutwardLimit(quickOutwardLimit.toMessage());
      message.setQuickStepAngleLimit(quickStepAngleLimit.toMessage());
      message.setQuickSwingTimeDistanceLower(quickSwingTimeDistanceLower.toMessage());
      message.setQuickSwingTimeDistanceUpper(quickSwingTimeDistanceUpper.toMessage());
      message.setQuickMinSwingTime(quickMinSwingTime.toMessage());
      message.setQuickMaxSwingTime(quickMaxSwingTime.toMessage());
   }

   public void fromMessage(WalkActionDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      swingDuration.fromMessage(message.getSwingDuration());
      transferDuration.fromMessage(message.getTransferDuration());
      executionMode.fromMessageOrdinal(message.getExecutionMode(), ExecutionMode.values);
      parentFrameName.fromMessage(message.getParentFrameNameAsString());
      isManuallyPlaced.fromMessage(message.getIsManuallyPlaced());

      waypoints.fromMessage(writableList ->
      {
         writableList.clear();
         for (int i = 0; i < message.getWaypoints().size(); i++)
            writableList.add().set(message.getWaypoints().get(i).getPose());
      });
      manuallyPlacedFootsteps.fromMessage(writableList ->
      {
         writableList.clear();
         for (WalkActionFootstepDefinitionMessage footstepMessage : message.getFootsteps())
            writableList.add().fromMessage(footstepMessage);
      });
      goalStancePoint.fromMessage(message.getGoalStancePoint().getPoint());
      goalFocalPoint.fromMessage(message.getGoalFocalPoint().getPoint());
      goalFootstepToGoalXs.get(RobotSide.LEFT).fromMessage(message.getLeftGoalFootXToGizmo());
      goalFootstepToGoalYs.get(RobotSide.LEFT).fromMessage(message.getLeftGoalFootYToGizmo());
      goalFootstepToGoalYaws.get(RobotSide.LEFT).fromMessage(message.getLeftGoalFootYawToGizmo());
      goalFootstepToGoalXs.get(RobotSide.RIGHT).fromMessage(message.getRightGoalFootXToGizmo());
      goalFootstepToGoalYs.get(RobotSide.RIGHT).fromMessage(message.getRightGoalFootYToGizmo());
      goalFootstepToGoalYaws.get(RobotSide.RIGHT).fromMessage(message.getRightGoalFootYawToGizmo());
      plannerInitialStanceSide.fromMessage(InitialStanceSide.fromByte(message.getPlannerInitialStanceSide()));
      plannerType.fromMessage(message.getPlanner());
      plannerWalkWithGoalOrientation.fromMessage(message.getPlannerWalkWithGoalOrientation());
      plannerPlanWithBodyPath.fromMessage(message.getPlannerPlanWithBodyPath());
      plannerParameters.fromMessage(message.getPlannerParameters());
      quickWaypointOnly.fromMessage(message.getQuickWaypointOnly());
      useRRTPathPlanner.fromMessage(message.getUseRrtPathPlanner());
      obstacleClearanceRadius.fromMessage(message.getObstacleClearanceRadius());
      quickHipWidth.fromMessage(message.getQuickHipWidth());
      quickStepLength.fromMessage(message.getQuickStepLength());
      quickNextPelvisYawLimit.fromMessage(message.getQuickNextPelvisYawLimit());
      quickInwardLimit.fromMessage(message.getQuickInwardLimit());
      quickOutwardLimit.fromMessage(message.getQuickOutwardLimit());
      quickStepAngleLimit.fromMessage(message.getQuickStepAngleLimit());
      quickSwingTimeDistanceLower.fromMessage(message.getQuickSwingTimeDistanceLower());
      quickSwingTimeDistanceUpper.fromMessage(message.getQuickSwingTimeDistanceUpper());
      quickMinSwingTime.fromMessage(message.getQuickMinSwingTime());
      quickMaxSwingTime.fromMessage(message.getQuickMaxSwingTime());
   }

   public double getSwingDuration()
   {
      return swingDuration.getValue();
   }

   public void setSwingDuration(double swingDuration)
   {
      this.swingDuration.setValue(swingDuration);
   }

   public double getTransferDuration()
   {
      return transferDuration.getValue();
   }

   public void setTransferDuration(double transferDuration)
   {
      this.transferDuration.setValue(transferDuration);
   }

   public CRDTBidirectionalEnumField<ExecutionMode> getExecutionMode()
   {
      return executionMode;
   }

   public String getParentFrameName()
   {
      return parentFrameName.getValue();
   }

   public void setParentFrameName(String parentFrameName)
   {
      this.parentFrameName.setValue(parentFrameName);
   }

   public boolean getIsManuallyPlaced()
   {
      return isManuallyPlaced.getValue();
   }

   public void setIsManuallyPlaced(boolean isManuallyPlaced)
   {
      this.isManuallyPlaced.setValue(isManuallyPlaced);
   }

   public CRDTBidirectionalString getCRDTParentFrameName()
   {
      return parentFrameName;
   }

   public CRDTBidirectionalRecyclingArrayList<WalkActionFootstepDefinition> getManuallyPlacedFootsteps()
   {
      return manuallyPlacedFootsteps;
   }

   public CRDTBidirectionalRecyclingArrayList<Pose3D> getWaypoints()
   {
      return waypoints;
   }

   public CRDTBidirectionalPoint3D getGoalStancePoint()
   {
      return goalStancePoint;
   }

   public CRDTBidirectionalPoint3D getGoalFocalPoint()
   {
      return goalFocalPoint;
   }

   public CRDTBidirectionalDouble getGoalFootstepToGoalX(RobotSide side)
   {
      return goalFootstepToGoalXs.get(side);
   }

   public CRDTBidirectionalDouble getGoalFootstepToGoalY(RobotSide side)
   {
      return goalFootstepToGoalYs.get(side);
   }

   public CRDTBidirectionalDouble getGoalFootstepToGoalYaw(RobotSide side)
   {
      return goalFootstepToGoalYaws.get(side);
   }

   public CRDTBidirectionalImmutableField<InitialStanceSide> getPlannerInitialStanceSide()
   {
      return plannerInitialStanceSide;
   }

   public CRDTBidirectionalInteger getPlannerType()
   {
      return plannerType;
   }

   public CRDTBidirectionalBoolean getPlannerWalkWithGoalOrientation()
   {
      return plannerWalkWithGoalOrientation;
   }

   public CRDTBidirectionalBoolean getPlannerPlanWithBodyPath()
   {
      return plannerPlanWithBodyPath;
   }

   public DefaultFootstepPlannerParametersReadOnly getPlannerParametersReadOnly()
   {
      return (DefaultFootstepPlannerParametersReadOnly) plannerParameters.getValueReadOnly();
   }

   public DefaultFootstepPlannerParametersBasics getAndModifyPlannerParameters()
   {
      return (DefaultFootstepPlannerParametersBasics) plannerParameters.getValueAndModify();
   }

   public DefaultFootstepPlannerParametersBasics getPlannerParametersUnsafe()
   {
      return (DefaultFootstepPlannerParametersBasics) plannerParameters.getValueUnsafe();
   }

   public CRDTBidirectionalBoolean getQuickWaypointOnly()
   {
      return quickWaypointOnly;
   }

   public CRDTBidirectionalBoolean getUseRRTPathPlanner()
   {
      return useRRTPathPlanner;
   }

   public CRDTBidirectionalDouble getObstacleClearanceRadius()
   {
      return obstacleClearanceRadius;
   }

   public CRDTBidirectionalDouble getQuickHipWidth()
   {
      return quickHipWidth;
   }

   public CRDTBidirectionalDouble getQuickStepLength()
   {
      return quickStepLength;
   }

   public CRDTBidirectionalDouble getQuickNextPelvisYawLimit()
   {
      return quickNextPelvisYawLimit;
   }

   public CRDTBidirectionalDouble getQuickInwardLimit()
   {
      return quickInwardLimit;
   }

   public CRDTBidirectionalDouble getQuickOutwardLimit()
   {
      return quickOutwardLimit;
   }

   public CRDTBidirectionalDouble getQuickStepAngleLimit()
   {
      return quickStepAngleLimit;
   }

   public CRDTBidirectionalDouble getQuickSwingTimeDistanceLower()
   {
      return quickSwingTimeDistanceLower;
   }

   public CRDTBidirectionalDouble getQuickSwingTimeDistanceUpper()
   {
      return quickSwingTimeDistanceUpper;
   }

   public CRDTBidirectionalDouble getQuickMinSwingTime()
   {
      return quickMinSwingTime;
   }

   public CRDTBidirectionalDouble getQuickMaxSwingTime()
   {
      return quickMaxSwingTime;
   }
}
