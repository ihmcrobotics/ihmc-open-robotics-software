package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessage;
import behavior_msgs.msg.dds.FootstepPlanActionFootstepDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.BooleanNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import com.fasterxml.jackson.databind.node.TextNode;
import us.ihmc.behaviors.sequence.ActionNodeDefinition;
import us.ihmc.behaviors.tools.BehaviorStoredPropertySetDefinition;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.crdt.*;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.graphSearch.parameters.InitialStanceSide;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.JSONTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class FootstepPlanActionDefinition extends ActionNodeDefinition
{
   private final CRDTBidirectionalDouble swingDuration;
   private final CRDTBidirectionalDouble transferDuration;
   private final CRDTBidirectionalEnumField<ExecutionMode> executionMode;
   private final CRDTBidirectionalString parentFrameName;
   private final CRDTBidirectionalBoolean isManuallyPlaced;
   private final CRDTBidirectionalRecyclingArrayList<FootstepPlanActionFootstepDefinition> manuallyPlacedFootsteps;
   private final CRDTBidirectionalPoint3D goalStancePoint;
   private final CRDTBidirectionalPoint3D goalFocalPoint;
   private final SideDependentList<CRDTBidirectionalDouble> goalFootstepToGoalXs;
   private final SideDependentList<CRDTBidirectionalDouble> goalFootstepToGoalYs;
   private final SideDependentList<CRDTBidirectionalDouble> goalFootstepToGoalYaws;
   private final CRDTBidirectionalImmutableField<InitialStanceSide> plannerInitialStanceSide;
   private final CRDTBidirectionalBoolean plannerPerformAStarSearch;
   private final CRDTBidirectionalBoolean plannerWalkWithGoalOrientation;
   private final BehaviorStoredPropertySetDefinition plannerParameters;

   // On disk fields
   private double onDiskSwingDuration;
   private double onDiskTransferDuration;
   private ExecutionMode onDiskExecutionMode;
   private String onDiskParentFrameName;
   private boolean onDiskIsManuallyPlaced;
   private int onDiskNumberOfFootsteps;
   private final Point3D onDiskGoalStancePoint = new Point3D();
   private final Point3D onDiskGoalFocalPoint = new Point3D();
   private final SideDependentList<Double> onDiskGoalFootstepToGoalXs = new SideDependentList<>(() -> 0.0);
   private final SideDependentList<Double> onDiskGoalFootstepToGoalYs = new SideDependentList<>(() -> 0.0);
   private final SideDependentList<Double> onDiskGoalFootstepToGoalYaws = new SideDependentList<>(() -> 0.0);
   private InitialStanceSide onDiskPlannerInitialStanceSide;
   private boolean onDiskPlannerPerformAStarSearch;
   private boolean onDiskPlannerWalkWithGoalOrientation;

   public FootstepPlanActionDefinition(CRDTInfo crdtInfo,
                                       WorkspaceResourceDirectory saveFileDirectory,
                                       DefaultFootstepPlannerParametersBasics initialPlannerParameters)
   {
      super(crdtInfo, saveFileDirectory);

      swingDuration = new CRDTBidirectionalDouble(this, 1.2);
      transferDuration = new CRDTBidirectionalDouble(this, 0.8);
      executionMode = new CRDTBidirectionalEnumField<>(this, ExecutionMode.OVERRIDE);
      parentFrameName = new CRDTBidirectionalString(this, ReferenceFrame.getWorldFrame().getName());
      isManuallyPlaced = new CRDTBidirectionalBoolean(this, false);
      manuallyPlacedFootsteps = new CRDTBidirectionalRecyclingArrayList<>(this, new RecyclingArrayList<>(() -> new FootstepPlanActionFootstepDefinition(this)));
      goalStancePoint = new CRDTBidirectionalPoint3D(this);
      goalFocalPoint = new CRDTBidirectionalPoint3D(this);
      goalFootstepToGoalXs = new SideDependentList<>(() -> new CRDTBidirectionalDouble(this, 0.0));
      goalFootstepToGoalYs = new SideDependentList<>(() -> new CRDTBidirectionalDouble(this, 0.0));
      goalFootstepToGoalYaws = new SideDependentList<>(() -> new CRDTBidirectionalDouble(this, 0.0));
      plannerInitialStanceSide = new CRDTBidirectionalImmutableField<>(this, InitialStanceSide.AUTO);
      plannerPerformAStarSearch = new CRDTBidirectionalBoolean(this, true);
      plannerWalkWithGoalOrientation = new CRDTBidirectionalBoolean(this, true);
      plannerParameters = new BehaviorStoredPropertySetDefinition(this, "plannerParameters", initialPlannerParameters);
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
         JSONTools.toJSON(jsonNode, "goalStancePoint", goalStancePoint.getValueReadOnly());
         JSONTools.toJSON(jsonNode, "goalFocalPoint", goalFocalPoint.getValueReadOnly());

         for (RobotSide side : RobotSide.values)
         {
            ObjectNode goalFootNode = jsonNode.putObject(side.getCamelCaseName() + "GoalFootToGoal");
            goalFootNode.put("x", (float) MathTools.roundToPrecision(goalFootstepToGoalXs.get(side).getValue(), 0.0005));
            goalFootNode.put("y", (float) MathTools.roundToPrecision(goalFootstepToGoalYs.get(side).getValue(), 0.0005));
            goalFootNode.put("yawInDegrees", (float) MathTools.roundToPrecision(Math.toDegrees(goalFootstepToGoalYaws.get(side).getValue()), 0.02));
         }
         jsonNode.put("plannerInitialStanceSide", plannerInitialStanceSide.getValue().name());
         jsonNode.put("plannerPerformAStarSearch", plannerPerformAStarSearch.getValue());
         if (plannerPerformAStarSearch.getValue())
            jsonNode.put("plannerWalkWithGoalOrientation", plannerWalkWithGoalOrientation.getValue());
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

      manuallyPlacedFootsteps.getValueAndFreeze().clear();
      if (isManuallyPlaced.getValue())
      {
         JSONTools.forEachArrayElement(jsonNode, "footsteps", footstepNode -> manuallyPlacedFootsteps.getValueAndFreeze().add().loadFromFile(footstepNode));
      }
      else
      {
         JSONTools.toEuclid(jsonNode, "goalStancePoint", goalStancePoint.getValueAndFreeze());
         JSONTools.toEuclid(jsonNode, "goalFocalPoint", goalFocalPoint.getValueAndFreeze());

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
            plannerPerformAStarSearch.setValue(booleanNode.booleanValue());
         if (jsonNode.get("plannerWalkWithGoalOrientation") instanceof BooleanNode booleanNode)
            plannerWalkWithGoalOrientation.setValue(booleanNode.booleanValue());
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
      onDiskPlannerPerformAStarSearch = plannerPerformAStarSearch.getValue();
      onDiskPlannerWalkWithGoalOrientation = plannerWalkWithGoalOrientation.getValue();
      plannerParameters.setOnDiskFields();
   }

   @Override
   public void undoAllNontopologicalChanges()
   {
      super.undoAllNontopologicalChanges();

      swingDuration.setValue(onDiskSwingDuration);
      transferDuration.setValue(onDiskTransferDuration);
      executionMode.setValue(onDiskExecutionMode);
      parentFrameName.setValue(onDiskParentFrameName);
      isManuallyPlaced.setValue(onDiskIsManuallyPlaced);
      manuallyPlacedFootsteps.getValueAndFreeze().clear();
      for (int i = 0; i < onDiskNumberOfFootsteps; i++)
         manuallyPlacedFootsteps.getValueAndFreeze().add();
      goalStancePoint.getValueAndFreeze().set(onDiskGoalStancePoint);
      goalFocalPoint.getValueAndFreeze().set(onDiskGoalFocalPoint);
      for (RobotSide side : onDiskGoalFootstepToGoalXs.sides())
      {
         goalFootstepToGoalXs.get(side).setValue(onDiskGoalFootstepToGoalXs.get(side));
         goalFootstepToGoalYs.get(side).setValue(onDiskGoalFootstepToGoalYs.get(side));
         goalFootstepToGoalYaws.get(side).setValue(onDiskGoalFootstepToGoalYaws.get(side));
      }

      for (int i = 0; i < manuallyPlacedFootsteps.getSize(); i++)
         manuallyPlacedFootsteps.getValueAndFreeze().get(i).undoAllNontopologicalChanges();
      plannerInitialStanceSide.setValue(onDiskPlannerInitialStanceSide);
      plannerPerformAStarSearch.setValue(onDiskPlannerPerformAStarSearch);
      plannerWalkWithGoalOrientation.setValue(onDiskPlannerWalkWithGoalOrientation);
      plannerParameters.undoAllNontopologicalChanges();
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
      unchanged &= plannerPerformAStarSearch.getValue() == onDiskPlannerPerformAStarSearch;
      unchanged &= plannerWalkWithGoalOrientation.getValue() == onDiskPlannerWalkWithGoalOrientation;
      unchanged &= plannerParameters.isUnchanged();

      return !unchanged;
   }

   public void toMessage(FootstepPlanActionDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setSwingDuration(swingDuration.toMessage());
      message.setTransferDuration(transferDuration.toMessage());
      message.setExecutionMode(executionMode.toMessageOrdinal());
      message.setParentFrameName(parentFrameName.toMessage());
      message.setIsManuallyPlaced(isManuallyPlaced.toMessage());

      message.getFootsteps().clear();
      for (int i = 0; i < manuallyPlacedFootsteps.getSize(); i++)
      {
         manuallyPlacedFootsteps.getValueReadOnly(i).toMessage(message.getFootsteps().add());
      }
      goalStancePoint.toMessage(message.getGoalStancePoint());
      goalFocalPoint.toMessage(message.getGoalFocalPoint());
      message.setLeftGoalFootXToGizmo(goalFootstepToGoalXs.get(RobotSide.LEFT).toMessage());
      message.setLeftGoalFootYToGizmo(goalFootstepToGoalYs.get(RobotSide.LEFT).toMessage());
      message.setLeftGoalFootYawToGizmo(goalFootstepToGoalYaws.get(RobotSide.LEFT).toMessage());
      message.setRightGoalFootXToGizmo(goalFootstepToGoalXs.get(RobotSide.RIGHT).toMessage());
      message.setRightGoalFootYToGizmo(goalFootstepToGoalYs.get(RobotSide.RIGHT).toMessage());
      message.setRightGoalFootYawToGizmo(goalFootstepToGoalYaws.get(RobotSide.RIGHT).toMessage());
      message.setPlannerInitialStanceSide(plannerInitialStanceSide.toMessage().toByte());
      message.setPlannerPerformAStarSearch(plannerPerformAStarSearch.toMessage());
      message.setPlannerWalkWithGoalOrientation(plannerWalkWithGoalOrientation.toMessage());
      plannerParameters.toMessage(message.getPlannerParameters());
   }

   public void fromMessage(FootstepPlanActionDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      swingDuration.fromMessage(message.getSwingDuration());
      transferDuration.fromMessage(message.getTransferDuration());
      executionMode.fromMessageOrdinal(message.getExecutionMode(), ExecutionMode.values);
      parentFrameName.fromMessage(message.getParentFrameNameAsString());
      isManuallyPlaced.fromMessage(message.getIsManuallyPlaced());

      manuallyPlacedFootsteps.fromMessage(writableList ->
      {
         writableList.clear();
         for (FootstepPlanActionFootstepDefinitionMessage footstepMessage : message.getFootsteps())
         {
            writableList.add().fromMessage(footstepMessage);
         }
      });
      goalStancePoint.fromMessage(message.getGoalStancePoint());
      goalFocalPoint.fromMessage(message.getGoalFocalPoint());
      goalFootstepToGoalXs.get(RobotSide.LEFT).fromMessage(message.getLeftGoalFootXToGizmo());
      goalFootstepToGoalYs.get(RobotSide.LEFT).fromMessage(message.getLeftGoalFootYToGizmo());
      goalFootstepToGoalYaws.get(RobotSide.LEFT).fromMessage(message.getLeftGoalFootYawToGizmo());
      goalFootstepToGoalXs.get(RobotSide.RIGHT).fromMessage(message.getRightGoalFootXToGizmo());
      goalFootstepToGoalYs.get(RobotSide.RIGHT).fromMessage(message.getRightGoalFootYToGizmo());
      goalFootstepToGoalYaws.get(RobotSide.RIGHT).fromMessage(message.getRightGoalFootYawToGizmo());
      plannerInitialStanceSide.fromMessage(InitialStanceSide.fromByte(message.getPlannerInitialStanceSide()));
      plannerPerformAStarSearch.fromMessage(message.getPlannerPerformAStarSearch());
      plannerWalkWithGoalOrientation.fromMessage(message.getPlannerWalkWithGoalOrientation());
      plannerParameters.fromMessage(message.getPlannerParameters());
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

   public CRDTBidirectionalRecyclingArrayList<FootstepPlanActionFootstepDefinition> getManuallyPlacedFootsteps()
   {
      return manuallyPlacedFootsteps;
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

   public CRDTBidirectionalBoolean getPlannerPerformAStarSearch()
   {
      return plannerPerformAStarSearch;
   }

   public CRDTBidirectionalBoolean getPlannerWalkWithGoalOrientation()
   {
      return plannerWalkWithGoalOrientation;
   }

   public DefaultFootstepPlannerParametersReadOnly getPlannerParametersReadOnly()
   {
      return (DefaultFootstepPlannerParametersReadOnly) plannerParameters.getValueReadOnly();
   }

   public DefaultFootstepPlannerParametersBasics getAndFreezePlannerParameters()
   {
      return (DefaultFootstepPlannerParametersBasics) plannerParameters.getValueAndFreeze();
   }

   public DefaultFootstepPlannerParametersBasics getPlannerParametersUnsafe()
   {
      return (DefaultFootstepPlannerParametersBasics) plannerParameters.getValueUnsafe();
   }
}
