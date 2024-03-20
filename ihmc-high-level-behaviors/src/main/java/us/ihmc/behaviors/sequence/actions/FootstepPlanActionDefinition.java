package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessage;
import behavior_msgs.msg.dds.FootstepPlanActionFootstepDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.ActionNodeDefinition;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.crdt.*;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.JSONTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class FootstepPlanActionDefinition extends ActionNodeDefinition
{
   private final CRDTUnidirectionalDouble swingDuration;
   private final CRDTUnidirectionalDouble transferDuration;
   private final CRDTUnidirectionalString parentFrameName;
   private final CRDTUnidirectionalBoolean isManuallyPlaced;
   private final CRDTUnidirectionalRecyclingArrayList<FootstepPlanActionFootstepDefinition> footsteps;
   private final CRDTUnidirectionalPoint3D goalStancePoint;
   private final CRDTUnidirectionalPoint3D goalFocalPoint;
   private final SideDependentList<CRDTUnidirectionalDouble> goalFootstepToGoalXs;
   private final SideDependentList<CRDTUnidirectionalDouble> goalFootstepToGoalYs;
   private final SideDependentList<CRDTUnidirectionalDouble> goalFootstepToGoalYaws;

   // On disk fields
   private double onDiskSwingDuration;
   private double onDiskTransferDuration;
   private String onDiskParentFrameName;
   private boolean onDiskIsManuallyPlaced;
   private int onDiskNumberOfFootsteps;
   private final Point3D onDiskGoalStancePoint = new Point3D();
   private final Point3D onDiskGoalFocalPoint = new Point3D();
   private final SideDependentList<Double> onDiskGoalFootstepToGoalXs = new SideDependentList<>(() -> 0.0);
   private final SideDependentList<Double> onDiskGoalFootstepToGoalYs = new SideDependentList<>(() -> 0.0);
   private final SideDependentList<Double> onDiskGoalFootstepToGoalYaws = new SideDependentList<>(() -> 0.0);

   public FootstepPlanActionDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo, saveFileDirectory);

      swingDuration = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, 1.2);
      transferDuration = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, 0.8);
      parentFrameName = new CRDTUnidirectionalString(ROS2ActorDesignation.OPERATOR, crdtInfo, ReferenceFrame.getWorldFrame().getName());
      isManuallyPlaced = new CRDTUnidirectionalBoolean(ROS2ActorDesignation.OPERATOR, crdtInfo, false);
      footsteps = new CRDTUnidirectionalRecyclingArrayList<>(ROS2ActorDesignation.OPERATOR,
                                                             crdtInfo,
                                                             () -> new RecyclingArrayList<>(() -> new FootstepPlanActionFootstepDefinition(crdtInfo)));
      goalStancePoint = new CRDTUnidirectionalPoint3D(ROS2ActorDesignation.OPERATOR, crdtInfo);
      goalFocalPoint = new CRDTUnidirectionalPoint3D(ROS2ActorDesignation.OPERATOR, crdtInfo);
      goalFootstepToGoalXs = new SideDependentList<>(() -> new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, 0.0));
      goalFootstepToGoalYs = new SideDependentList<>(() -> new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, 0.0));
      goalFootstepToGoalYaws = new SideDependentList<>(() -> new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, 0.0));
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("swingDuration", swingDuration.getValue());
      jsonNode.put("transferDuration", transferDuration.getValue());
      jsonNode.put("parentFrame", parentFrameName.getValue());

      if (isManuallyPlaced.getValue())
      {
         ArrayNode foostepsArrayNode = jsonNode.putArray("footsteps");
         for (int i = 0; i < footsteps.getSize(); i++)
         {
            ObjectNode footstepNode = foostepsArrayNode.addObject();
            footsteps.getValueReadOnly(i).saveToFile(footstepNode);
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
      }
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      swingDuration.setValue(jsonNode.get("swingDuration").asDouble());
      transferDuration.setValue(jsonNode.get("transferDuration").asDouble());
      parentFrameName.setValue(jsonNode.get("parentFrame").textValue());
      isManuallyPlaced.setValue(jsonNode.get("footsteps") != null);

      footsteps.getValue().clear();
      if (isManuallyPlaced.getValue())
      {
         JSONTools.forEachArrayElement(jsonNode, "footsteps", footstepNode -> footsteps.getValue().add().loadFromFile(footstepNode));
      }
      else
      {
//         ObjectNode goalToParentNode = (ObjectNode) jsonNode.get("goalToParent");
//         goalToParentX.setValue(goalToParentNode.get("x").asDouble());
//         goalToParentY.setValue(goalToParentNode.get("y").asDouble());
//         goalToParentYaw.setValue(Math.toRadians(goalToParentNode.get("yawInDegrees").asDouble()));

         for (RobotSide side : RobotSide.values)
         {
            ObjectNode goalFootNode = (ObjectNode) jsonNode.get(side.getCamelCaseName() + "GoalFootToGoal");
            goalFootstepToGoalXs.get(side).setValue(goalFootNode.get("x").asDouble());
            goalFootstepToGoalYs.get(side).setValue(goalFootNode.get("y").asDouble());
            goalFootstepToGoalYaws.get(side).setValue(Math.toRadians(goalFootNode.get("yawInDegrees").asDouble()));
         }
      }
   }

   @Override
   public void setOnDiskFields()
   {
      super.setOnDiskFields();

      onDiskSwingDuration = swingDuration.getValue();
      onDiskTransferDuration = transferDuration.getValue();
      onDiskParentFrameName = parentFrameName.getValue();
      onDiskIsManuallyPlaced = isManuallyPlaced.getValue();
      onDiskNumberOfFootsteps = footsteps.getSize();
      onDiskGoalStancePoint.set(goalStancePoint.getValueReadOnly());
      onDiskGoalFocalPoint.set(goalFocalPoint.getValueReadOnly());
      for (RobotSide side : goalFootstepToGoalXs.sides())
      {
         onDiskGoalFootstepToGoalXs.put(side, goalFootstepToGoalXs.get(side).getValue());
         onDiskGoalFootstepToGoalYs.put(side, goalFootstepToGoalYs.get(side).getValue());
         onDiskGoalFootstepToGoalYaws.put(side, goalFootstepToGoalYaws.get(side).getValue());
      }

      for (int i = 0; i < footsteps.getSize(); i++)
         footsteps.getValueReadOnly(i).setOnDiskFields();
   }

   @Override
   public void undoAllNontopologicalChanges()
   {
      super.undoAllNontopologicalChanges();

      swingDuration.setValue(onDiskSwingDuration);
      transferDuration.setValue(onDiskTransferDuration);
      parentFrameName.setValue(onDiskParentFrameName);
      isManuallyPlaced.setValue(onDiskIsManuallyPlaced);
      footsteps.getValue().clear();
      for (int i = 0; i < onDiskNumberOfFootsteps; i++)
         footsteps.getValue().add();
      goalStancePoint.getValue().set(onDiskGoalStancePoint);
      goalFocalPoint.getValue().set(onDiskGoalFocalPoint);
      for (RobotSide side : onDiskGoalFootstepToGoalXs.sides())
      {
         goalFootstepToGoalXs.get(side).setValue(onDiskGoalFootstepToGoalXs.get(side));
         goalFootstepToGoalYs.get(side).setValue(onDiskGoalFootstepToGoalYs.get(side));
         goalFootstepToGoalYaws.get(side).setValue(onDiskGoalFootstepToGoalYaws.get(side));
      }

      for (int i = 0; i < footsteps.getSize(); i++)
         footsteps.getValue().get(i).undoAllNontopologicalChanges();
   }

   @Override
   public boolean hasChanges()
   {
      boolean unchanged = !super.hasChanges();

      unchanged &= swingDuration.getValue() == onDiskSwingDuration;
      unchanged &= transferDuration.getValue() == onDiskTransferDuration;
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

      boolean sameNumberOfFootsteps = footsteps.getSize() == onDiskNumberOfFootsteps;
      unchanged &= sameNumberOfFootsteps;

      if (sameNumberOfFootsteps)
         for (int i = 0; i < footsteps.getSize(); i++)
            unchanged &= !footsteps.getValueReadOnly(i).hasChanges();

      return !unchanged;
   }

   public void toMessage(FootstepPlanActionDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setSwingDuration(swingDuration.toMessage());
      message.setTransferDuration(transferDuration.toMessage());
      message.setParentFrameName(parentFrameName.toMessage());
      message.setIsManuallyPlaced(isManuallyPlaced.toMessage());

      message.getFootsteps().clear();
      for (int i = 0; i < footsteps.getSize(); i++)
      {
         footsteps.getValueReadOnly(i).toMessage(message.getFootsteps().add());
      }
      goalStancePoint.toMessage(message.getGoalStancePoint());
      goalFocalPoint.toMessage(message.getGoalFocalPoint());
      message.setLeftGoalFootXToGizmo(goalFootstepToGoalXs.get(RobotSide.LEFT).toMessage());
      message.setLeftGoalFootYToGizmo(goalFootstepToGoalYs.get(RobotSide.LEFT).toMessage());
      message.setLeftGoalFootYawToGizmo(goalFootstepToGoalYaws.get(RobotSide.LEFT).toMessage());
      message.setRightGoalFootXToGizmo(goalFootstepToGoalXs.get(RobotSide.RIGHT).toMessage());
      message.setRightGoalFootYToGizmo(goalFootstepToGoalYs.get(RobotSide.RIGHT).toMessage());
      message.setRightGoalFootYawToGizmo(goalFootstepToGoalYaws.get(RobotSide.RIGHT).toMessage());
   }

   public void fromMessage(FootstepPlanActionDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      swingDuration.fromMessage(message.getSwingDuration());
      transferDuration.fromMessage(message.getTransferDuration());
      parentFrameName.fromMessage(message.getParentFrameNameAsString());
      isManuallyPlaced.fromMessage(message.getIsManuallyPlaced());

      footsteps.fromMessage(writableList ->
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

   public CRDTUnidirectionalString getCRDTParentFrameName()
   {
      return parentFrameName;
   }

   public CRDTUnidirectionalRecyclingArrayList<FootstepPlanActionFootstepDefinition> getFootsteps()
   {
      return footsteps;
   }

   public CRDTUnidirectionalPoint3D getGoalStancePoint()
   {
      return goalStancePoint;
   }

   public CRDTUnidirectionalPoint3D getGoalFocalPoint()
   {
      return goalFocalPoint;
   }

   public CRDTUnidirectionalDouble getGoalFootstepToGoalX(RobotSide side)
   {
      return goalFootstepToGoalXs.get(side);
   }

   public CRDTUnidirectionalDouble getGoalFootstepToGoalY(RobotSide side)
   {
      return goalFootstepToGoalYs.get(side);
   }

   public CRDTUnidirectionalDouble getGoalFootstepToGoalYaw(RobotSide side)
   {
      return goalFootstepToGoalYaws.get(side);
   }
}
