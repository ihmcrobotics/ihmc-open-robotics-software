package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessage;
import behavior_msgs.msg.dds.FootstepPlanActionFootstepDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.ActionNodeDefinition;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.crdt.*;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
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
   private final CRDTUnidirectionalDouble goalToParentX;
   private final CRDTUnidirectionalDouble goalToParentY;
   private final CRDTUnidirectionalDouble goalToParentYaw;
   private final SideDependentList<CRDTUnidirectionalDouble> goalFootstepToGoalXs;
   private final SideDependentList<CRDTUnidirectionalDouble> goalFootstepToGoalYs;
   private final SideDependentList<CRDTUnidirectionalDouble> goalFootstepToGoalYaws;

   // On disk fields
   private double onDiskSwingDuration;
   private double onDiskTransferDuration;
   private String onDiskParentFrameName;
   private boolean onDiskIsManuallyPlaced;
   private int onDiskNumberOfFootsteps;
   private double onDiskGoalToParentX;
   private double onDiskGoalToParentY;
   private double onDiskGoalToParentYaw;
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
      goalToParentX = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, 0.0);
      goalToParentY = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, 0.0);
      goalToParentYaw = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, 0.0);
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
         jsonNode.put("goalToParentX", goalToParentX.getValue());
         jsonNode.put("goalToParentY", goalToParentY.getValue());
         jsonNode.put("goalToParentYaw", goalToParentYaw.getValue());
         for (RobotSide side : RobotSide.values)
         {
            ObjectNode goalFootNode = jsonNode.putObject(side.getCamelCaseName() + "GoalFootTransform");
            goalFootNode.put("goalToParentX", goalFootstepToGoalXs.get(side).getValue());
            goalFootNode.put("goalToParentY", goalFootstepToGoalYs.get(side).getValue());
            goalFootNode.put("goalToParentYaw", goalFootstepToGoalYaws.get(side).getValue());
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
         RigidBodyTransform goalToParentTransform = new RigidBodyTransform();
         JSONTools.toEuclid(jsonNode, "goalToParentTransform", goalToParentTransform);

         goalToParentX.setValue(goalToParentTransform.getTranslationX());
         goalToParentY.setValue(goalToParentTransform.getTranslationY());
         goalToParentYaw.setValue(goalToParentTransform.getRotation().getYaw());

         for (RobotSide side : RobotSide.values)
         {
            JsonNode goalFootNode = jsonNode.get(side.getCamelCaseName() + "GoalFootTransform");
            RigidBodyTransform goalFootstepToGoalTransform = new RigidBodyTransform();
            JSONTools.toEuclid(goalFootNode, goalFootstepToGoalTransform);

            goalFootstepToGoalXs.get(side).setValue(goalFootstepToGoalTransform.getTranslationX());
            goalFootstepToGoalYs.get(side).setValue(goalFootstepToGoalTransform.getTranslationY());
            goalFootstepToGoalYaws.get(side).setValue(goalFootstepToGoalTransform.getRotation().getYaw());
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
      onDiskGoalToParentX = goalToParentX.getValue();
      onDiskGoalToParentY = goalToParentY.getValue();
      onDiskGoalToParentYaw = goalToParentYaw.getValue();
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
      goalToParentX.setValue(onDiskGoalToParentX);
      goalToParentY.setValue(onDiskGoalToParentY);
      goalToParentYaw.setValue(onDiskGoalToParentYaw);
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
      unchanged &= goalToParentX.getValue() == onDiskGoalToParentX;
      unchanged &= goalToParentY.getValue() == onDiskGoalToParentY;
      unchanged &= goalToParentYaw.getValue() == onDiskGoalToParentYaw;
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
      message.setGoalXInParent(goalToParentX.toMessage());
      message.setGoalYInParent(goalToParentY.toMessage());
      message.setGoalYawInParent(goalToParentYaw.toMessage());
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
      goalToParentX.fromMessage(message.getGoalXInParent());
      goalToParentY.fromMessage(message.getGoalYInParent());
      goalToParentYaw.fromMessage(message.getGoalYawInParent());
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

   public CRDTUnidirectionalDouble getGoalToParentX()
   {
      return goalToParentX;
   }

   public CRDTUnidirectionalDouble getGoalToParentY()
   {
      return goalToParentY;
   }

   public CRDTUnidirectionalDouble getGoalToParentYaw()
   {
      return goalToParentYaw;
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
