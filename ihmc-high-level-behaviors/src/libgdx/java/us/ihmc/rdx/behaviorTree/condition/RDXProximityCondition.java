package us.ihmc.rdx.behaviorTree.condition;

import imgui.ImGui;
import us.ihmc.behaviors.behaviorTree.condition.ConditionNodeDefinition;
import us.ihmc.behaviors.behaviorTree.condition.ConditionNodeState;
import us.ihmc.behaviors.behaviorTree.condition.ProximityConditionDefinition;
import us.ihmc.behaviors.behaviorTree.condition.ProximityConditionDefinition.DistanceType;
import us.ihmc.behaviors.behaviorTree.condition.ProximityConditionState;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneState;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiReferenceFrameLibraryCombo;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

public class RDXProximityCondition
{
   private final BehaviorTreeSceneState scene;
   private final ConditionNodeState state;
   private final ConditionNodeDefinition definition;
   private final ProximityConditionState proximityState;
   private final ProximityConditionDefinition proximityDefinition;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiReferenceFrameLibraryCombo frameAComboBox;
   private final ImGuiReferenceFrameLibraryCombo frameBComboBox;
   private final ImDoubleWrapper minDistanceWidget;
   private final ImDoubleWrapper maxDistanceWidget;
   private final ImDoubleWrapper timeoutWidget;

   public RDXProximityCondition(ConditionNodeState state, BehaviorTreeSceneState scene)
   {
      this.state = state;
      this.scene = scene;
      definition = state.getDefinition();
      proximityState = state.getProximityCheck();
      proximityDefinition = definition.getProximityCheck();

      frameAComboBox = new ImGuiReferenceFrameLibraryCombo("Frame A",
                                                           scene::getAllFrameNames,
                                                           definition.getProximityCheck()::getFrameNameA,
                                                           definition.getProximityCheck()::setFrameNameA);
      frameBComboBox = new ImGuiReferenceFrameLibraryCombo("Frame B",
                                                           scene::getAllFrameNames,
                                                           definition.getProximityCheck()::getFrameNameB,
                                                           definition.getProximityCheck()::setFrameNameB);
      minDistanceWidget = new ImDoubleWrapper(proximityDefinition::getMinDistance,
                                              proximityDefinition::setMinDistance,
                                              imDouble -> ImGuiTools.volatileInputDouble(labels.get("Min Distance"), imDouble));
      maxDistanceWidget = new ImDoubleWrapper(proximityDefinition::getMaxDistance,
                                              proximityDefinition::setMaxDistance,
                                              imDouble -> ImGuiTools.volatileInputDouble(labels.get("Max Distance"), imDouble));
      timeoutWidget = new ImDoubleWrapper(proximityDefinition::getTimeout,
                                          proximityDefinition::setTimeout,
                                          imDouble -> ImGuiTools.volatileInputDouble(labels.get("Timeout"), imDouble));
   }

   public void renderImGuiWidgetsInternal()
   {
      frameAComboBox.render();
      frameBComboBox.render();
      DistanceType currentType = definition.getProximityCheck().getDistanceType();
      if (ImGui.beginCombo(labels.get("Distance Type"), currentType.name()))
      {
         for (DistanceType value : DistanceType.values)
            if (ImGui.selectable(value.name(), value == currentType))
               definition.getProximityCheck().setDistanceType(value);
         ImGui.endCombo();
      }
      timeoutWidget.renderImGuiWidget();
      ImGui.pushItemWidth(80.0f);
      minDistanceWidget.renderImGuiWidget();
      maxDistanceWidget.renderImGuiWidget();
      ImGui.popItemWidth();
      Vector3DReadOnly vectorBToA = proximityState.getVectorBToA();
      ImGui.text("Current vector B to A: (%.3f, %.3f, %.3f)".formatted(vectorBToA.getX(), vectorBToA.getY(), vectorBToA.getZ()));
      double distance = switch (proximityDefinition.getDistanceType())
      {
         case XYZ -> vectorBToA.norm();
         case XY -> Math.hypot(vectorBToA.getX(), vectorBToA.getY());
         case Z -> Math.abs(vectorBToA.getZ());
      };
      ImGui.text("Distance for Type: %.3f".formatted(distance));
   }
}