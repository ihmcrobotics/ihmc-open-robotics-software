package us.ihmc.rdx.behaviorTree.condition;

import imgui.ImGui;
import us.ihmc.behaviors.behaviorTree.condition.ConditionNodeDefinition;
import us.ihmc.behaviors.behaviorTree.condition.ConditionNodeState;
import us.ihmc.behaviors.behaviorTree.condition.ProximityConditionDefinition.DistanceType;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneState;
import us.ihmc.communication.crdt.CRDTBidirectionalBoolean;
import us.ihmc.communication.crdt.CRDTBidirectionalDouble;
import us.ihmc.rdx.imgui.ImBooleanWrapper;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiReferenceFrameLibraryCombo;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

public class RDXProximityCondition
{
   private final ConditionNodeState state;
   private final BehaviorTreeSceneState scene;
   private final ConditionNodeDefinition definition;

   private final CRDTBidirectionalDouble distance;
   private final CRDTBidirectionalDouble maxDistance;
   private final CRDTBidirectionalDouble maxEvaluationTime;
   private final CRDTBidirectionalBoolean manageMissingFrameInternally;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiReferenceFrameLibraryCombo objectFrameComboBox;
   private final ImGuiReferenceFrameLibraryCombo referenceFrameComboBox;
   private final ImDoubleWrapper maxDistanceWidget;
   private final ImDoubleWrapper distanceWidget;
   private final ImDoubleWrapper maxEvaluationTimeWidget;
   private final ImBooleanWrapper manageMissingFrameInternallyCheckbox;

   public RDXProximityCondition(ConditionNodeState state, BehaviorTreeSceneState scene)
   {
      this.state = state;
      this.scene = scene;
      definition = state.getDefinition();

      distance =  state.getProximityCheck().getCurrentDistance();
      maxDistance = definition.getProximityCheck().getCRDTMaxDistanceToObject();
      maxEvaluationTime = definition.getProximityCheck().getCRDTMaxEvaluationTime();
      manageMissingFrameInternally = definition.getProximityCheck().getCRDTManageMissingFrameInternally();

      objectFrameComboBox = new ImGuiReferenceFrameLibraryCombo("Object Name",
                                                                scene::getAllFrameNames,
                                                                definition.getProximityCheck()::getObjectFrameName,
                                                                definition.getProximityCheck()::setObjectFrameName);
      referenceFrameComboBox = new ImGuiReferenceFrameLibraryCombo("Reference Name",
                                                                   scene::getAllFrameNames,
                                                                   definition.getProximityCheck()::getReferenceFrameName,
                                                                   definition.getProximityCheck()::setReferenceFrameName);
      maxDistanceWidget = new ImDoubleWrapper(maxDistance::getValue,
                                              maxDistance::setValue,
                                              imDouble -> ImGuiTools.volatileInputDouble(labels.get("Max Distance"), imDouble));
      distanceWidget = new ImDoubleWrapper(distance::getValue,
                                           distance::setValue,
                                           imDouble -> ImGuiTools.volatileInputDouble(labels.get("Distance"), imDouble));
      maxEvaluationTimeWidget = new ImDoubleWrapper(maxEvaluationTime::getValue,
                                                    maxEvaluationTime::setValue,
                                                    imDouble -> ImGuiTools.volatileInputDouble(labels.get("Max Evaluation Time"), imDouble));
      manageMissingFrameInternallyCheckbox = new ImBooleanWrapper(manageMissingFrameInternally::getValue,
                                                                  manageMissingFrameInternally::setValue,
                                                                  imBoolean -> ImGui.checkbox(labels.get("Manage Missing Frame Internally"), imBoolean));
   }

   public void renderImGuiWidgetsInternal()
   {
      objectFrameComboBox.render();
      referenceFrameComboBox.render();
      ImGui.pushItemWidth(80.0f);
      distanceWidget.renderImGuiWidget();
      maxDistanceWidget.renderImGuiWidget();
      ImGui.popItemWidth();
      DistanceType currentType = definition.getProximityCheck().getType().getValue();
      if (ImGui.beginCombo(labels.get("Distance Type"), currentType.name()))
      {
         for (DistanceType value : DistanceType.values)
         {
            if (ImGui.selectable(value.name(), value == currentType))
            {
               definition.getProximityCheck().getType().setValue(value);
            }
         }

         ImGui.endCombo();
      }
      maxEvaluationTimeWidget.renderImGuiWidget();
      manageMissingFrameInternallyCheckbox.renderImGuiWidget();
   }
}