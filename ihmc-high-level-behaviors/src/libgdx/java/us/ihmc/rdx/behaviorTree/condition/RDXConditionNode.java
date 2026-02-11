package us.ihmc.rdx.behaviorTree.condition;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import us.ihmc.behaviors.behaviorTree.condition.ConditionNodeDefinition;
import us.ihmc.behaviors.behaviorTree.condition.ConditionNodeDefinition.ConditionNodeType;
import us.ihmc.behaviors.behaviorTree.condition.ConditionNodeState;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTreeRootNode;
import us.ihmc.rdx.behaviorTree.RDXLeafNode;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.widgets.ImGuiConditionNodeWidget;

public class RDXConditionNode extends RDXLeafNode<ConditionNodeState, ConditionNodeDefinition>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiConditionNodeWidget conditionIconWidget = new ImGuiConditionNodeWidget();

   private final RDXCounterCondition counter;
   private final RDXLLMCondition llm;
   private final RDXProximityCondition proximityCheck;
   private final RDXShapeContainsCondition shapeContains;

   public RDXConditionNode(long id, RDXBehaviorTreeRootNode rootNode)
   {
      super(new ConditionNodeState(id, rootNode.getState()), rootNode);

      counter = new RDXCounterCondition(state);
      llm = new RDXLLMCondition(state);
      proximityCheck = new RDXProximityCondition(state, scene);
      shapeContains = new RDXShapeContainsCondition(this, scene, panel3D);
   }

   @Override
   public void update()
   {
      super.update();

      if (definition.getConditionType().getValue() == ConditionNodeType.SHAPE_CONTAINS)
         shapeContains.update();
   }

   @Override
   public void renderTreeViewRow()
   {
      super.renderRowBeginning();
      super.renderEditableName();

      ImGui.sameLine();
      conditionIconWidget.render();

      renderRowEnd();
   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      ConditionNodeType currentConditionType = definition.getConditionType().getValue();
      if (ImGui.beginCombo(labels.get("Condition Type"), currentConditionType.name()))
      {
         for (ConditionNodeType value : ConditionNodeType.values)
            if (ImGui.selectable(value.name(), value == currentConditionType))
               definition.getConditionType().setValue(value);

         ImGui.endCombo();
      }


      switch (currentConditionType)
      {
         case COUNTER -> counter.renderImGuiWidgetsInternal();
         case LLM -> llm.renderImGuiWidgetsInternal();
         case PROXIMITY -> proximityCheck.renderImGuiWidgetsInternal();
         case SHAPE_CONTAINS -> shapeContains.renderImGuiWidgetsInternal();
      }
   }

   @Override
   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      if (definition.getConditionType().getValue() == ConditionNodeType.SHAPE_CONTAINS)
         shapeContains.calculate3DViewPick(input);
   }

   @Override
   public void process3DViewInput(ImGui3DViewInput input)
   {
      if (definition.getConditionType().getValue() == ConditionNodeType.SHAPE_CONTAINS)
         shapeContains.process3DViewInput(input);
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (definition.getConditionType().getValue() == ConditionNodeType.SHAPE_CONTAINS)
         shapeContains.getVirtualRenderables(renderables, pool);
   }

   @Override
   public void deselectGizmos()
   {
      shapeContains.deselectGizmos();
   }

   @Override
   public String getLeafTypeTitle()
   {
      return "Condition Node";
   }
}
