package us.ihmc.rdx.ui.behavior.tree;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.ImVec2;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiMouseButton;
import imgui.type.ImString;
import us.ihmc.behaviors.behaviorTree.*;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.ImStringWrapper;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.vr.RDXVRContext;

import javax.annotation.Nullable;
import java.util.ArrayList;
import java.util.List;

public abstract class RDXBehaviorTreeNode<S extends BehaviorTreeNodeState<D>,
                                          D extends BehaviorTreeNodeDefinition>
      implements BehaviorTreeNodeExtension<RDXBehaviorTreeNode<?, ?>, S, S, D>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private ImStringWrapper descriptionWrapper;

   private final List<RDXBehaviorTreeNode<?, ?>> children = new ArrayList<>();
   private transient RDXBehaviorTreeNode<?, ?> parent;
   private boolean treeWidgetExpanded = false;
   private boolean isDescriptionBeingEdited = false;
   private transient final ImString imDescriptionText = new ImString();
   private transient final ImVec2 descriptionTextSize = new ImVec2();
   private final String nodePopupID = labels.get("Node popup");

   @Override
   public void update()
   {
      BehaviorTreeNodeExtension.super.update();

      if (descriptionWrapper == null)
      {
         descriptionWrapper = new ImStringWrapper(getDefinition()::getDescription,
                                                  getDefinition()::setDescription,
                                                  imString -> ImGuiTools.inputText(labels.getHidden("description"), imString));
      }
   }

   public void calculateVRPick(RDXVRContext vrContext)
   {

   }

   public void processVRInput(RDXVRContext vrContext)
   {

   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {

   }

   public void process3DViewInput(ImGui3DViewInput input)
   {

   }

   public void renderTreeViewIconArea()
   {

   }

   public void renderNodeDescription()
   {
      String descriptionText = getDefinition().getDescription();
      ImGui.calcTextSize(descriptionTextSize, descriptionText);
      boolean textHovered = ImGuiTools.isItemHovered(descriptionTextSize.x);

      if (textHovered && ImGui.isMouseDoubleClicked(ImGuiMouseButton.Left))
      {
         RDXBehaviorTreeTools.runForSubtreeNodes(RDXBehaviorTreeTools.findRootNode(this), node -> node.setDescriptionBeingEdited(false));
         isDescriptionBeingEdited = true;
         imDescriptionText.set(getDefinition().getDescription());
      }

      if (isDescriptionBeingEdited)
      {
         if (ImGuiTools.inputText(labels.getHidden("description"), imDescriptionText))
         {
            getDefinition().setDescription(imDescriptionText.get());
            isDescriptionBeingEdited = false;
         }
      }
      else
      {
         ImGui.pushFont(ImGuiTools.getSmallBoldFont());
         ImGui.textColored(textHovered ? ImGui.getColorU32(ImGuiCol.ButtonHovered) : ImGui.getColorU32(ImGuiCol.Text), descriptionText);
         ImGui.popFont();
      }

      if (textHovered && !isDescriptionBeingEdited && ImGui.isMouseClicked(ImGuiMouseButton.Right))
      {
         ImGui.openPopup(nodePopupID);
      }
   }

   public void renderImGuiWidgets()
   {

   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {

   }

   @Override
   public void destroy()
   {
      LogTools.info("Destroying node: {}:{}", getState().getDefinition().getDescription(), getState().getID());
      getState().destroy();
   }

   public ImStringWrapper getDescriptionWrapper()
   {
      return descriptionWrapper;
   }

   public void setDescriptionBeingEdited(boolean descriptionBeingEdited)
   {
      isDescriptionBeingEdited = descriptionBeingEdited;
   }

   public boolean getDescriptionBeingEdited()
   {
      return isDescriptionBeingEdited;
   }

   public void setTreeWidgetExpanded(boolean treeWidgetExpanded)
   {
      this.treeWidgetExpanded = treeWidgetExpanded;
   }

   public boolean getTreeWidgetExpanded()
   {
      return treeWidgetExpanded;
   }

   public String getNodePopupID()
   {
      return nodePopupID;
   }

   public List<RDXBehaviorTreeNode<?, ?>> getChildren()
   {
      return children;
   }

   @Override
   public void setParent(@Nullable RDXBehaviorTreeNode<?, ?> parent)
   {
      this.parent = parent;
   }

   @Nullable
   @Override
   public RDXBehaviorTreeNode<?, ?> getParent()
   {
      return parent;
   }

   @Override
   public S getExtendedNode()
   {
      return getState();
   }

   @Override
   public D getDefinition()
   {
      return getState().getDefinition();
   }
}
