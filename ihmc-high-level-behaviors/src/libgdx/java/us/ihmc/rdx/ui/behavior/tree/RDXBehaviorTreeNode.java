package us.ihmc.rdx.ui.behavior.tree;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.ImVec2;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiMouseButton;
import imgui.type.ImBoolean;
import imgui.type.ImString;
import us.ihmc.behaviors.behaviorTree.*;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.ImGuiVerticalAligner;
import us.ihmc.rdx.imgui.ImStringWrapper;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import javax.annotation.Nullable;
import java.util.ArrayList;
import java.util.List;

public class RDXBehaviorTreeNode<S extends BehaviorTreeNodeState<D>,
                                 D extends BehaviorTreeNodeDefinition>
      implements BehaviorTreeNodeLayer<RDXBehaviorTreeNode<?, ?>, S, S, D>
{
   private final S state;
   private final List<RDXBehaviorTreeNode<?, ?>> children = new ArrayList<>();
   private transient RDXBehaviorTreeNode<?, ?> parent;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private ImStringWrapper descriptionWrapper;
   private boolean treeWidgetExpanded = false;
   private boolean isDescriptionBeingEdited = false;
   private transient final ImString imDescriptionText = new ImString();
   private transient final ImString imJSONFileNameText = new ImString();
   private transient final ImVec2 descriptionTextSize = new ImVec2();
   private transient final ImBoolean isJSONFileRoot = new ImBoolean();
   private final String nodePopupID = labels.get("Node popup");
   private String modalPopupID = labels.get("Create node");
   private boolean nodeContextMenuShowing = false;
   private final ImGuiVerticalAligner childrenDescriptionAligner = new ImGuiVerticalAligner();

   /** For extending types. */
   public RDXBehaviorTreeNode(S state)
   {
      this.state = state;
   }

   /** For creating a basic node. */
   @SuppressWarnings("unchecked")
   public RDXBehaviorTreeNode(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      D definition = (D) new BehaviorTreeNodeDefinition(crdtInfo, saveFileDirectory);
      this.state = (S) new BehaviorTreeNodeState<D>(id, definition, crdtInfo);

      getDefinition().setDescription("BasicNode");
   }

   @Override
   public void update()
   {
      BehaviorTreeNodeLayer.super.update();

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

      if (textHovered && ImGui.isMouseClicked(ImGuiMouseButton.Left))
      {
         treeWidgetExpanded = !treeWidgetExpanded;
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
         // We want the text to stay highlighted when the context menu is showing to help the operator
         // know which node they're operating on.
         boolean highlightText = textHovered || nodeContextMenuShowing;
         ImGui.textColored(highlightText ? ImGui.getColorU32(ImGuiCol.ButtonHovered) : getDescriptionColor(), descriptionText);
         nodeContextMenuShowing = false;
         ImGui.popFont();
      }

      if (textHovered && !isDescriptionBeingEdited && ImGui.isMouseClicked(ImGuiMouseButton.Right))
      {
         ImGui.openPopup(nodePopupID);
      }
   }

   public void renderContextMenuItems()
   {
      nodeContextMenuShowing = true;

      if (ImGui.menuItem(labels.get("Rename...")))
      {
         RDXBehaviorTreeTools.runForSubtreeNodes(RDXBehaviorTreeTools.findRootNode(this), node -> node.setDescriptionBeingEdited(false));
         isDescriptionBeingEdited = true;
         imDescriptionText.set(getDefinition().getDescription());
      }

      ImGui.separator();

      if (ImGui.beginMenu(labels.get("File")))
      {
         isJSONFileRoot.set(getDefinition().isJSONRoot());

         if (ImGui.checkbox(labels.get("Is JSON File Root"), isJSONFileRoot))
         {
            if (isJSONFileRoot.get())
            {
               getDefinition().setJSONFileName(getClass().getSimpleName() + ".json");
            }
            else
            {
               getDefinition().setJSONFileName("");
            }
         }
         if (getDefinition().isJSONRoot())
         {
            imJSONFileNameText.set(getDefinition().getJSONFilename().replaceAll("\\.json$", ""));
            ImGui.text("File name:");
            ImGui.sameLine();
            ImGui.setNextItemWidth(ImGuiTools.calcTextSizeX(imJSONFileNameText.get()) + 10.0f);
            if (ImGui.inputText(labels.get(".json"), imJSONFileNameText))
            {
               getDefinition().setJSONFileName(imJSONFileNameText.get() + ".json");
            }
            if (ImGui.menuItem(labels.get("Save to File")))
            {
               RDXBaseUI.getInstance().getPrimary3DPanel().getNotificationManager().pushNotification("Saving %s".formatted(getDefinition().getJSONFilename()));
               getDefinition().saveToFile();
            }
         }
         ImGui.endMenu();
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

   public int getDescriptionColor()
   {
      return ImGui.getColorU32(ImGuiCol.Text);
   }

   public void setTreeWidgetExpanded(boolean treeWidgetExpanded)
   {
      this.treeWidgetExpanded = treeWidgetExpanded;
   }

   public boolean getTreeWidgetExpanded()
   {
      return treeWidgetExpanded;
   }

   public ImGuiVerticalAligner getChildrenDescriptionAligner()
   {
      return childrenDescriptionAligner;
   }

   public String getNodePopupID()
   {
      return nodePopupID;
   }

   public void setModalPopupTitle(String modalPopupTitle)
   {
      modalPopupID = labels.get(modalPopupTitle);
   }

   public String getModalPopupID()
   {
      return modalPopupID;
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
   public S getNextLowerLayer()
   {
      return getState();
   }

   @Override
   public D getDefinition()
   {
      return getState().getDefinition();
   }

   @Override
   public S getState()
   {
      return state;
   }
}
