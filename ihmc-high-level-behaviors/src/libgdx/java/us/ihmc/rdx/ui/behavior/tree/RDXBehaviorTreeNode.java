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
import us.ihmc.behaviors.behaviorTree.log.BehaviorTreeNodeMessageLogger.LogMessage;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.ImGuiExpandCollapseRenderer;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.ImGuiVerticalAligner;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.tools.ImGuiLogWidget;
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
   private final D definition;
   private final List<RDXBehaviorTreeNode<?, ?>> children = new ArrayList<>();
   private transient RDXBehaviorTreeNode<?, ?> parent;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean selected = new ImBoolean();
   private transient final ImVec2 lineMin = new ImVec2();
   private transient final ImVec2 lineMax = new ImVec2();
   private final ImGuiExpandCollapseRenderer expandCollapseRenderer = new ImGuiExpandCollapseRenderer();
   private boolean mouseHoveringNodeLine;
   private boolean anySpecificWidgetOnLineClicked = false;
   private boolean treeWidgetExpanded = false;
   private boolean isNameBeingEdited = false;
   private transient final ImString imNodeNameText = new ImString();
   private transient final ImString notesText = new ImString(1500);
   private final String nodePopupID = labels.get("Node popup");
   private String modalPopupID = labels.get("Create node");
   private final ImGuiVerticalAligner childrenDescriptionAligner = new ImGuiVerticalAligner();
   private final ImGuiLogWidget logMessages = new ImGuiLogWidget();

   /** For extending types. */
   public RDXBehaviorTreeNode(S state)
   {
      this.state = state;
      definition = state.getDefinition();
   }

   /** For creating a basic node. */
   @SuppressWarnings("unchecked")
   public RDXBehaviorTreeNode(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      definition = (D) new BehaviorTreeNodeDefinition(crdtInfo, saveFileDirectory);
      state = (S) new BehaviorTreeNodeState<D>(id, definition, crdtInfo);

      definition.setName("Basic node");
   }

   @Override
   public void update()
   {
      BehaviorTreeNodeLayer.super.update();
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

   public void renderGeneralRowBeginWidgets()
   {
      anySpecificWidgetOnLineClicked = false;

      ImGui.dummy(0.0f, ImGui.getFrameHeight()); // Make the lines as tall as when they have and input box
      ImGui.sameLine(0.0f, 0.0f);

      ImGui.alignTextToFramePadding(); // Centers the node descriptions vertically in the frame height area

      ImGui.getCursorScreenPos(lineMin);
      lineMax.set(lineMin.x + ImGui.getContentRegionAvailX(), lineMin.y + ImGui.getFrameHeightWithSpacing());

      mouseHoveringNodeLine = ImGuiTools.isItemHovered(ImGui.getContentRegionAvailX(), ImGui.getFrameHeight());
      if (mouseHoveringNodeLine)
      {
         ImGui.getWindowDrawList().addRectFilled(lineMin.x, lineMin.y, lineMax.x, lineMax.y, ImGui.getColorU32(ImGuiCol.MenuBarBg));
      }

      if (!getChildren().isEmpty())
      {
         if (expandCollapseRenderer.render(treeWidgetExpanded, false, ImGui.getFrameHeight()))
         {
            anySpecificWidgetOnLineClicked = true;
            treeWidgetExpanded = !treeWidgetExpanded;
         }
         ImGui.sameLine();
      }
      else
      {
         treeWidgetExpanded = false;
      }
   }

   public void renderTreeViewIconArea()
   {

   }

   public void renderNodeName()
   {
      String nameText = definition.getName();

      if (definition.hasChanges())
         nameText += "*";

      boolean textHovered = ImGuiTools.isItemHovered(ImGuiTools.calcTextSizeX(nameText), ImGui.getFrameHeight());

      if (selected.get())
      {
         ImGui.getWindowDrawList().addRectFilled(lineMin.x, lineMin.y, lineMax.x, lineMax.y, ImGui.getColorU32(ImGuiCol.Header));
      }

      if (textHovered && ImGui.isMouseDoubleClicked(ImGuiMouseButton.Left))
      {
         setSpecificWidgetOnRowClicked();
         RDXBehaviorTreeTools.clearOtherNodeSelections(this);
         selected.set(true);
         isNameBeingEdited = true;
         imNodeNameText.set(definition.getName());
      }

      if (isNameBeingEdited)
      {
         if (ImGuiTools.inputText(labels.getHidden("name"), imNodeNameText))
         {
            definition.setName(imNodeNameText.get());
            isNameBeingEdited = false;
         }
      }
      else
      {
         ImGui.textColored(getNameColor(), nameText);
      }

      if (mouseHoveringNodeLine && !isNameBeingEdited && ImGui.isMouseClicked(ImGuiMouseButton.Right))
      {
         RDXBehaviorTreeTools.clearOtherNodeSelections(this);
         selected.set(true);
         ImGui.openPopup(nodePopupID);
      }

      // We try to make anywhere on the row clickable to select the node,
      // execpt for specific interactions
      if (!anySpecificWidgetOnLineClicked && mouseHoveringNodeLine && ImGui.isMouseClicked(ImGuiMouseButton.Left) && !isNameBeingEdited)
      {
         boolean desiredValue = !selected.get();
         RDXBehaviorTreeTools.clearOtherNodeSelections(this);
         selected.set(desiredValue);
      }
   }

   public void renderContextMenuItems()
   {
      if (ImGui.menuItem(labels.get("Rename...")))
      {
         RDXBehaviorTreeTools.runForEntireTree(this, node -> node.setNameBeingEdited(false));
         isNameBeingEdited = true;
         imNodeNameText.set(definition.getName());
      }

      ImGui.separator();

      if (definition.isJSONRoot())
      {
         if (ImGui.menuItem(labels.get("Save to File")))
         {
            RDXBaseUI.pushNotification("Saving %s".formatted(definition.getName()));
            definition.saveToFile();
         }
         if (ImGui.menuItem(labels.get("Unlink from JSON File")))
         {
            definition.setName(definition.getName().replace(".json", ""));
         }
      }
      else
      {
         if (ImGui.menuItem(labels.get("Convert to JSON Root")))
         {
            definition.setName(definition.getName() + ".json");
         }
      }
   }

   public void renderNodeSettingsWidgets()
   {
      ImGui.text("Notes:");
      notesText.set(definition.getNotes());
      ImGui.setNextItemWidth(ImGui.getColumnWidth());
      if (ImGui.inputTextMultiline(labels.getHidden("Notes"), notesText))
      {
         definition.setNotes(notesText.get());
      }

      while (!state.getLogger().getRecentMessages().isEmpty())
      {
         LogMessage message = state.getLogger().getRecentMessages().poll();
         logMessages.submitEntry(message.level(), message.message());
      }
      logMessages.renderImGuiWidgets();
   }

   public void clearSelections()
   {
      selected.set(false);
      isNameBeingEdited = false;
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {

   }

   @Override
   public void destroy()
   {
      LogTools.info("Destroying node: {}:{}", definition.getName(), state.getID());
      state.destroy();
   }

   public boolean getSelected()
   {
      return selected.get();
   }

   protected void setSpecificWidgetOnRowClicked()
   {
      anySpecificWidgetOnLineClicked = true;
   }

   public void setNameBeingEdited(boolean nameBeingEdited)
   {
      isNameBeingEdited = nameBeingEdited;
   }

   public boolean getNameBeingEdited()
   {
      return isNameBeingEdited;
   }

   public int getNameColor()
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
      return state;
   }

   @Override
   public D getDefinition()
   {
      return definition;
   }

   @Override
   public S getState()
   {
      return state;
   }
}
