package us.ihmc.rdx.behaviorTree;

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
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.tools.ImGuiScrollableLogArea;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import javax.annotation.Nullable;
import java.util.ArrayList;
import java.util.List;

/**
 * The base class for an RDX node, which is the type that solely exists
 * in the UIs and is used by an operator to interact with the behavior system.
 *
 * @param <S> The type of this node's state instance.
 * @param <D> The type of this node's definition instance.
 */
public class RDXBehaviorTreeNode<S extends BehaviorTreeNodeState<D>,
                                 D extends BehaviorTreeNodeDefinition>
      implements BehaviorTreeNode<RDXBehaviorTreeNode<?, ?>, S, D>
{
   /** Convenient accessor to the state to keep the code clean, available to all inheriting classes. */
   protected final S state;
   /** Convenient accessor to the definition to keep the code clean, available to all inheriting classes. */
   protected final D definition;
   private final List<RDXBehaviorTreeNode<?, ?>> children = new ArrayList<>();
   private transient RDXBehaviorTreeNode<?, ?> parent;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean selected = new ImBoolean();
   private transient final ImVec2 lineMin = new ImVec2();
   private transient final ImVec2 indentMin = new ImVec2();
   private transient final ImVec2 lineMax = new ImVec2();
   private boolean mouseHoveringNodeLine;
   protected boolean anySpecificWidgetOnLineClicked = false;
   protected boolean treeWidgetExpanded = false;
   private int previousNumberOfChildren = 0;
   private boolean isNameBeingEdited = false;
   private transient final ImString imNodeNameText = new ImString();
   private transient final ImString notesText = new ImString(1500);
   private final String nodePopupID = labels.get("Node popup");
   private String modalPopupID = labels.get("Create node");
   private final ImGuiScrollableLogArea logArea = new ImGuiScrollableLogArea();
   private RDXBehaviorTreeNode<?, ?> draggedNode = null;
   private boolean dragging = false;
   private boolean dragReleasedBefore = false;
   private boolean dragReleasedAfter = false;

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
      state = (S) new BehaviorTreeNodeState<D>(id, definition);
   }

   @Override
   public void update()
   {
      BehaviorTreeNode.super.update();

      // Automatically expand if less than 5 children are added at once
      int deltaChildren = getChildren().size() - previousNumberOfChildren;
      previousNumberOfChildren = getChildren().size();
      if (deltaChildren > 0 && deltaChildren < 5)
         treeWidgetExpanded = true;

      while (!state.getLogger().getRecentMessages().isEmpty())
      {
         LogMessage message = state.getLogger().getRecentMessages().poll();
         logArea.submitEntry(message.instant(), message.level(), message.message());
         RDXBaseUI.pushNotification(message.message());
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

   /** Override to add node specific stuff */
   public void renderTreeViewRow()
   {
      renderRowBeginning();
      renderEditableName();
   }

   public void renderRowBeginning()
   {
      anySpecificWidgetOnLineClicked = false;

      ImGui.dummy(0.0f, ImGui.getFrameHeight()); // Make the lines as tall as when they have and input box
      ImGui.sameLine(0.0f, 0.0f);
      ImGui.alignTextToFramePadding(); // Centers the node descriptions vertically in the frame height area

      ImGui.getCursorScreenPos(indentMin);
      lineMin.x = ImGui.getWindowContentRegionMin().x + ImGui.getWindowPosX();
      lineMin.y = indentMin.y;
      lineMax.set(indentMin.x + ImGui.getContentRegionAvailX(), lineMin.y + ImGui.getFrameHeight());

      mouseHoveringNodeLine = ImGui.isWindowHovered();
      mouseHoveringNodeLine &= ImGui.getMousePosX() > lineMin.x && ImGui.getMousePosX() <= lineMax.x;
      mouseHoveringNodeLine &= ImGui.getMousePosY() > lineMin.y && ImGui.getMousePosY() <= lineMax.y;
      dragging = false;
      dragReleasedBefore = false;
      dragReleasedAfter = false;
      if (mouseHoveringNodeLine)
      {
         ImGui.getWindowDrawList().addRectFilled(lineMin.x, lineMin.y, lineMax.x, lineMax.y, ImGui.getColorU32(ImGuiCol.MenuBarBg));

         if (!isRootNode())
         {
            if (ImGui.isMouseDragging(ImGuiMouseButton.Left))
            {
               float dragStartX = ImGui.getMousePosX() - ImGui.getMouseDragDeltaX();
               float dragStartY = ImGui.getMousePosY() - ImGui.getMouseDragDeltaY();
               if (dragStartX > lineMin.x && dragStartX <= lineMax.x && dragStartY > lineMin.y && dragStartY <= lineMax.y)
                  dragging = true;
            }
            if (draggedNode != null && draggedNode != this)
            {
               float height = lineMax.y - lineMin.y;
               if (ImGui.getMousePosY() - lineMin.y <= 0.5f * height)
               {
                  ImGui.getWindowDrawList().addRectFilled(indentMin.x, lineMin.y, lineMax.x, lineMin.y + 0.15f * height, ImGui.getColorU32(ImGuiCol.CheckMark));
                  if (!ImGui.isMouseDown(ImGuiMouseButton.Left))
                     dragReleasedBefore = true;
               }
               else
               {
                  ImGui.getWindowDrawList().addRectFilled(indentMin.x, lineMin.y + 0.85f * height, lineMax.x, lineMax.y, ImGui.getColorU32(ImGuiCol.CheckMark));
                  if (!ImGui.isMouseDown(ImGuiMouseButton.Left))
                     dragReleasedAfter = true;
               }
            }
         }
      }

      float itemWidth = ImGui.getFontSize() * 1.0f;
      if (!getChildren().isEmpty()) // expand/collapse arrow
      {
         float width = ImGui.getFontSize() / 2.5f;
         float halfHeight = ImGui.getFrameHeight() * 0.5f / 2.0f;
         boolean isHovered = ImGuiTools.isItemHovered(itemWidth, ImGui.getFrameHeight());
         int color = isHovered ? ImGui.getColorU32(ImGuiCol.ButtonHovered) : ImGui.getColorU32(ImGuiCol.Text);
         if (treeWidgetExpanded)
         {
            float offsetX = ImGui.getCursorScreenPosX() + ImGui.getFontSize() * 0.2f;
            float offsetY = ImGui.getCursorScreenPosY() + ImGui.getFrameHeight() * 0.4f;
            ImGui.getWindowDrawList().addLine(offsetX, offsetY, offsetX + halfHeight, offsetY + width, color);
            ImGui.getWindowDrawList().addLine(offsetX + halfHeight, offsetY + width, offsetX + halfHeight * 2.0f, offsetY, color);
         }
         else
         {
            float offsetX = ImGui.getCursorScreenPosX() + width;
            float offsetY = ImGui.getCursorScreenPosY() + ImGui.getFrameHeight() * 0.2f;
            ImGui.getWindowDrawList().addLine(offsetX + width, offsetY + halfHeight, offsetX, offsetY + halfHeight * 2.0f, color);
            ImGui.getWindowDrawList().addLine(offsetX + width, offsetY + halfHeight, offsetX, offsetY, color);
         }
         if (isHovered && ImGui.isMouseClicked(ImGuiMouseButton.Left))
         {
            anySpecificWidgetOnLineClicked = true;
            treeWidgetExpanded = !treeWidgetExpanded;
         }
      }
      ImGui.setCursorScreenPos(indentMin.x + itemWidth, indentMin.y); // Leave space for the expand/collapse arrow regardless
   }

   public void renderEditableName()
   {
      String nameText = definition.getName();

      if (definition.hasChanges())
         nameText += "*";

      ImGui.setCursorScreenPos(indentMin.x + ImGui.getFontSize() * 3.0f, indentMin.y);

      boolean textHovered = ImGuiTools.isItemHovered(ImGuiTools.calcTextSizeX(nameText), ImGui.getFrameHeight());

      if (selected.get())
         ImGui.getWindowDrawList().addRectFilled(lineMin.x, lineMin.y, lineMax.x, lineMax.y, ImGui.getColorU32(ImGuiCol.Header));

      if (!isRootNode() && textHovered && ImGui.isMouseDoubleClicked(ImGuiMouseButton.Left))
      {
         anySpecificWidgetOnLineClicked = true;
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
      if (!isRootNode())
      {
         if (ImGui.menuItem(labels.get("Rename...")))
         {
            RDXBehaviorTreeTools.runForEntireTree(this, node -> node.setNameBeingEdited(false));
            isNameBeingEdited = true;
            imNodeNameText.set(definition.getName());
         }

         ImGui.separator();
      }
      else
      {
         if (ImGui.menuItem(labels.get("Expand all nodes"))) // TODO: Maybe render the icon too
            expandCollapseAll(true, this);
         if (ImGui.menuItem(labels.get("Collapse all nodes")))
            expandCollapseAll(false, this);
         ImGui.separator();
      }

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
      else if (!isRootNode())
      {
         if (ImGui.menuItem(labels.get("Convert to JSON Root")))
         {
            definition.setName(definition.getName() + ".json");
         }
      }

      if (ImGui.menuItem(labels.get("Draw to SVG")))
      {
         state.drawToSVG();
      }
   }

   private void expandCollapseAll(boolean expandOrCollapse, RDXBehaviorTreeNode<?, ?> node)
   {
      node.setTreeWidgetExpanded(expandOrCollapse);

      for (RDXBehaviorTreeNode<?, ?> child : node.getChildren())
      {
         expandCollapseAll(expandOrCollapse, child);
      }
   }

   public void renderNodeSettingsWidgets()
   {
      logArea.renderImGuiWidgets();

      ImGui.text("Notes:");
      String notes = definition.getNotes();
      String modifiedNotes = ImGuiTools.inputTextMultiline(labels.getHidden("Notes"), notes, notesText);
      if (modifiedNotes != null)
      {
         definition.setNotes(modifiedNotes);
      }
   }

   public void clearSelections()
   {
      selected.set(false);
      isNameBeingEdited = false;
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {

   }

   public boolean getSelected()
   {
      return selected.get();
   }

   public void setNameBeingEdited(boolean nameBeingEdited)
   {
      isNameBeingEdited = nameBeingEdited;
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

   public void setDraggedNode(RDXBehaviorTreeNode<?, ?> draggedNode)
   {
      this.draggedNode = draggedNode;
   }

   public boolean getDragging()
   {
      return dragging;
   }

   public boolean getDragReleasedBefore()
   {
      return dragReleasedBefore;
   }

   public boolean getDragReleasedAfter()
   {
      return dragReleasedAfter;
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
