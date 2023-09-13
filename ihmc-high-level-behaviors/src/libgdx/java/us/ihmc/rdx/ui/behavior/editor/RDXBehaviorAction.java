package us.ihmc.rdx.ui.behavior.editor;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.type.ImBoolean;
import imgui.type.ImString;
import us.ihmc.behaviors.sequence.BehaviorActionData;
import us.ihmc.rdx.imgui.ImBooleanCheckbox;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.vr.RDXVRContext;

/**
 * The UI representation of a robot behavior action. It provides a base
 * template for implementing an interactable action.
 */
public abstract class RDXBehaviorAction
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBooleanCheckbox selected = new ImBooleanCheckbox(labels.get("Selected"));
   private final ImBooleanCheckbox executeWithNextAction = new ImBooleanCheckbox(labels.get("Execute With Next Action"));
   private final ImBoolean expanded = new ImBoolean(true);
   private final ImString description = new ImString();
   private int actionIndex = -1;
   private int actionNextExecutionIndex = -1;

   public RDXBehaviorAction()
   {

   }

   public void update()
   {

   }

   public void updateAfterLoading()
   {

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

   public final void renderImGuiWidgets()
   {
      if (expanded.get())
      {
         renderImGuiSettingWidgets();
      }
   }

   public void renderImGuiSettingWidgets()
   {

   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {

   }

   public abstract BehaviorActionData getActionData();

   public ImBooleanCheckbox getSelected()
   {
      return selected;
   }

   public ImBooleanCheckbox getExecutionWithNextAction()
   {
      return executeWithNextAction;
   }

   public ImBoolean getExpanded()
   {
      return expanded;
   }

   public abstract String getActionTypeTitle();

   public ImString getDescription()
   {
      return description;
   }

   public int getActionIndex()
   {
      return actionIndex;
   }

   public void setActionIndex(int actionIndex)
   {
      this.actionIndex = actionIndex;
   }

   public int getActionNextExecutionIndex()
   {
      return actionNextExecutionIndex;
   }

   public void setActionNextExecutionIndex(int actionNextExecutionIndex)
   {
      this.actionNextExecutionIndex = actionNextExecutionIndex;
   }
}
