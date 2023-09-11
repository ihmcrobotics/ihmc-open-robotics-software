package us.ihmc.rdx.ui.behavior.editor;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImString;
import org.apache.commons.lang3.mutable.MutableBoolean;
import us.ihmc.behaviors.sequence.BehaviorActionData;
import us.ihmc.rdx.imgui.ImBooleanWrapper;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.vr.RDXVRContext;

/**
 * The UI representation of a robot behavior action. It provides a base
 * template for implementing an interactable action.
 */
public abstract class RDXBehaviorAction
{
   private final MutableBoolean selected = new MutableBoolean();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   protected final ImBooleanWrapper selectedWrapper = new ImBooleanWrapper(selected::booleanValue,
                                                                           selected::setValue,
                                                                           imBoolean -> ImGui.checkbox(labels.get("Selected"), imBoolean));
   protected final MutableBoolean executeWithNextAction = new MutableBoolean(false);
   protected final ImBooleanWrapper executeWithNextActionWrapper = new ImBooleanWrapper(executeWithNextAction::booleanValue,
                                                                                      executeWithNextAction::setValue,
                                                                                      imBoolean -> ImGui.checkbox(labels.get("Execute With Next Action"),
                                                                                                                  imBoolean));
   private final ImBoolean expanded = new ImBoolean(true);
   private final ImString description = new ImString();
   private int actionIndex = -1;
   private int actionNextExcecutionIndex = -1;

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

   public ImBooleanWrapper getSelected()
   {
      return selectedWrapper;
   }

   public ImBooleanWrapper getExecutionWithNextAction()
   {
      return executeWithNextActionWrapper;
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

   public int getActionNextExcecutionIndex()
   {
      return actionNextExcecutionIndex;
   }

   public void setActionNextExcecutionIndex(int actionNextExcecutionIndex)
   {
      this.actionNextExcecutionIndex = actionNextExcecutionIndex;
   }
}
