package us.ihmc.rdx.ui.behavior.editor;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.type.ImBoolean;
import imgui.type.ImString;
import us.ihmc.behaviors.sequence.BehaviorActionState;
import us.ihmc.rdx.imgui.ImBooleanWrapper;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.vr.RDXVRContext;

/**
 * The UI representation of a robot behavior action. It provides a base
 * template for implementing an interactable action.
 */
public interface RDXBehaviorAction extends BehaviorActionState
{
   // TODO: Probably remove and use BehaviorActionDefinition#update?
   default void update(boolean concurrentActionIsNextForExecution)
   {

   }

   default void updateAfterLoading()
   {

   }

   default void updateBeforeRemoving()
   {

   }

   default void calculateVRPick(RDXVRContext vrContext)
   {

   }

   default void processVRInput(RDXVRContext vrContext)
   {

   }

   default void calculate3DViewPick(ImGui3DViewInput input)
   {

   }

   default void process3DViewInput(ImGui3DViewInput input)
   {

   }

   default void renderImGuiWidgets()
   {

   }

   default void renderImGuiSettingWidgets()
   {

   }

   default void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {

   }

   ImBooleanWrapper getSelected();

   ImBoolean getExpanded();

   // TODO: Put in BehaviorActionState
   String getActionTypeTitle();

   ImString getImDescription();

   ImString getRejectionTooltip();

   // TODO: Put in BehaviorActionState
   int getActionIndex();

   // TODO: Put in BehaviorActionState
   void setActionIndex(int actionIndex);

   // TODO: Put in BehaviorActionState
   int getActionNextExecutionIndex();

   // TODO: Put in BehaviorActionState
   void setActionNextExecutionIndex(int actionNextExecutionIndex);
}
