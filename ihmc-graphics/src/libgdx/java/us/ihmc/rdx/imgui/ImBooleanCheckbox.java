package us.ihmc.rdx.imgui;

import imgui.ImGui;
import imgui.type.ImBoolean;

/**
 * A class for holding onto an ImBoolean and precomputed label
 * and making rendering it a simple call.
 */
public class ImBooleanCheckbox
{
   private final String label;
   private final ImBoolean imBoolean = new ImBoolean();

   public ImBooleanCheckbox(String label)
   {
      this.label = label;
   }

   public void renderImGuiWidget()
   {
      ImGui.checkbox(label, imBoolean);
   }

   public void set(boolean value)
   {
      imBoolean.set(value);
   }

   public boolean get()
   {
      return imBoolean.get();
   }
}
