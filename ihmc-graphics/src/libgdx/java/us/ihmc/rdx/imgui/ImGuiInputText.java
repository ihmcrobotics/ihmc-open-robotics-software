package us.ihmc.rdx.imgui;

import imgui.type.ImString;

public class ImGuiInputText extends ImGuiFancyWidget
{
   private final ImString imString;

   public ImGuiInputText(String label)
   {
      super(label, null);
      imString = new ImString();
   }

   public boolean render()
   {
      beforeWidgetRender();
      boolean valueChanged = ImGuiTools.inputText(label, imString);
      afterWidgetRender();
      return valueChanged;
   }

   public void setImString(String value)
   {
      imString.set(value);
   }

   public String getString()
   {
      return imString.get();
   }

   public ImString getImString()
   {
      return imString;
   }
}