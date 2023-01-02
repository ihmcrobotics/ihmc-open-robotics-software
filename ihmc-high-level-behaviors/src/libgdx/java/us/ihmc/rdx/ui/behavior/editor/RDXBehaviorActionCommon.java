package us.ihmc.rdx.ui.behavior.editor;

import imgui.type.ImBoolean;

public class RDXBehaviorActionCommon
{
   private String nameForDisplay = "";
   private final ImBoolean selected = new ImBoolean();
   private final ImBoolean expanded = new ImBoolean(true);

   public RDXBehaviorActionCommon(String nameForDisplay)
   {
      this.nameForDisplay = nameForDisplay;
   }

   public ImBoolean getSelected()
   {
      return selected;
   }

   public ImBoolean getExpanded()
   {
      return expanded;
   }

   public void setNameForDisplay(String nameForDisplay)
   {
      this.nameForDisplay = nameForDisplay;
   }

   public String getNameForDisplay()
   {
      return nameForDisplay;
   }
}
