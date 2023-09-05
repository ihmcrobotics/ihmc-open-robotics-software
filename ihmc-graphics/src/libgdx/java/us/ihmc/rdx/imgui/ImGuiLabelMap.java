package us.ihmc.rdx.imgui;

import java.util.HashMap;

/**
 * Use this class to create ImGui labels without reallocating the Strings every tick.
 * This is necessary for some widgets to have unique IDs so they display the same text
 * but are still addressable under the hood uniquely.
 */
public class ImGuiLabelMap
{
   private final HashMap<String, String> namesToLabels = new HashMap<>();

   public String get(String name)
   {
      String label = namesToLabels.get(name);
      if (label == null)
      {
         label = ImGuiTools.uniqueLabel(name);
         namesToLabels.put(name, label);
      }
      return label;
   }
}
