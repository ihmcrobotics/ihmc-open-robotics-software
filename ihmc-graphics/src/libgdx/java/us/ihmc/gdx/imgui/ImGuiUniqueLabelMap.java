package us.ihmc.gdx.imgui;

import java.util.HashMap;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * Use this class to create ImGui labels without reallocating the Strings every tick.
 * This is necessary for some widgets to have unique IDs so they display the same text
 * but are still addressable under the hood uniquely.
 */
public class ImGuiUniqueLabelMap
{
   private static final HashMap<String, AtomicInteger> CLASS_TO_INDEX = new HashMap<>();

   private final HashMap<String, String> namesToLabels = new HashMap<>();
   private final int index;
   private final String simpleName;

   public ImGuiUniqueLabelMap(Class<?> clazz)
   {
      simpleName = clazz.getSimpleName();
      AtomicInteger classIndex = CLASS_TO_INDEX.get(simpleName);
      if (classIndex == null)
      {
         classIndex = new AtomicInteger();
         CLASS_TO_INDEX.put(simpleName, classIndex);
      }
      index = classIndex.getAndIncrement();
   }

   public String get(String name)
   {
      String label = namesToLabels.get(name);
      if (label == null)
      {
         label = ImGuiTools.uniqueLabel(simpleName + index, name);
         namesToLabels.put(name, label);
      }
      return label;
   }

   public String get(String name, int moreSpecificIndex)
   {
      String label = namesToLabels.get(name + moreSpecificIndex);
      if (label == null)
      {
         label = ImGuiTools.uniqueLabel(simpleName + index + "_" + moreSpecificIndex, name);
         namesToLabels.put(name + moreSpecificIndex, label);
      }
      return label;
   }
}
