package us.ihmc.rdx.imgui;

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
   private final String classSimpleName;

   public ImGuiUniqueLabelMap(Class<?> clazz)
   {
      classSimpleName = clazz.getSimpleName();
      AtomicInteger classIndex = CLASS_TO_INDEX.get(classSimpleName);
      if (classIndex == null)
      {
         classIndex = new AtomicInteger();
         CLASS_TO_INDEX.put(classSimpleName, classIndex);
      }
      index = classIndex.getAndIncrement();
   }

   public String get(String visibleLabel)
   {
      String label = namesToLabels.get(visibleLabel);
      if (label == null)
      {
         label = ImGuiTools.uniqueLabel(classSimpleName + index, visibleLabel);
         namesToLabels.put(visibleLabel, label);
      }
      return label;
   }

   public String getHidden(String hiddenQualifier)
   {
      return get("###" + hiddenQualifier);
   }

   public String get(String visibleLabel, String hiddenQualifier)
   {
      String label = namesToLabels.get(visibleLabel + hiddenQualifier);
      if (label == null)
      {
         label = ImGuiTools.uniqueLabel(classSimpleName + index + "_" + hiddenQualifier, visibleLabel);
         namesToLabels.put(visibleLabel, label);
      }
      return label;
   }

   public String get(String visibleLabel, int moreSpecificIndex)
   {
      String label = namesToLabels.get(visibleLabel + moreSpecificIndex);
      if (label == null)
      {
         label = ImGuiTools.uniqueLabel(classSimpleName + index + "_" + moreSpecificIndex, visibleLabel);
         namesToLabels.put(visibleLabel + moreSpecificIndex, label);
      }
      return label;
   }

   public String get(String visibleLabel, String hiddenQualifier, int moreSpecificIndex)
   {
      String label = namesToLabels.get(visibleLabel + hiddenQualifier + moreSpecificIndex);
      if (label == null)
      {
         label = ImGuiTools.uniqueLabel(classSimpleName + index + "_" + moreSpecificIndex + "_" + hiddenQualifier, visibleLabel);
         namesToLabels.put(visibleLabel + moreSpecificIndex, label);
      }
      return label;
   }
}
