package us.ihmc.rdx.imgui;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

import java.util.SortedSet;
import java.util.TreeSet;
import java.util.function.Supplier;

/**
 * Used to select between the reference frames in a library by human readable names.
 */
public class ImGuiReferenceFrameLibraryCombo
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final String comboName;
   private final ReferenceFrameLibrary referenceFrameLibrary;
   private final Supplier<String> currentFrameNameSupplier;
   /** Stores a history of all frames that were ever here. */
   private final SortedSet<String> selectableReferenceFrameNames = new TreeSet<>();
   private final SortedSet<String> referenceFrameLibraryNames = new TreeSet<>();
   private int selectedFrameIndex;
   private transient String[] selectableReferenceFrameNameArray = new String[0];

   public ImGuiReferenceFrameLibraryCombo(String comboName, ReferenceFrameLibrary referenceFrameLibrary, Supplier<String> currentFrameNameSupplier)
   {
      this.comboName = comboName;
      this.referenceFrameLibrary = referenceFrameLibrary;
      this.currentFrameNameSupplier = currentFrameNameSupplier;
   }

   public boolean render()
   {
      referenceFrameLibraryNames.clear();
      referenceFrameLibrary.getAllFrameNames(referenceFrameLibraryNames::add);

      selectableReferenceFrameNames.add(currentFrameNameSupplier.get());
      selectableReferenceFrameNames.addAll(referenceFrameLibraryNames);

      selectableReferenceFrameNameArray = selectableReferenceFrameNames.toArray(selectableReferenceFrameNameArray);

      if (ImGui.beginCombo(labels.get(comboName), selectableReferenceFrameNameArray[selectedFrameIndex]))
      {
         for (int i = 0; i < selectableReferenceFrameNameArray.length; i++)
         {
            String referenceFrameName = selectableReferenceFrameNameArray[i];
            boolean libraryContainsFrame = referenceFrameLibraryNames.contains(referenceFrameName);

            if (!libraryContainsFrame)
               ImGui.pushStyleColor(ImGuiCol.Text, ImGuiTools.RED);

            if (ImGui.selectable(selectableReferenceFrameNameArray[i], selectedFrameIndex == i))
               selectedFrameIndex = i;

            if (!libraryContainsFrame)
               ImGui.popStyleColor();
         }

         ImGui.endCombo();
      }

      return true;
   }
}
