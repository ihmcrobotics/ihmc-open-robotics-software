package us.ihmc.rdx.imgui;

import com.badlogic.gdx.graphics.Color;
import imgui.ImGui;
import imgui.flag.ImGuiCol;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.referenceFrames.ReferenceFrameSupplier;

/**
 * Used to select between the reference frames in a library by human readable names.
 */
public class ImGuiReferenceFrameLibraryCombo
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ReferenceFrameLibrary referenceFrameLibrary;
   private String invalidReferenceFrameName;
   private int selectedFrameIndex;

   public ImGuiReferenceFrameLibraryCombo(ReferenceFrameLibrary referenceFrameLibrary)
   {
      this.referenceFrameLibrary = referenceFrameLibrary;
   }

   public boolean render()
   {
      String[] referenceFrameNameArray;

      if (invalidReferenceFrameName != null)
      {
         String[] referenceFrameNameArrayWithInvalidReferenceFrameName = new String[referenceFrameLibrary.getReferenceFrameNameArray().length + 1];
         referenceFrameNameArrayWithInvalidReferenceFrameName[0] =  invalidReferenceFrameName + " [invalid]";
         for (int i = 1; i <= referenceFrameLibrary.getReferenceFrameNameArray().length; i++)
         {
            referenceFrameNameArrayWithInvalidReferenceFrameName[i] = referenceFrameLibrary.getReferenceFrameNameArray()[i - 1];
         }
         referenceFrameNameArray = referenceFrameNameArrayWithInvalidReferenceFrameName;
      }
      else
      {
         referenceFrameNameArray = referenceFrameLibrary.getReferenceFrameNameArray();
      }

      if (ImGui.beginCombo(labels.get("Reference frame"), referenceFrameNameArray[selectedFrameIndex]))
      {
         for (int i = 0; i < referenceFrameNameArray.length; i++)
         {
            if (referenceFrameNameArray[i].contains("invalid"))
            {
               ImGui.pushStyleColor(ImGuiCol.Text, Color.RED.toIntBits());
            }
            boolean isSelected = selectedFrameIndex == i;
            if (ImGui.selectable(referenceFrameNameArray[i], isSelected))
            {
               selectedFrameIndex = i;
            }
            if (referenceFrameNameArray[i].contains("invalid"))
            {
               ImGui.popStyleColor();
            }
         }
         ImGui.endCombo();
      }

      return true;
   }

   public boolean setSelectedReferenceFrame(String referenceFrameName)
   {
      int frameIndex = referenceFrameLibrary.findFrameIndexByName(referenceFrameName);
      boolean frameFound = frameIndex >= 0;
      if (frameFound)
      {
         invalidReferenceFrameName = null;
         selectedFrameIndex = frameIndex;
      }
      else
      {
         invalidReferenceFrameName = referenceFrameName;
      }
      return frameFound;
   }

   public ReferenceFrameSupplier getSelectedReferenceFrame()
   {
      return referenceFrameLibrary.findFrameByIndex(selectedFrameIndex);
   }
}
