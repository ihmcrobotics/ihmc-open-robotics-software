package us.ihmc.rdx.imgui;

import com.badlogic.gdx.graphics.Color;
import imgui.ImGui;
import imgui.flag.ImGuiCol;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.ConditionalReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

/**
 * Used to select between the reference frames in a library by human readable names.
 */
public class ImGuiReferenceFrameLibraryCombo
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ReferenceFrameLibrary referenceFrameLibrary;
   private String parentReferenceFrameName;
   private int selectedFrameIndex;

   public ImGuiReferenceFrameLibraryCombo(ReferenceFrameLibrary referenceFrameLibrary)
   {
      this.referenceFrameLibrary = referenceFrameLibrary;
   }

   public boolean render()
   {
      selectedFrameIndex = referenceFrameLibrary.findFrameIndexByName(parentReferenceFrameName);

      String[] referenceFrameNamesArray = referenceFrameLibrary.getReferenceFrameNameArray();

      if (ImGui.beginCombo(labels.get("Parent frame"), referenceFrameNamesArray[selectedFrameIndex]))
      {
         for (int i = 0; i < referenceFrameNamesArray.length; i++)
         {
            ReferenceFrame referenceFrame = referenceFrameLibrary.findFrameByName(referenceFrameNamesArray[i]);

            if (referenceFrame != null)
            {
               boolean frameHasNoParentFrame = referenceFrame.equals(ConditionalReferenceFrame.INVALID_FRAME);

               if (frameHasNoParentFrame)
                  ImGui.pushStyleColor(ImGuiCol.Text, Color.RED.toIntBits());

               if (ImGui.selectable(referenceFrameNamesArray[i], selectedFrameIndex == i))
               {
                  selectedFrameIndex = i;
                  setSelectedParentReferenceFrameName(referenceFrameNamesArray[selectedFrameIndex]);
               }

               if (frameHasNoParentFrame)
                  ImGui.popStyleColor();
            }
         }

         ImGui.endCombo();
      }

      return true;
   }

   public void setSelectedParentReferenceFrameName(String parentReferenceFrameName)
   {
      this.parentReferenceFrameName = parentReferenceFrameName;
      selectedFrameIndex = referenceFrameLibrary.findFrameIndexByName(parentReferenceFrameName);
   }

   public String getSelectedParentReferenceFrameName()
   {
      return parentReferenceFrameName;
   }
}
