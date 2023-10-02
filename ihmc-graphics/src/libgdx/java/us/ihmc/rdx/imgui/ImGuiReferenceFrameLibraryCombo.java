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
   private final String comboName;
   private final ReferenceFrameLibrary referenceFrameLibrary;
   private ConditionalReferenceFrame selectedReferenceFrame = new ConditionalReferenceFrame();
   private int selectedFrameIndex;

   public ImGuiReferenceFrameLibraryCombo(String comboName, ReferenceFrameLibrary referenceFrameLibrary)
   {
      this.comboName = comboName;
      this.referenceFrameLibrary = referenceFrameLibrary;
   }

   public boolean render()
   {
      String[] referenceFrameNamesArray = referenceFrameLibrary.getReferenceFrameNameArray();

      if (ImGui.beginCombo(labels.get(comboName), referenceFrameNamesArray[selectedFrameIndex]))
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
                  selectedFrameIndex = i;

               if (frameHasNoParentFrame)
                  ImGui.popStyleColor();
            }
         }

         ImGui.endCombo();
      }

      return true;
   }

   public void setSelectedReferenceFrame(ConditionalReferenceFrame referenceFrame)
   {
      this.selectedReferenceFrame = referenceFrame;
   }

   public ReferenceFrame getSelectedReferenceFrame()
   {
      return selectedReferenceFrame.get();
   }
}
