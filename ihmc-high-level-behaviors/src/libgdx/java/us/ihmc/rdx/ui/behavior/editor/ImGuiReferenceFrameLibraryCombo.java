package us.ihmc.rdx.ui.behavior.editor;

import imgui.ImGui;
import imgui.type.ImInt;
import us.ihmc.behaviors.sequence.ReferenceFrameLibrary;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

public class ImGuiReferenceFrameLibraryCombo
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImInt referenceFrameIndex = new ImInt();
   private final ReferenceFrameLibrary referenceFrameLibrary;

   public ImGuiReferenceFrameLibraryCombo(ReferenceFrameLibrary referenceFrameLibrary)
   {
      this.referenceFrameLibrary = referenceFrameLibrary;
   }

   public boolean combo()
   {
      return ImGui.combo(labels.get("Reference frame"), referenceFrameIndex, referenceFrameLibrary.getReferenceFrameNames());
   }

   public boolean setSelectedReferenceFrame(String referenceFrameName)
   {
      int frameIndex = referenceFrameLibrary.findFrameIndexByName(referenceFrameName);
      boolean frameFound = frameIndex >= 0;
      if (frameFound)
      {
         referenceFrameIndex.set(frameIndex);
      }
      return frameFound;
   }

   public ReferenceFrame getSelectedReferenceFrame()
   {
      return referenceFrameLibrary.getReferenceFrames().get(referenceFrameIndex.get());
   }
}
