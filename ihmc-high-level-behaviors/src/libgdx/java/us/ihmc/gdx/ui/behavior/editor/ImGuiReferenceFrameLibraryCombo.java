package us.ihmc.gdx.ui.behavior.editor;

import imgui.ImGui;
import imgui.type.ImInt;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.log.LogTools;

import java.util.Arrays;
import java.util.List;

public class ImGuiReferenceFrameLibraryCombo
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private List<ReferenceFrame> referenceFrameLibrary;
   private final ImInt referenceFrameIndex = new ImInt();
   private String[] referenceFrameNames;

   public ImGuiReferenceFrameLibraryCombo(List<ReferenceFrame> referenceFrameLibrary)
   {
      this.referenceFrameLibrary = referenceFrameLibrary;
      referenceFrameNames = new String[referenceFrameLibrary.size()];
      for (int i = 0; i < referenceFrameLibrary.size(); i++)
      {
         String fullName = referenceFrameLibrary.get(i).getName();
         referenceFrameNames[i] = fullName.substring(fullName.lastIndexOf(".") + 1);
      }
   }

   public boolean combo()
   {
      return ImGui.combo(labels.get("Reference frame"), referenceFrameIndex, referenceFrameNames);
   }

   public boolean setSelectedReferenceFrame(String referenceFrameName)
   {
      for (int i = 0; i < referenceFrameLibrary.size(); i++)
      {
         ReferenceFrame referenceFrame = referenceFrameLibrary.get(i);
         if (referenceFrameName.equals(referenceFrame.getName()))
         {
            referenceFrameIndex.set(i);
            return true;
         }
      }
      LogTools.error("Frame {} is not present in library! {}", referenceFrameName, Arrays.toString(referenceFrameNames));
      return false;
   }

   public ReferenceFrame getSelectedReferenceFrame()
   {
      return referenceFrameLibrary.get(referenceFrameIndex.get());
   }
}
