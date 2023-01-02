package us.ihmc.behaviors.sequence;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ReferenceFrameLibrary
{
   private final ArrayList<ReferenceFrame> referenceFrames = new ArrayList<>();
   private String[] referenceFrameNames;

   public void add(ReferenceFrame referenceFrame)
   {
      referenceFrames.add(referenceFrame);
   }

   public void build()
   {
      referenceFrameNames = new String[referenceFrames.size()];
      for (int i = 0; i < referenceFrames.size(); i++)
      {
         String fullName = referenceFrames.get(i).getName();
         referenceFrameNames[i] = fullName.substring(fullName.lastIndexOf(".") + 1);
      }
   }

   public ReferenceFrame findFrameByName(String referenceFrameName)
   {
      int frameIndex = findFrameIndexByName(referenceFrameName);
      boolean frameFound = frameIndex >= 0;
      if (frameFound)
      {
         return referenceFrames.get(frameIndex);
      }
      LogTools.warn("Using world frame.");
      return ReferenceFrame.getWorldFrame();
   }

   public int findFrameIndexByName(String referenceFrameName)
   {
      for (int i = 0; i < referenceFrames.size(); i++)
      {
         ReferenceFrame referenceFrame = referenceFrames.get(i);
         if (referenceFrameName.equals(referenceFrame.getName()))
         {
            return i;
         }
      }
      LogTools.error("Frame {} is not present in library! {}", referenceFrameName, Arrays.toString(referenceFrameNames));
      return -1;
   }

   public List<ReferenceFrame> getReferenceFrames()
   {
      return referenceFrames;
   }

   public String[] getReferenceFrameNames()
   {
      return referenceFrameNames;
   }
}
