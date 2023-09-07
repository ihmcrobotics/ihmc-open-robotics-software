package us.ihmc.robotics.referenceFrames;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;

import java.util.*;

/**
 * A library of reference frames. Useful for putting together a specific collection
 * of frames for a specific purpose, rather than offering the full tree which would
 * be overwhelming. This can get passed around to places that don't have access to
 * robot specific frames as well, so things can be generalized better.
 */
public class ReferenceFrameLibrary
{
   /** Reference frames are have immutable parents, so we must use Suppliers. */
   private final HashMap<String, ReferenceFrameSupplier> frameNameToSupplierMap = new HashMap<>();
   private transient String[] referenceFrameNameArray;

   public void addAll(List<ReferenceFrameSupplier> referenceFrameSuppliers)
   {
      for (ReferenceFrameSupplier referenceFrame : referenceFrameSuppliers)
      {
         add(referenceFrame);
      }
   }

   public void add(ReferenceFrameSupplier referenceFrame)
   {
      if (!frameNameToSupplierMap.containsKey(referenceFrame.get().getName()))
      {
         frameNameToSupplierMap.put(referenceFrame.get().getName(), referenceFrame);
      }
   }

   public ReferenceFrameSupplier findFrameByIndex(int referenceFrameIndex)
   {
      return findFrameByName(getReferenceFrameNameArray()[referenceFrameIndex]);
   }

   public ReferenceFrameSupplier findFrameByName(String referenceFrameName)
   {
      ReferenceFrameSupplier frameSupplier = frameNameToSupplierMap.get(referenceFrameName);
      boolean frameFound = frameSupplier != null;
      if (!frameFound)
         LogTools.error("Frame not found: {}. Using world frame.", referenceFrameName);
      return frameFound ? frameSupplier : ReferenceFrame::getWorldFrame;
   }

   public int findFrameIndexByName(String referenceFrameName)
   {
      String[] referenceFrameNameArray = getReferenceFrameNameArray();
      for (int i = 0; i < referenceFrameNameArray.length; i++)
      {
         if (referenceFrameName.equals(referenceFrameNameArray[i]))
         {
            return i;
         }
      }
      LogTools.error("Frame {} is not present in library! {}", referenceFrameName, Arrays.toString(referenceFrameNameArray));
      return -1;
   }

   public String[] getReferenceFrameNameArray()
   {
      if (referenceFrameNameArray == null || referenceFrameNameArray.length != frameNameToSupplierMap.size())
      {
         // Sort in alphabetical order
         SortedSet<String> referenceFrameNameSet = new TreeSet<>(frameNameToSupplierMap.keySet());
         referenceFrameNameArray = referenceFrameNameSet.toArray(new String[referenceFrameNameSet.size()]);
      }

      return referenceFrameNameArray;
   }
}
