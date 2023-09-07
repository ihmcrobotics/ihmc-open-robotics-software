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
   private final ArrayList<ReferenceFrameSupplier> referenceFrameSuppliers = new ArrayList<>();
   private final HashMap<String, ReferenceFrameSupplier> frameNameToSupplierMap = new HashMap<>();
   private String[] referenceFrameNames;

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
         referenceFrameSuppliers.add(referenceFrame);
      }
   }

   public void build()
   {
      referenceFrameNames = new String[referenceFrameSuppliers.size()];
      for (int i = 0; i < referenceFrameSuppliers.size(); i++)
      {
         String fullName = referenceFrameSuppliers.get(i).get().getName();
         referenceFrameNames[i] = fullName.substring(fullName.lastIndexOf(".") + 1);
      }
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
      for (int i = 0; i < referenceFrameNames.length; i++)
      {
         if (referenceFrameName.equals(referenceFrameNames[i]))
         {
            return i;
         }
      }
      LogTools.error("Frame {} is not present in library! {}", referenceFrameName, Arrays.toString(referenceFrameNames));
      return -1;
   }

   public List<ReferenceFrameSupplier> getReferenceFrameSuppliers()
   {
      return referenceFrameSuppliers;
   }

   public String[] getReferenceFrameNames()
   {
      return referenceFrameNames;
   }
}
