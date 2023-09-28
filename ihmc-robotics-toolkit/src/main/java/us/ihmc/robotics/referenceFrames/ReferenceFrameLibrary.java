package us.ihmc.robotics.referenceFrames;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;

import javax.annotation.Nullable;
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
   private final Map<String, ReferenceFrameSupplier> frameNameToSupplierMap = new HashMap<>();
   /** Lookups allow for a dynamically changing set of frames. */
   private final List<ReferenceFrameDynamicCollection> dynamicCollections = new ArrayList<>();
   private transient final SortedSet<String> referenceFrameNameSet = new TreeSet<>();
   private transient String[] referenceFrameNameArray = new String[0];

   public ReferenceFrameLibrary()
   {
      // Here so it's easier to track instances in the IDE
   }

   public void addAll(List<ReferenceFrameSupplier> referenceFrameSuppliers)
   {
      for (ReferenceFrameSupplier referenceFrame : referenceFrameSuppliers)
      {
         add(referenceFrame);
      }
   }

   public void add(ReferenceFrameSupplier referenceFrameSupplier)
   {
      ReferenceFrame referenceFrame = referenceFrameSupplier.get();

      if (referenceFrame != null)
      {
         frameNameToSupplierMap.put(referenceFrame.getName(), referenceFrameSupplier);
      }
   }

   public void addParent(ReferenceFrameSupplier referenceFrameSupplier)
   {
      if (referenceFrameSupplier instanceof ConditionalReferenceFrame conditionalReferenceFrame)
      {
         ReferenceFrame referenceFrame = conditionalReferenceFrame.getModifiableReferenceFrame().getReferenceFrame();
         frameNameToSupplierMap.put(conditionalReferenceFrame.getConditionallyValidParentFrameName(), referenceFrame::getParent);
      }
   }

   public Collection<ReferenceFrameSupplier> getAll()
   {
      return frameNameToSupplierMap.values();
   }

   /**
    * @param dynamicCollection A pair of a frame supplier lookup and frame name enumerator.
    */
   public void addDynamicCollection(ReferenceFrameDynamicCollection dynamicCollection)
   {
      dynamicCollections.add(dynamicCollection);
   }

   public void update()
   {
      // Sort in alphabetical order
      referenceFrameNameSet.clear();
      referenceFrameNameSet.addAll(frameNameToSupplierMap.keySet());
      for (ReferenceFrameDynamicCollection dynamicCollection : dynamicCollections)
         for (String dynamicFrameName : dynamicCollection.getFrameNameList())
            referenceFrameNameSet.add(dynamicFrameName);

      referenceFrameNameArray = referenceFrameNameSet.toArray(referenceFrameNameArray);
   }

   @Nullable
   public ReferenceFrame findFrameByName(String referenceFrameName)
   {
      // Check map first, then dynamic collections
      ReferenceFrameSupplier frameSupplier = frameNameToSupplierMap.get(referenceFrameName);
      boolean frameFound = frameSupplier != null;

      ReferenceFrame referenceFrame = null;
      if (frameFound)
      {
         referenceFrame = frameSupplier.get();
      }
      else
      {
         for (ReferenceFrameDynamicCollection dynamicCollection : dynamicCollections)
         {
            referenceFrame = dynamicCollection.getFrameLookup().apply(referenceFrameName);
            frameFound = referenceFrame != null;
            if (frameFound)
               break;
         }
      }

      return frameFound ? referenceFrame : null;
   }

   public int findFrameIndexByName(String referenceFrameName)
   {
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
         referenceFrameNameArray = referenceFrameNameSet.toArray(new String[0]);
      }

      return referenceFrameNameArray;
   }
}
