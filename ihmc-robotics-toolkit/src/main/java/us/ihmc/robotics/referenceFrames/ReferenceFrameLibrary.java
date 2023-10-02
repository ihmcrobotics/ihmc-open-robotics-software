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
   /** These frames are are always present. */
   private final Map<String, ReferenceFrame> nameToAlwaysPresentFrameMap = new HashMap<>();
   /** Lookups allow for a dynamically changing set of frames. */
   private final List<ReferenceFrameDynamicCollection> dynamicCollections = new ArrayList<>();
   private transient final SortedSet<String> referenceFrameNameSet = new TreeSet<>();
   private transient String[] referenceFrameNameArray = new String[0];

   public ReferenceFrameLibrary()
   {
      // Here so it's easier to track instances in the IDE
   }

   // TODO: Add robot frames in constructor?
   public void add(ReferenceFrame referenceFrame)
   {
      if (referenceFrame != null)
      {
         nameToAlwaysPresentFrameMap.put(referenceFrame.getName(), referenceFrame);
      }
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
      referenceFrameNameSet.addAll(nameToAlwaysPresentFrameMap.keySet());
      for (ReferenceFrameDynamicCollection dynamicCollection : dynamicCollections)
         for (String dynamicFrameName : dynamicCollection.getFrameNameList())
            referenceFrameNameSet.add(dynamicFrameName);

      getReferenceFrameNameArray();
   }

   @Nullable
   public ReferenceFrame findFrameByName(String referenceFrameName)
   {
      // Check map first, then dynamic collections
      ReferenceFrame referenceFrame = nameToAlwaysPresentFrameMap.get(referenceFrameName);
      boolean frameFound = referenceFrame != null;

      if (!frameFound)
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
      if (referenceFrameNameArray == null || referenceFrameNameArray.length != nameToAlwaysPresentFrameMap.size())
      {
         // Sort in alphabetical order
         SortedSet<String> referenceFrameNameSet = new TreeSet<>(nameToAlwaysPresentFrameMap.keySet());
         referenceFrameNameArray = referenceFrameNameSet.toArray(new String[referenceFrameNameSet.size()]);
      }

      return referenceFrameNameArray;
   }
}
