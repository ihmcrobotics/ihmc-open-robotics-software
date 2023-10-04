package us.ihmc.robotics.referenceFrames;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;

import javax.annotation.Nullable;
import java.util.*;
import java.util.function.Consumer;

/**
 * A library of reference frames. Useful for putting together a specific collection
 * of frames for a specific purpose, rather than offering the full tree which would
 * be overwhelming. This can get passed around to places that don't have access to
 * robot specific frames as well, so things can be generalized better.
 */
public class ReferenceFrameLibrary
{
   /** These frames are are always present. */
   private final ArrayList<ReferenceFrame> alwaysPresentFrames = new ArrayList<>();
   private final Map<String, ReferenceFrame> nameToAlwaysPresentFrameMap = new HashMap<>();
   /** Lookups allow for a dynamically changing set of frames. */
   private final List<ReferenceFrameDynamicCollection> dynamicCollections = new ArrayList<>();

   public ReferenceFrameLibrary()
   {
      // Here so it's easier to track instances in the IDE
   }

   // TODO: Add robot frames in constructor?
   public void add(ReferenceFrame referenceFrame)
   {
      if (referenceFrame != null)
      {
         alwaysPresentFrames.add(referenceFrame);
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

   public void getAllFrameNames(Consumer<String> frameNameConsumer)
   {
      for (ReferenceFrame frame : alwaysPresentFrames)
      {
         frameNameConsumer.accept(frame.getName());
      }

      for (ReferenceFrameDynamicCollection dynamicCollection : dynamicCollections)
      {
         for (String dynamicFrameName : dynamicCollection.getFrameNameList())
         {
            frameNameConsumer.accept(dynamicFrameName);
         }
      }
   }
}
