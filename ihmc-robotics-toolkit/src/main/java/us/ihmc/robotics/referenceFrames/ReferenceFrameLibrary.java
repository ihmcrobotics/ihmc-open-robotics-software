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
   /**
    * These frames are always present.
    */
   private final ArrayList<ReferenceFrame> alwaysPresentFrames = new ArrayList<>();
   private final Map<String, ReferenceFrame> nameToAlwaysPresentFrameMap = new HashMap<>();

   public ReferenceFrameLibrary()
   {
      // Here so it's easier to track instances in the IDE
   }

   public void addAll(Collection<ReferenceFrame> referenceFrames)
   {
      alwaysPresentFrames.addAll(referenceFrames);
      referenceFrames.forEach(referenceFrame -> nameToAlwaysPresentFrameMap.put(referenceFrame.getName(), referenceFrame));
   }

   public boolean containsFrame(String referenceFrameName)
   {
      return nameToAlwaysPresentFrameMap.containsKey(referenceFrameName);
   }

   @Nullable
   public ReferenceFrame findFrameByName(String referenceFrameName)
   {
      return nameToAlwaysPresentFrameMap.get(referenceFrameName);
   }

   public void getAllFrameNames(Consumer<String> frameNameConsumer)
   {
      for (ReferenceFrame frame : alwaysPresentFrames)
      {
         frameNameConsumer.accept(frame.getName());
      }
   }
}
