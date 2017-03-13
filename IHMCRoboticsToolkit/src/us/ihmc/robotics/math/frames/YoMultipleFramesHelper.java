package us.ihmc.robotics.math.frames;

import java.util.List;

import gnu.trove.map.hash.TLongObjectHashMap;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.LongYoVariable;
import us.ihmc.robotics.geometry.AbstractReferenceFrameHolder;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class YoMultipleFramesHelper extends AbstractReferenceFrameHolder
{
   private final LongYoVariable currentFrameId;
   private final TLongObjectHashMap<ReferenceFrame> referenceFrames = new TLongObjectHashMap<ReferenceFrame>();

   public YoMultipleFramesHelper(String namePrefix, YoVariableRegistry registry, ReferenceFrame... referenceFrames)
   {
      currentFrameId = new LongYoVariable(namePrefix + "FrameId", registry);
      currentFrameId.set(referenceFrames[0].getNameBasedHashCode());

      if (referenceFrames == null || referenceFrames.length == 0)
         throw new RuntimeException("Need to provide at least one ReferenceFrame.");

      for (ReferenceFrame referenceFrame : referenceFrames)
      {
         this.registerReferenceFrame(referenceFrame);
      }
   }
   
   /**
    * Get the current reference frame
    * @return ReferenceFrame
    */
   public ReferenceFrame getCurrentReferenceFrame()
   {
      return referenceFrames.get(currentFrameId.getLongValue());
   }

   /**
    * Same as getCurrentReferenceFrame(), necessary to extend ReferenceFrameHolder.
    */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return getCurrentReferenceFrame();
   }

   /**
    * Change the current reference frame for another one that has already been registered.
    * @param newCurrentReferenceFrame
    * @return ReferenceFrame the previous current reference frame
    */
   public ReferenceFrame switchCurrentReferenceFrame(ReferenceFrame newCurrentReferenceFrame)
   {
      ReferenceFrame previousReferenceFrame = getCurrentReferenceFrame();
      currentFrameId.set(newCurrentReferenceFrame.getNameBasedHashCode());
      return previousReferenceFrame;
   }

   /**
    * Register a new reference frame.
    * @param newReferenceFrame
    * @throws RuntimeException if newReferenceFrame has already been registered
    */
   public void registerReferenceFrame(ReferenceFrame newReferenceFrame)
   {
      if (newReferenceFrame == null) throw new RuntimeException("The Reference Frames cannot be null.");
      
      if (isReferenceFrameRegistered(newReferenceFrame))
      {
         return;
      }

      referenceFrames.put(newReferenceFrame.getNameBasedHashCode(), newReferenceFrame);
   }

   /**
    * Check if a reference frame has already been registered
    * @param referenceFrame
    * @return true if the reference frame has already been registered, false otherwise.
    */
   public boolean isReferenceFrameRegistered(ReferenceFrame referenceFrame)
   {
      return referenceFrames.contains(referenceFrame.getNameBasedHashCode());
   }

   /**
    * Get the number of reference frames that have already been registered.
    * @return int
    */
   public int getNumberOfReferenceFramesRegistered()
   {
      return referenceFrames.size();
   }

   /**
    * Pack the reference frames that have already been registered into the list.
    * Careful, this method does not clear the list.
    * @param referenceFramesToPack the list in which the reference frames are packed.
    */
   public void getRegisteredReferenceFrames(List<ReferenceFrame> referenceFramesToPack)
   {
      referenceFramesToPack.addAll(referenceFrames.valueCollection());
   }
}
