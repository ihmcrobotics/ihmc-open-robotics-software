package us.ihmc.robotics.math.frames;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.ReferenceFrameHolder;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class YoMultipleFramesHelper extends ReferenceFrameHolder
{
   private static final int MAX_REGISTERED_FRAMES = 10;
   private final IntegerYoVariable currentFrameIndex;
   private final ArrayList<ReferenceFrame> referenceFrames = new ArrayList<ReferenceFrame>();

   public YoMultipleFramesHelper(String namePrefix, YoVariableRegistry registry, ReferenceFrame... referenceFrames)
   {
      currentFrameIndex = new IntegerYoVariable(namePrefix + "FrameIndex", registry);
      currentFrameIndex.set(0);

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
      return referenceFrames.get(currentFrameIndex.getIntegerValue());
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
      currentFrameIndex.set(findRegisteredReferenceFrame(newCurrentReferenceFrame));
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

      referenceFrames.add(newReferenceFrame);
      if (referenceFrames.size() > MAX_REGISTERED_FRAMES) throw new RuntimeException("Can only register 10 frames. If you really want more, we need to make findRegisteredReferenceFrame() more efficient by using a Hash Map of some sort.");

   }

   /**
    * Check if a reference frame has already been registered
    * @param referenceFrame
    * @return true if the reference frame has already been registered, false otherwise.
    */
   public boolean isReferenceFrameRegistered(ReferenceFrame referenceFrame)
   {
      return referenceFrames.contains(referenceFrame);
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
      referenceFramesToPack.addAll(referenceFrames);
   }

   private int findRegisteredReferenceFrame(ReferenceFrame referenceFrame)
   {
      int newFrameIndex = referenceFrames.indexOf(referenceFrame);

      if (newFrameIndex == -1)
         throw new RuntimeException("The frame: " + referenceFrame.getName() + " has not been registered.");

      return newFrameIndex;
   }
}
