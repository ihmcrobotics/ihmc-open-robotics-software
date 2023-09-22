package us.ihmc.robotics.referenceFrames;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;

/**
 * Represents a ReferenceFrameSupplier which may not have a valid parent ReferenceFrame.
 * In the case that the parent ReferenceFrame is not valid, it will be set to the NULL_FRAME.
 * <p>
 * You must call update() and optionally pass a ReferenceFrameLibrary every tick, or when it makes sense.
 */
public class ConditionalReferenceFrame implements ReferenceFrameSupplier
{
   public static final ReferenceFrame INVALID_FRAME = ReferenceFrameTools.constructARootFrame("InvalidFrame");

   private String parentFrameName;
   private final ModifiableReferenceFrame modifiableReferenceFrame;

   public ConditionalReferenceFrame(String frameName, String parentFrameName)
   {
      this.parentFrameName = parentFrameName;
      modifiableReferenceFrame = new ModifiableReferenceFrame(frameName, INVALID_FRAME);
   }

   public String getParentFrameName()
   {
      return parentFrameName;
   }

   public void setParentFrameName(String parentFrameName)
   {
      this.parentFrameName = parentFrameName;
   }

   private boolean update(ReferenceFrame referenceFrame)
   {
      if (referenceFrame.getName().equals(parentFrameName))
      {
         modifiableReferenceFrame.changeParentFrame(referenceFrame);
         return true;
      }
      else
      {
         for (int i = 0; i < referenceFrame.getNumberOfChildren(); i++)
         {
            if (update(referenceFrame.getChild(i)))
            {
               return true;
            }
         }
      }
      return false;
   }

   /**
    * Update to find the parent frame by name recursively using the root ReferenceFrame.
    */
   public void update()
   {
      ReferenceFrame rootFrame = ReferenceFrame.getWorldFrame();
      boolean found = update(rootFrame);
      if (!found)
         modifiableReferenceFrame.changeParentFrame(INVALID_FRAME);
   }

   /**
    * Update to find the parent frame by name using a ReferenceFrameLibrary - most efficient method.
    *
    * @param referenceFrameLibrary the ReferenceFrameLibrary which may contain the parent frame
    */
   public void update(ReferenceFrameLibrary referenceFrameLibrary)
   {
      ReferenceFrameSupplier parentFrameSupplier = referenceFrameLibrary.findFrameByName(parentFrameName);
      if (parentFrameSupplier != null)
         modifiableReferenceFrame.changeParentFrame(parentFrameSupplier.get());
      else
         modifiableReferenceFrame.changeParentFrame(INVALID_FRAME);
   }

   /**
    * Check if there is a valid parent ReferenceFrame.
    *
    * @return true if there exists a valid parent ReferenceFrame
    */
   public boolean hasParentFrame()
   {
      return !modifiableReferenceFrame.getReferenceFrame().getParent().equals(INVALID_FRAME);
   }

   @Override
   public ReferenceFrame get()
   {
      return modifiableReferenceFrame.getReferenceFrame();
   }
}
