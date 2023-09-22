package us.ihmc.robotics.referenceFrames;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;

import java.util.function.Supplier;

/**
 * Represents a ReferenceFrameSupplier which may not have a valid parent ReferenceFrame.
 * In the case that the parent ReferenceFrame is not valid, it will be set to the NULL_FRAME.
 *
 * You must call update() and pass a ReferenceFrameLibrary every tick, or when it makes sense.
 */
public class ConditionalReferenceFrame implements Supplier<ReferenceFrame>
{
   public static final ReferenceFrame NULL_FRAME = ReferenceFrameTools.constructARootFrame("NullFrame");

   private String parentFrameName;
   private final ModifiableReferenceFrame modifiableReferenceFrame;

   public ConditionalReferenceFrame(String frameName, String parentFrameName)
   {
      this.parentFrameName = parentFrameName;
      modifiableReferenceFrame = new ModifiableReferenceFrame(frameName, NULL_FRAME);
   }

   public String getParentFrameName()
   {
      return parentFrameName;
   }

   public void setParentFrameName(String parentFrameName)
   {
      this.parentFrameName = parentFrameName;
   }

   public void update(ReferenceFrameLibrary referenceFrameLibrary)
   {
      ReferenceFrameSupplier parentFrameSupplier = referenceFrameLibrary.findFrameByName(parentFrameName);
      if (parentFrameSupplier != null)
         modifiableReferenceFrame.changeParentFrame(parentFrameSupplier.get());
      else
         modifiableReferenceFrame.changeParentFrame(NULL_FRAME);
   }

   public boolean hasParentFrame()
   {
      return !modifiableReferenceFrame.getReferenceFrame().getParent().equals(NULL_FRAME);
   }

   @Override
   public ReferenceFrame get()
   {
      return modifiableReferenceFrame.getReferenceFrame();
   }
}
