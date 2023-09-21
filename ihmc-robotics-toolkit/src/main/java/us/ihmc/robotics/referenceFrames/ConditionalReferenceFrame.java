package us.ihmc.robotics.referenceFrames;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;

import java.util.Objects;
import java.util.function.Supplier;

public class ConditionalReferenceFrame implements Supplier<ReferenceFrame>
{
   private static final ReferenceFrame NULL_FRAME = ReferenceFrameTools.constructARootFrame("NullFrame");

   private final String frameName;
   private String parentFrameName;
   private ModifiableReferenceFrame modifiableReferenceFrame;

   public ConditionalReferenceFrame(String frameName, String parentFrameName)
   {
      this.frameName = frameName;
      this.parentFrameName = parentFrameName;
      modifiableReferenceFrame = new ModifiableReferenceFrame(frameName, NULL_FRAME);
   }

   public String getFrameName()
   {
      return frameName;
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
      ReferenceFrame parentFrame = referenceFrameLibrary.findFrameByName(parentFrameName).get();
      modifiableReferenceFrame.changeParentFrame(Objects.requireNonNullElse(parentFrame, NULL_FRAME));
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
