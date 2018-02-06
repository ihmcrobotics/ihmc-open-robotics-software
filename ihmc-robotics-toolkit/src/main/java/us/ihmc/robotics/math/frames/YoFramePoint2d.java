package us.ihmc.robotics.math.frames;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoFramePoint2d extends YoFrameTuple2d implements FixedFramePoint2DBasics
{
   /** Only for some garbage-free operations and reducing number of operations on the YoDoubles. */
   private final FramePoint2D framePoint2D = new FramePoint2D();

   /**
    * Creates a new yo frame point using the given yo variables and sets its reference frame to
    * {@code referenceFrame}.
    *
    * @param referenceFrame the reference frame for this yo frame point.
    * @param xVariable an existing variable representing the x value of this yo frame point.
    * @param yVariable an existing variable representing the y value of this yo frame point.
    */
   public YoFramePoint2d(YoDouble xVariable, YoDouble yVariable, ReferenceFrame referenceFrame)
   {
      super(xVariable, yVariable, referenceFrame);
   }

   /**
    * Creates a new yo frame point, initializes its coordinates to zero and its reference frame to
    * {@code referenceFrame}, and registers variables to {@code registry}.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param referenceFrame the reference frame for this yo frame point.
    * @param registry the registry to register child variables to.
    */
   public YoFramePoint2d(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      super(namePrefix, "", referenceFrame, registry);
   }

   /**
    * Creates a new yo frame point, initializes its coordinates to zero and its reference frame to
    * {@code referenceFrame}, and registers variables to {@code registry}.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param nameSuffix a string to use as the suffix for child variable names.
    * @param referenceFrame the reference frame for this yo frame point.
    * @param registry the registry to register child variables to.
    */
   public YoFramePoint2d(String namePrefix, String nameSuffix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      super(namePrefix, nameSuffix, referenceFrame, registry);
   }

   @Override
   public void setAndMatchFrame(FrameTuple2DReadOnly frameTuple2DReadOnly)
   {
      framePoint2D.setIncludingFrame(frameTuple2DReadOnly);
      framePoint2D.changeFrame(getReferenceFrame());
      set(framePoint2D);
   }
}
