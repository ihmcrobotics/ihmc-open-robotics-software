package us.ihmc.robotics.math.frames;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoFramePoint extends YoFrameTuple implements FixedFramePoint3DBasics
{
   /**
    * Creates a new yo frame point using the given yo variables and sets its reference frame to
    * {@code referenceFrame}.
    *
    * @param xVariable an existing variable representing the x value of this yo frame point.
    * @param yVariable an existing variable representing the y value of this yo frame point.
    * @param zVariable an existing variable representing the z value of this yo frame point.
    * @param referenceFrame the reference frame for this yo frame point.
    */
   public YoFramePoint(YoDouble xVariable, YoDouble yVariable, YoDouble zVariable, ReferenceFrame referenceFrame)
   {
      super(xVariable, yVariable, zVariable, referenceFrame);
   }

   /**
    * Creates a new yo frame point, initializes its coordinates to zero and its reference frame to
    * {@code referenceFrame}, and registers variables to {@code registry}.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param referenceFrame the reference frame for this yo frame point.
    * @param registry the registry to register child variables to.
    */
   public YoFramePoint(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
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
   public YoFramePoint(String namePrefix, String nameSuffix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      super(namePrefix, nameSuffix, referenceFrame, registry);
   }
}
