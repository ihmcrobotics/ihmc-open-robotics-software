package us.ihmc.robotics.math.frames;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoFrameVector extends YoFrameTuple implements FixedFrameVector3DBasics
{
   /** Only for some garbage-free operations and reducing number of operations on the YoDoubles. */
   private final FrameVector3D frameVector3D = new FrameVector3D();

   /**
    * Creates a new yo frame vector using the given yo variables and sets its reference frame to
    * {@code referenceFrame}.
    *
    * @param xVariable an existing variable representing the x value of this yo frame vector.
    * @param yVariable an existing variable representing the y value of this yo frame vector.
    * @param zVariable an existing variable representing the z value of this yo frame vector.
    * @param referenceFrame the reference frame for this yo frame vector.
    */
   public YoFrameVector(YoDouble xVariable, YoDouble yVariable, YoDouble zVariable, ReferenceFrame referenceFrame)
   {
      super(xVariable, yVariable, zVariable, referenceFrame);
   }

   /**
    * Creates a new yo frame vector, initializes its coordinates to zero and its reference frame to
    * {@code referenceFrame}, and registers variables to {@code registry}.
    *
    * @param referenceFrame the reference frame for this yo frame vector.
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param registry the registry to register child variables to.
    */
   public YoFrameVector(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      super(namePrefix, "", referenceFrame, registry);
   }

   /**
    * Creates a new yo frame vector using the given yo variables and sets its reference frame to
    * {@code referenceFrame}.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param nameSuffix a string to use as the suffix for child variable names.
    * @param referenceFrame the reference frame for this yo frame vector.
    * @param registry the registry to register child variables to.
    */
   public YoFrameVector(String namePrefix, String nameSuffix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      super(namePrefix, nameSuffix, referenceFrame, registry);
   }

   @Override
   public void setAndMatchFrame(FrameTuple3DReadOnly frameTuple3DReadOnly)
   {
      frameVector3D.setIncludingFrame(frameTuple3DReadOnly);
      frameVector3D.changeFrame(getReferenceFrame());
      set(frameVector3D);
   }
}
