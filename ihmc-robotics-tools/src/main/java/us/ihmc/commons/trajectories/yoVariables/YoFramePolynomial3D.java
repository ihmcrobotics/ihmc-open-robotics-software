package us.ihmc.commons.trajectories.yoVariables;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameMissingFactories;
import us.ihmc.commons.trajectories.interfaces.FramePolynomial3DBasics;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * {@code YoFramePolynomial3D} provides a frame based wrapper around {@link YoPolynomial3D}, making the polynomial expressible in a specific frame.
 * <p>
 * This frame can be set at construction, or changed without modifying the underlying data by calling {@link #setReferenceFrame(ReferenceFrame)}, as well as
 * changed and changing the coefficients to matching the changed frame through {@link #changeFrame(ReferenceFrame)}.
 * </p>
 * <p>
 * The output is given in the form of {@link FramePoint3DReadOnly}, {@link FrameVector3DReadOnly}, or
 * {@link FrameVector3DReadOnly}.
 * </p>
 *
 * @author Robert Griffin
 */
public class YoFramePolynomial3D extends YoPolynomial3D implements FramePolynomial3DBasics
{
   private ReferenceFrame referenceFrame;

   private final FramePoint3DReadOnly framePosition;
   private final FrameVector3DReadOnly frameVelocity;
   private final FrameVector3DReadOnly frameAcceleration;

   public YoFramePolynomial3D(String name, int maximumNumberOfCoefficients, ReferenceFrame referenceFrame, YoRegistry registry)
   {
      super(name, maximumNumberOfCoefficients, registry);

      this.referenceFrame = referenceFrame;

      framePosition = EuclidFrameMissingFactories.newLinkedFramePoint3DReadOnly(this, super.getPosition());
      frameVelocity = EuclidFrameMissingFactories.newLinkedFrameVector3DReadOnly(this, super.getVelocity());
      frameAcceleration = EuclidFrameMissingFactories.newLinkedFrameVector3DReadOnly(this, super.getAcceleration());
   }

   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   @Override
   public FramePoint3DReadOnly getPosition()
   {
      return framePosition;
   }

   @Override
   public FrameVector3DReadOnly getVelocity()
   {
      return frameVelocity;
   }

   @Override
   public FrameVector3DReadOnly getAcceleration()
   {
      return frameAcceleration;
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }
}
