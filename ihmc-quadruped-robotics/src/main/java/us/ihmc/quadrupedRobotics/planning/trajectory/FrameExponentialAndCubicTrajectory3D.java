package us.ihmc.quadrupedRobotics.planning.trajectory;

import us.ihmc.commons.Epsilons;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class FrameExponentialAndCubicTrajectory3D extends ExponentialAndCubicTrajectory3D implements ReferenceFrameHolder
{
   private ReferenceFrame referenceFrame;

   private final FramePoint3D framePosition = new FramePoint3D();
   private final FrameVector3D frameVelocity = new FrameVector3D();
   private final FrameVector3D frameAcceleration = new FrameVector3D();


   public FrameExponentialAndCubicTrajectory3D(ReferenceFrame referenceFrame)
   {
      super();
      this.referenceFrame = referenceFrame;
   }

   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   public FramePoint3DReadOnly getFramePosition()
   {
      framePosition.setIncludingFrame(referenceFrame, getPosition());
      return framePosition;
   }

   public FrameVector3DReadOnly getFrameVelocity()
   {
      frameVelocity.setIncludingFrame(referenceFrame, getVelocity());
      return frameVelocity;
   }

   public Vector3DReadOnly getFrameAcceleration()
   {
      frameAcceleration.setIncludingFrame(referenceFrame, getAcceleration());
      return frameAcceleration;
   }

   public void setFromBounds(double t0, double tFinal, double timeConstant, FramePoint3DReadOnly x0, FrameVector3DReadOnly xd0, FrameVector3DReadOnly vrp0,
                             FramePoint3DReadOnly xFinal, FrameVector3DReadOnly xdFinal, FrameVector3DReadOnly vrpFinal)
   {
      x0.checkReferenceFrameMatch(referenceFrame);
      xd0.checkReferenceFrameMatch(referenceFrame);
      vrp0.checkReferenceFrameMatch(referenceFrame);
      xFinal.checkReferenceFrameMatch(referenceFrame);
      xdFinal.checkReferenceFrameMatch(referenceFrame);
      vrpFinal.checkReferenceFrameMatch(referenceFrame);

      super.setFromBounds(t0, tFinal, timeConstant, x0, xd0, vrp0, xFinal, xdFinal, vrpFinal);
   }

   @Override
   public void checkReferenceFrameMatch(ReferenceFrameHolder referenceFrameHolder) throws ReferenceFrameMismatchException
   {
      getReferenceFrame().checkReferenceFrameMatch(referenceFrameHolder.getReferenceFrame());
   }

   @Override
   public void checkReferenceFrameMatch(ReferenceFrame frame) throws ReferenceFrameMismatchException
   {
      getReferenceFrame().checkReferenceFrameMatch(referenceFrame);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }




}
