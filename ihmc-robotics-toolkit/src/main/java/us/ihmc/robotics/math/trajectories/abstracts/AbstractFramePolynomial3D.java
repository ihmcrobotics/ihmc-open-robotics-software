package us.ihmc.robotics.math.trajectories.abstracts;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.robotics.math.trajectories.interfaces.FramePolynomial3DBasics;
import us.ihmc.robotics.math.trajectories.interfaces.Polynomial3DBasics;
import us.ihmc.robotics.math.trajectories.core.Polynomial3DFrameFactories;
import us.ihmc.robotics.math.trajectories.interfaces.PolynomialBasics;
import us.ihmc.robotics.time.TimeIntervalBasics;

public class AbstractFramePolynomial3D implements FramePolynomial3DBasics
{
   private ReferenceFrame referenceFrame;
   private final Polynomial3DBasics polynomial3D;

   private final FramePoint3DReadOnly framePosition;
   private final FrameVector3DReadOnly frameVelocity;
   private final FrameVector3DReadOnly frameAcceleration;

   public AbstractFramePolynomial3D(Polynomial3DBasics polynomial3D, ReferenceFrame referenceFrame)
   {
      this.polynomial3D = polynomial3D;
      this.referenceFrame = referenceFrame;

      framePosition = Polynomial3DFrameFactories.newLinkedFramePoint3DReadOnly(this, polynomial3D.getPosition());
      frameVelocity = Polynomial3DFrameFactories.newLinkedFrameVector3DReadOnly(this, polynomial3D.getVelocity());
      frameAcceleration = Polynomial3DFrameFactories.newLinkedFrameVector3DReadOnly(this, polynomial3D.getAcceleration());
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
   public void showVisualization()
   {
      polynomial3D.showVisualization();
   }

   @Override
   public void hideVisualization()
   {
      polynomial3D.hideVisualization();
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   @Override
   public PolynomialBasics getAxis(int ordinal)
   {
      return polynomial3D.getAxis(ordinal);
   }

   @Override
   public Tuple3DBasics[] getCoefficients()
   {
      return polynomial3D.getCoefficients();
   }

   @Override
   public TimeIntervalBasics getTimeInterval()
   {
      return polynomial3D.getTimeInterval();
   }
}
