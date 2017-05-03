package us.ihmc.robotics.screwTheory;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public abstract class MovingReferenceFrame extends ReferenceFrame
{
   private static final long serialVersionUID = 8822523097080864423L;

   private boolean isTwistOfFrameUpToDate = false;
   private final Twist twistRelativeToParent;
   private final Twist twistOfFrame = new Twist();

   private final MovingReferenceFrame parentMovingFrame;

   private final boolean isFixedInParent;

   public static MovingReferenceFrame constructFrameFixedInParent(String frameName, ReferenceFrame parentFrame, RigidBodyTransform transformToParent)
   {
      boolean isZUpFrame = parentFrame.isZupFrame() && transformToParent.isRotation2D();
      boolean isFixedInParent = true;
      MovingReferenceFrame newFrame = new MovingReferenceFrame(frameName, parentFrame, transformToParent, isZUpFrame, isFixedInParent)
      {
         private static final long serialVersionUID = 1036812472469332287L;

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
         }

         @Override
         protected void updateTwistRelativeToParent(Twist twistRelativeToParentToPack)
         {
         }
      };

      return newFrame;
   }

   public MovingReferenceFrame(String frameName, ReferenceFrame parentFrame)
   {
      this(frameName, parentFrame, null, false, false);
   }

   public MovingReferenceFrame(String frameName, ReferenceFrame parentFrame, RigidBodyTransform transformToParent)
   {
      this(frameName, parentFrame, transformToParent, false, false);
   }

   public MovingReferenceFrame(String frameName, ReferenceFrame parentFrame, RigidBodyTransform transformToParent, boolean isZUpFrame)
   {
      this(frameName, parentFrame, transformToParent, isZUpFrame, false);
   }

   public MovingReferenceFrame(String frameName, ReferenceFrame parentFrame, RigidBodyTransform transformToParent, boolean isZUpFrame, boolean isFixedInParent)
   {
      super(frameName, parentFrame, transformToParent, parentFrame.isAStationaryFrame() && isFixedInParent, isZUpFrame);

      this.isFixedInParent = isFixedInParent;

      if (parentFrame instanceof MovingReferenceFrame)
      {
         parentMovingFrame = (MovingReferenceFrame) parentFrame;
      }
      else
      {
         parentMovingFrame = null;

         if (!parentFrame.isAStationaryFrame())
            throw unhandledReferenceFrameTypeException(parentFrame);
      }

      if (isFixedInParent)
         twistRelativeToParent = null;
      else
         twistRelativeToParent = new Twist();
   }

   @Override
   public void update()
   {
      super.update();

      if (!isFixedInParent)
      {
         updateTwistRelativeToParent(twistRelativeToParent);
         twistRelativeToParent.checkReferenceFramesMatch(this, parentFrame, this);
      }

      isTwistOfFrameUpToDate = false;
   }

   protected abstract void updateTwistRelativeToParent(Twist twistRelativeToParentToPack);

   private void updateTwistOfFrame()
   {
      if (isTwistOfFrameUpToDateRecursive())
         return;

      if (parentMovingFrame == null)
      {
         if (isFixedInParent)
            twistOfFrame.setToZero(this, parentFrame, this);
         else
            twistOfFrame.set(twistRelativeToParent);
      }
      else
      {
         Twist parentTwist = parentMovingFrame.getTwistOfFrame();
         twistOfFrame.set(parentTwist);
         twistOfFrame.changeFrame(this);

         if (isFixedInParent)
            twistOfFrame.changeBodyFrameNoRelativeTwist(this);
         else
            twistOfFrame.add(twistRelativeToParent);
      }

      isTwistOfFrameUpToDate = true;
   }

   private boolean isTwistOfFrameUpToDateRecursive()
   {
      return isTwistOfFrameUpToDate && (parentMovingFrame == null || parentMovingFrame.isTwistOfFrameUpToDateRecursive());
   }

   public Twist getTwistOfFrame()
   {
      updateTwistOfFrame();
      return twistOfFrame;
   }

   public void getTwistOfFrame(Twist twistToPack)
   {
      twistToPack.set(getTwistOfFrame());
   }

   public void getTwistRelativeToOther(ReferenceFrame base, Twist relativeTwistToPack)
   {
      if (base.isAStationaryFrame())
      {
         getTwistOfFrame(relativeTwistToPack);
      }
      else if (base instanceof MovingReferenceFrame)
      {
         ((MovingReferenceFrame) base).getTwistOfFrame(relativeTwistToPack);
         relativeTwistToPack.changeFrame(this);
         relativeTwistToPack.sub(getTwistOfFrame());
         relativeTwistToPack.invert();
      }
   }

   public MovingReferenceFrame getMovingParent()
   {
      return parentMovingFrame;
   }

   private static ScrewTheoryException unhandledReferenceFrameTypeException(ReferenceFrame referenceFrame)
   {
      return new ScrewTheoryException("The reference frame type: " + referenceFrame.getClass().getSimpleName()
            + " is currently not handled. Reference frame name: " + referenceFrame.getName());
   }
}
