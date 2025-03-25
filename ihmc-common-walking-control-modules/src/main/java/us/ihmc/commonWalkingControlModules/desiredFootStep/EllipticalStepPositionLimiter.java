package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.robotics.geometry.GroundPlaneEstimator;
import us.ihmc.robotics.robotSide.RobotSide;

public class EllipticalStepPositionLimiter
{
   public final static double NOMINAL_STANCE_WIDTH_DEFAULT = 0.25;
   public final static double MAX_STEP_FORWARD_DEFAULT = 0.75;
   public final static double MIN_STANCE_WIDTH_DEFAULT = 0.075;
   public final static double MAX_STANCE_WIDTH_DEFAULT = 0.4;
   public final static double MIN_DISTANCE_FROM_STANCE_FOOT_DEFAULT = 0.075;

   private final FramePoint2D stanceFootPosition = new FramePoint2D();
   private final FramePoint2D constrainedTouchdownPosition2D = new FramePoint2D();
   private final FramePoint3DBasics constrainedSoleTouchdownPosition = new FramePoint3D();

   public void enforceFootPositionConstraint(FramePoint3DReadOnly desiredSoleTouchdownPosition,
                                                FixedFramePoint3DBasics constrainedSoleTouchdownPositionToPack,
                                                ReferenceFrame desiredConstraintFrame,
                                                ReferenceFrame stanceFootZUpFrame,
                                                RobotSide swingSide)
   {
      enforceFootPositionConstraint(desiredSoleTouchdownPosition,
                                    constrainedSoleTouchdownPositionToPack,
                                    desiredConstraintFrame,
                                    stanceFootZUpFrame,
                                    NOMINAL_STANCE_WIDTH_DEFAULT,
                                    MAX_STEP_FORWARD_DEFAULT,
                                    MIN_STANCE_WIDTH_DEFAULT,
                                    MAX_STANCE_WIDTH_DEFAULT,
                                    MIN_DISTANCE_FROM_STANCE_FOOT_DEFAULT,
                                    swingSide);
   }

   /**
    * @param desiredSoleTouchdownPosition              The unconstrained position. Not Modified.
    * @param constrainedSoleTouchdownPositionToPack    The step position that is constrained.
    * @param desiredConstraintFrame                    The frame in which to apply the constraint.
    */
   public boolean enforceFootPositionConstraint(FramePoint3DReadOnly desiredSoleTouchdownPosition,
                                                FixedFramePoint3DBasics constrainedSoleTouchdownPositionToPack,
                                                ReferenceFrame desiredConstraintFrame,
                                                ReferenceFrame stanceFootZUpFrame,
                                                double nominalStanceWidth,
                                                double maxStepForward,
                                                double minStanceWidth,
                                                double maxStanceWidth,
                                                double minDistanceFromStanceFoot,
                                                RobotSide swingSide)
   {
      return enforceFootPositionConstraint(desiredSoleTouchdownPosition,
                                           constrainedSoleTouchdownPositionToPack,
                                           desiredConstraintFrame,
                                           stanceFootZUpFrame,
                                           null,
                                           nominalStanceWidth,
                                           maxStepForward,
                                           minStanceWidth,
                                           maxStanceWidth,
                                           minDistanceFromStanceFoot,
                                           swingSide);
   }

   /**
    * @param desiredSoleTouchdownPosition              The unconstrained position. Not Modified.
    * @param constrainedSoleTouchdownPositionToPack    The step position that is constrained.
    * @param desiredConstraintFrame                    The frame in which to apply the constraint.
    */
   public boolean enforceFootPositionConstraint(FramePoint3DReadOnly desiredSoleTouchdownPosition,
                                                FixedFramePoint3DBasics constrainedSoleTouchdownPositionToPack,
                                                ReferenceFrame desiredConstraintFrame,
                                                ReferenceFrame stanceFootZUpFrame,
                                                GroundPlaneEstimator groundPlaneEstimator,
                                                double nominalStanceWidth,
                                                double maxStepForward,
                                                double minStanceWidth,
                                                double maxStanceWidth,
                                                double minDistanceFromStanceFoot,
                                                RobotSide swingSide)
   {
      constrainedSoleTouchdownPosition.setIncludingFrame(desiredSoleTouchdownPosition);

      boolean wasLimited = false;
      wasLimited |= enforceMinimumStepWidthInConstraintFrame(constrainedSoleTouchdownPosition, desiredConstraintFrame, minStanceWidth, swingSide);

      wasLimited |= enforceOuterEllipticalBoundInConstraintFrame(constrainedSoleTouchdownPosition,
                                                                 desiredConstraintFrame,
                                                                 nominalStanceWidth,
                                                                 maxStepForward,
                                                                 maxStanceWidth,
                                                                 swingSide);

      wasLimited |= enforceStepDistanceFromStance(constrainedSoleTouchdownPosition, stanceFootZUpFrame, minDistanceFromStanceFoot);

      wasLimited |= enforceStepWidthFromStance(constrainedSoleTouchdownPosition, desiredConstraintFrame, stanceFootZUpFrame, minStanceWidth, swingSide);

      //TODO do i need to do this on line 90?
      constrainedSoleTouchdownPosition.changeFrame(desiredConstraintFrame);

      //TODO limit z position as well
      if (groundPlaneEstimator != null)
      {
         constrainedTouchdownPosition2D.setIncludingFrame(constrainedSoleTouchdownPosition);
//         constrainedSoleTouchdownPosition.setMatchingFrame(groundPlaneEstimator.getGroundPosition(constrainedTouchdownPosition2D));
      }

      constrainedSoleTouchdownPositionToPack.setMatchingFrame(constrainedSoleTouchdownPosition);
      return wasLimited;
   }

   static boolean enforceMinimumStepWidthInConstraintFrame(FramePoint3DBasics desiredStepPositionToConstrain,
                                                                   ReferenceFrame desiredConstraintFrame,
                                                                   double minStanceWidth,
                                                                   RobotSide swingSide)
   {
      desiredStepPositionToConstrain.changeFrame(desiredConstraintFrame);

      // make sure you don't step too narrowly. This is done by cropping the lateral position of the footstep.
      // If the width is negative, we assume that we're allowing a cross over distance relative to the constraint frame. If it's positive, then that's the
      // nominal stance width, as defined by one foot relative to the other. This means we then divide it by half when computing this value.
      double desiredWidth = desiredStepPositionToConstrain.getY();
      double widthBound = minStanceWidth < 0.0 ? minStanceWidth : 0.5 * minStanceWidth;
      if (swingSide == RobotSide.LEFT)
      {
         if (desiredWidth < widthBound)
         {
            desiredStepPositionToConstrain.setY(widthBound);
            return true;
         }
      }
      else
      {
         if (desiredWidth > -widthBound)
         {
            desiredStepPositionToConstrain.setY(-widthBound);
            return true;
         }
      }

      return false;
   }

   /**
    * The constraint frame is assumed to be the center of the pelvis, or the center of mass frame. So the ellipse is centered half the nominal stance width from
    * the  constraint frame.
    * @param desiredStepPositionToConstrain
    * @param constraintFrame
    * @param nominalStanceWidth
    * @param maxStepForward
    * @param maxStanceWidth
    * @param stepSide
    * @return
    */
   static boolean enforceOuterEllipticalBoundInConstraintFrame(FramePoint3DBasics desiredStepPositionToConstrain,
                                                               ReferenceFrame constraintFrame,
                                                               double nominalStanceWidth,
                                                               double maxStepForward,
                                                               double maxStanceWidth,
                                                               RobotSide stepSide)
   {
      ReferenceFrame originalFrame = desiredStepPositionToConstrain.getReferenceFrame();
      desiredStepPositionToConstrain.changeFrame(constraintFrame);

      double ellipseYOrigin = stepSide.negateIfRightSide(0.5 * nominalStanceWidth);
      double ellipseYMax = maxStanceWidth - nominalStanceWidth;

      double ellipseX = desiredStepPositionToConstrain.getX();
      double ellipseY = desiredStepPositionToConstrain.getY() - ellipseYOrigin;

      // check if the point is inside the ellipse
      double r = MathTools.square(ellipseX / maxStepForward) + MathTools.square(ellipseY / ellipseYMax);

      if (r <= 1.0)
      {
         desiredStepPositionToConstrain.changeFrame(originalFrame);
         return false;
      }

      // project the constraint onto the ellipse bound, since it was outside
      double theta = Math.atan2(desiredStepPositionToConstrain.getX(), ellipseYMax);
      double sinTheta = Math.sin(theta);
      double cosTheta = Math.cos(theta);
      double k = maxStepForward * ellipseYMax / (Math.sqrt(MathTools.square(ellipseYMax * sinTheta) + MathTools.square(maxStepForward * cosTheta)));
      double x = Math.abs(k * sinTheta) * Math.signum(ellipseX);
      double y = Math.abs(k * cosTheta) * Math.signum(ellipseY);

      y += ellipseYOrigin;

      desiredStepPositionToConstrain.set(x, y, desiredStepPositionToConstrain.getZ());
      desiredStepPositionToConstrain.changeFrame(originalFrame);

      return true;
   }

   private static boolean enforceStepDistanceFromStance(FramePoint3DBasics desiredTouchdownInStanceFootToPack,
                                                        ReferenceFrame stanceFootZUpFrame,
                                                        double minDistanceFromStanceFoot)
   {
      // make sure you don't step too close to the stance foot. This is done by making sure the desired touchdown position stays outside a circle
      // around the stance foot.
      if (minDistanceFromStanceFoot > 0.0)
      {
         desiredTouchdownInStanceFootToPack.changeFrame(stanceFootZUpFrame);
         double distanceFromStance = desiredTouchdownInStanceFootToPack.distanceXY(EuclidCoreTools.origin2D);
         if (distanceFromStance < minDistanceFromStanceFoot)
         {
            double scaleFactor = minDistanceFromStanceFoot / distanceFromStance;
            desiredTouchdownInStanceFootToPack.scale(scaleFactor, scaleFactor, 1.0);

            return true;
         }
      }

      return false;
   }

   private boolean enforceStepWidthFromStance(FramePoint3DBasics desiredStepPositionToConstrain,
                                              ReferenceFrame desiredConstraintFrame,
                                              ReferenceFrame stanceFootZUpFrame,
                                              double minStanceWidth,
                                              RobotSide swingSide)
   {
      // This makes sure step is no less than a defined distance in y away from the stance foot.
      stanceFootPosition.setToZero(stanceFootZUpFrame);
      stanceFootPosition.changeFrameAndProjectToXYPlane(desiredConstraintFrame);
      desiredStepPositionToConstrain.changeFrame(desiredConstraintFrame);

      if (swingSide == RobotSide.LEFT)
      {
         double innerYBound = stanceFootPosition.getY() + minStanceWidth;
         if (innerYBound > desiredStepPositionToConstrain.getY())
         {
            desiredStepPositionToConstrain.setY(innerYBound);
            return true;
         }
      }
      else
      {
         double innerYBound = stanceFootPosition.getY() - minStanceWidth;
         if (innerYBound < desiredStepPositionToConstrain.getY())
         {
            desiredStepPositionToConstrain.setY(innerYBound);
            return true;
         }
      }

      return false;
   }
}
