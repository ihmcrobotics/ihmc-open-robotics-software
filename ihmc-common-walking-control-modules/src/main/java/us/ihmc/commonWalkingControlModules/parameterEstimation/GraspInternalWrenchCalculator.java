package us.ihmc.commonWalkingControlModules.parameterEstimation;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

/**
 * Computes the INTERNAL (squeeze) component of a two-nub force-closure grasp and packs it as per-side contact
 * wrenches for {@link InertialParameterManager}. For two point contacts the grasp map has a 1-D null space -- the
 * squeeze along the line joining the nubs -- so the internal force is {@code lambda * n} at the left nub and
 * {@code -lambda * n} at the right, with {@code n = (p_L - p_R) / ||p_L - p_R||}. Feeding only this component to
 * the estimator's hand-contact channel stops the squeeze being mis-attributed to the forearm inertial parameters,
 * while the external gravitational share (which is NOT in the null space) still informs the estimate.
 *
 * <p>The packed wrench is a pure point-contact force (no moment) expressed in each side's nub frame, laid out as
 * {@code [0, 0, 0, fx, fy, fz]} to match the spatial-vector convention (angular rows first, then linear) used by
 * the contact Jacobians it will be dotted against.</p>
 *
 * <p>Two entry points: {@link #computeFromKnownSqueeze} (Stage A -- the squeeze magnitude is prescribed, e.g. the
 * testbed's commanded lambda) and {@link #computeFromForces} (Stage B -- lambda is recovered from the two
 * backed-out endpoint forces as {@code 0.5 * (f_L - f_R) . n}).</p>
 *
 * @author James Foster
 */
public class GraspInternalWrenchCalculator
{
   private static final double MIN_NUB_SEPARATION = 1.0e-6;

   private final SideDependentList<ReferenceFrame> nubFrames;
   private final ReferenceFrame baseFrame;

   private final FramePoint3D leftPositionBase = new FramePoint3D();
   private final FramePoint3D rightPositionBase = new FramePoint3D();
   private final FrameVector3D graspLine = new FrameVector3D();      // unit n, in baseFrame
   private final FrameVector3D leftForceBase = new FrameVector3D();
   private final FrameVector3D rightForceBase = new FrameVector3D();
   private final FrameVector3D internalForce = new FrameVector3D();  // scratch for packing

   private double squeeze;    // last computed squeeze magnitude lambda (N), for logging/validation
   private double distance;   // last nub separation (m)
   private boolean valid;

   public GraspInternalWrenchCalculator(SideDependentList<ReferenceFrame> nubFrames, ReferenceFrame baseFrame)
   {
      this.nubFrames = nubFrames;
      this.baseFrame = baseFrame;
   }

   /**
    * Stage A: the squeeze magnitude is known (prescribed). Nub positions may be in any frame; they are converted
    * to the base frame internally.
    */
   public void computeFromKnownSqueeze(FramePoint3DReadOnly leftNub, FramePoint3DReadOnly rightNub, double knownSqueeze,
                                       SideDependentList<DMatrixRMaj> handWrenchesToPack)
   {
      if (!updateGraspLine(leftNub, rightNub))
      {
         zero(handWrenchesToPack);
         return;
      }
      squeeze = knownSqueeze;
      pack(handWrenchesToPack);
   }

   /**
    * Stage B: the squeeze magnitude is recovered from the two backed-out endpoint forces as
    * {@code lambda = 0.5 * (f_L - f_R) . n}. Positions and forces may be in any frame.
    */
   public void computeFromForces(FramePoint3DReadOnly leftNub, FramePoint3DReadOnly rightNub,
                                 FrameVector3DReadOnly leftForce, FrameVector3DReadOnly rightForce,
                                 SideDependentList<DMatrixRMaj> handWrenchesToPack)
   {
      if (!updateGraspLine(leftNub, rightNub))
      {
         zero(handWrenchesToPack);
         return;
      }
      leftForceBase.setIncludingFrame(leftForce);
      leftForceBase.changeFrame(baseFrame);
      rightForceBase.setIncludingFrame(rightForce);
      rightForceBase.changeFrame(baseFrame);
      leftForceBase.sub(rightForceBase);           // (f_L - f_R) in baseFrame
      squeeze = 0.5 * leftForceBase.dot(graspLine);
      pack(handWrenchesToPack);
   }

   /** Last computed squeeze magnitude lambda (N). Zero if the grasp line was degenerate. */
   public double getSqueeze()
   {
      return valid ? squeeze : 0.0;
   }

   /** Last nub separation ||p_L - p_R|| (m). */
   public double getDistance()
   {
      return distance;
   }

   private boolean updateGraspLine(FramePoint3DReadOnly leftNub, FramePoint3DReadOnly rightNub)
   {
      leftPositionBase.setIncludingFrame(leftNub);
      leftPositionBase.changeFrame(baseFrame);
      rightPositionBase.setIncludingFrame(rightNub);
      rightPositionBase.changeFrame(baseFrame);
      graspLine.setIncludingFrame(leftPositionBase);   // as a base-frame vector = p_L
      graspLine.sub(rightPositionBase);                // n = p_L - p_R (both in baseFrame)
      distance = graspLine.length();
      valid = distance >= MIN_NUB_SEPARATION;
      if (!valid)
      {
         squeeze = 0.0;
         return false;
      }
      graspLine.scale(1.0 / distance);
      return true;
   }

   private void pack(SideDependentList<DMatrixRMaj> handWrenchesToPack)
   {
      for (RobotSide side : RobotSide.values)
      {
         // +lambda*n at the LEFT nub, -lambda*n at the RIGHT nub (equal-and-opposite squeeze along the line).
         internalForce.setIncludingFrame(graspLine);
         internalForce.scale(side == RobotSide.LEFT ? squeeze : -squeeze);
         internalForce.changeFrame(nubFrames.get(side));
         DMatrixRMaj wrench = handWrenchesToPack.get(side);
         wrench.zero();                       // pure point-contact force: no moment
         wrench.set(3, 0, internalForce.getX());
         wrench.set(4, 0, internalForce.getY());
         wrench.set(5, 0, internalForce.getZ());
      }
   }

   private static void zero(SideDependentList<DMatrixRMaj> handWrenchesToPack)
   {
      for (RobotSide side : RobotSide.values)
         handWrenchesToPack.get(side).zero();
   }
}
