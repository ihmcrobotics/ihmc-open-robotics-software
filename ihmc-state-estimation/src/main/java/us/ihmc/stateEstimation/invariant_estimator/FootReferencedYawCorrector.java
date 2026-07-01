package us.ihmc.stateEstimation.invariant_estimator;

import java.util.function.ToDoubleFunction;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Foot-referenced yaw seeding for the right-invariant contact filter.
 *
 * <p>The heading (yaw about world Z) and absolute horizontal position are unobservable in this filter, so
 * yaw drifts. This corrector limits that drift using the only available yaw reference on flat ground: a
 * planted foot does not rotate, so the world yaw of a stance foot is constant. When a foot touches down we
 * anchor {@code footWorldYaw = pelvisYaw + relativeFootYaw} (the latter from pure leg FK, independent of
 * the filter). While that foot stays in contact, the pelvis yaw that keeps the foot's world yaw at its
 * anchor is {@code refYaw = anchorPelvisYaw + (anchorRelativeFootYaw − relativeFootYaw_now)}. The per-foot
 * yaw errors are averaged weighted by contact probability and fed back as a small <em>mean-only</em>
 * complementary nudge of the filter orientation ({@link RotationMatrix#prependYawRotation}, i.e. a left/
 * world-frame yaw rotation).</p>
 *
 * <p>Deliberately conservative: it corrects only the yaw of the mean and never touches the covariance P
 * (so it makes no false claim of reduced yaw uncertainty). Roll and pitch are observable from gravity and
 * are left untouched. The correction gain is small so a single bad touchdown anchor cannot snap the base.
 * This is a heuristic seed, not an observability fix — verify the {@code prependYawRotation} sign and the
 * anchor bookkeeping against your frame conventions before trusting it on hardware.</p>
 */
public class FootReferencedYawCorrector
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final InvariantEKF ekf;
   private final MovingReferenceFrame pelvisFrame;
   private final SideDependentList<MovingReferenceFrame> soleFrames;
   private final ToDoubleFunction<RobotSide> contactProbability;

   private final double correctionGain;   // fraction of the yaw error applied per tick, in (0, 1]
   private final double highThreshold;     // p above which a foot is (re)anchored at touchdown
   private final double lowThreshold;      // p below which a foot's anchor is released

   private final SideDependentList<YoBoolean> anchored;
   private final SideDependentList<YoDouble> anchorPelvisYaw;
   private final SideDependentList<YoDouble> anchorRelativeFootYaw;
   private final YoDouble yoYawCorrection = new YoDouble("yawSeedCorrection", registry);

   private final RotationMatrix filterRotation = new RotationMatrix();
   private final RigidBodyTransform soleToPelvis = new RigidBodyTransform();

   /** Builds a corrector with conservative defaults (gain 0.02, anchor/release at p = 0.8 / 0.3). */
   public FootReferencedYawCorrector(InvariantEKF ekf,
                                     MovingReferenceFrame pelvisFrame,
                                     SideDependentList<MovingReferenceFrame> soleFrames,
                                     ToDoubleFunction<RobotSide> contactProbability,
                                     YoRegistry parentRegistry)
   {
      this(ekf, pelvisFrame, soleFrames, contactProbability, 0.02, 0.8, 0.3, parentRegistry);
   }

   public FootReferencedYawCorrector(InvariantEKF ekf,
                                     MovingReferenceFrame pelvisFrame,
                                     SideDependentList<MovingReferenceFrame> soleFrames,
                                     ToDoubleFunction<RobotSide> contactProbability,
                                     double correctionGain,
                                     double highThreshold,
                                     double lowThreshold,
                                     YoRegistry parentRegistry)
   {
      this.ekf = ekf;
      this.pelvisFrame = pelvisFrame;
      this.soleFrames = soleFrames;
      this.contactProbability = contactProbability;
      this.correctionGain = correctionGain;
      this.highThreshold = highThreshold;
      this.lowThreshold = lowThreshold;

      anchored = new SideDependentList<>(new YoBoolean("yawAnchoredLeft", registry), new YoBoolean("yawAnchoredRight", registry));
      anchorPelvisYaw = new SideDependentList<>(new YoDouble("yawAnchorPelvisLeft", registry), new YoDouble("yawAnchorPelvisRight", registry));
      anchorRelativeFootYaw = new SideDependentList<>(new YoDouble("yawAnchorRelFootLeft", registry),
                                                      new YoDouble("yawAnchorRelFootRight", registry));

      parentRegistry.addChild(registry);
   }

   /** Clears all anchors; call on (re)initialization. */
   public void reset()
   {
      for (RobotSide side : RobotSide.values)
         anchored.get(side).set(false);
      yoYawCorrection.set(0.0);
   }

   /**
    * Applies one yaw-seeding correction to the filter mean. Assumes the reference frames are current for
    * this tick and the filter measurement update has already run.
    */
   public void correct()
   {
      ekf.getRotation(filterRotation);
      double pelvisYaw = filterRotation.getYaw();

      double weightedDeltaSum = 0.0;
      double weightSum = 0.0;

      for (RobotSide side : RobotSide.values)
      {
         double relativeFootYaw = relativeFootYaw(side);
         double contactP = clamp(contactProbability.applyAsDouble(side));

         boolean isAnchored = anchored.get(side).getBooleanValue();
         if (contactP >= highThreshold && !isAnchored)
         {
            anchorPelvisYaw.get(side).set(pelvisYaw);
            anchorRelativeFootYaw.get(side).set(relativeFootYaw);
            anchored.get(side).set(true);
            isAnchored = true;
         }
         else if (contactP <= lowThreshold)
         {
            anchored.get(side).set(false);
            isAnchored = false;
         }

         if (isAnchored)
         {
            double referenceYaw = anchorPelvisYaw.get(side).getDoubleValue()
                                  + wrapToPi(anchorRelativeFootYaw.get(side).getDoubleValue() - relativeFootYaw);
            weightedDeltaSum += contactP * wrapToPi(referenceYaw - pelvisYaw);
            weightSum += contactP;
         }
      }

      if (weightSum > 1.0e-6)
      {
         double correction = correctionGain * (weightedDeltaSum / weightSum);
         filterRotation.prependYawRotation(correction); // left/world yaw: R ← Rz(correction)·R
         ekf.setRotation(filterRotation);
         yoYawCorrection.set(correction);
      }
      else
      {
         yoYawCorrection.set(0.0);
      }
   }

   /** Yaw of the sole frame expressed in the pelvis frame (pure leg FK, independent of the filter). */
   private double relativeFootYaw(RobotSide side)
   {
      soleFrames.get(side).getTransformToDesiredFrame(soleToPelvis, pelvisFrame);
      return soleToPelvis.getRotation().getYaw();
   }

   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   private static double wrapToPi(double angle)
   {
      return Math.atan2(Math.sin(angle), Math.cos(angle));
   }

   private static double clamp(double value)
   {
      return value < 0.0 ? 0.0 : (value > 1.0 ? 1.0 : value);
   }
}
