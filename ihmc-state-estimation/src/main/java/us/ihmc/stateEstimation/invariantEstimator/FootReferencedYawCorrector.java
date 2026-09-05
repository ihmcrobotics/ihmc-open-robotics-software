package us.ihmc.stateEstimation.invariantEstimator;

import java.util.function.ToDoubleFunction;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.geometry.AngleTools;
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

   // Unit direction vectors for the AngleTools angle differences. Fields rather than locals so the
   // correction stays allocation-free in the control loop.
   private final Vector2D currentFootDirection = new Vector2D();
   private final Vector2D anchorFootDirection = new Vector2D();
   private final Vector2D pelvisDirection = new Vector2D();
   private final Vector2D referenceDirection = new Vector2D();

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
      pelvisDirection.set(Math.cos(pelvisYaw), Math.sin(pelvisYaw));

      double weightedDeltaSum = 0.0;
      double weightSum = 0.0;

      for (RobotSide side : RobotSide.values)
      {
         double relativeFootYaw = relativeFootYaw(side);
         double contactP = MathTools.clamp(contactProbability.applyAsDouble(side), 0.0, 1.0);

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
            double anchorFootYaw = anchorRelativeFootYaw.get(side).getDoubleValue();
            currentFootDirection.set(Math.cos(relativeFootYaw), Math.sin(relativeFootYaw));
            anchorFootDirection.set(Math.cos(anchorFootYaw), Math.sin(anchorFootYaw));

            double referenceYaw = anchorPelvisYaw.get(side).getDoubleValue()
                                  + signedAngleFromTo(currentFootDirection, anchorFootDirection);

            referenceDirection.set(Math.cos(referenceYaw), Math.sin(referenceYaw));
            weightedDeltaSum += contactP * signedAngleFromTo(pelvisDirection, referenceDirection);
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

   /**
    * Signed angle in (-pi, pi] from {@code startDirection} to {@code endDirection}, i.e. the wrapped
    * difference {@code endYaw - startYaw}.
    *
    * <p>Thin guard over {@link AngleTools#angleMinusPiToPi}, which forms its magnitude as an unclamped
    * {@code Math.acos(dot / normStart / normEnd)}. For (anti)parallel inputs round-off pushes that cosine
    * just outside [-1, 1] and {@code acos} returns NaN — measured at 21% of angles for two unit vectors
    * built from the cosine/sine of the <em>same</em> angle. That is not a corner case here: on the tick a
    * foot is anchored, {@code anchorRelativeFootYaw} is assigned {@code relativeFootYaw}, so the two
    * direction vectors are identical by construction and a NaN would propagate through
    * {@code prependYawRotation} into the filter mean. The limiting value is exact in that case, so
    * recover it directly: parallel gives 0, antiparallel gives pi.</p>
    */
   static double signedAngleFromTo(Vector2DReadOnly startDirection, Vector2DReadOnly endDirection)
   {
      double angle = AngleTools.angleMinusPiToPi(startDirection, endDirection);
      if (Double.isNaN(angle))
         return startDirection.dot(endDirection) >= 0.0 ? 0.0 : Math.PI;
      return angle;
   }

}
