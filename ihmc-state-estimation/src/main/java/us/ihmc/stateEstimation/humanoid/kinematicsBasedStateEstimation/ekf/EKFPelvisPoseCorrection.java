package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.ekf;

import static us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.newYoGraphicCoordinateSystem3D;

import ihmc_common_msgs.StampedPosePacket;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.PelvisPoseHistoryCorrectionInterface;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.function.Consumer;
import java.util.function.LongSupplier;

/**
 * Pelvis pose correction that fuses the estimator's pelvis pose with external pose measurements in a {@link FusedPoseEstimator},
 * an alternative to {@link us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.NewPelvisPoseHistoryCorrection}.
 * <p>
 * Each external pose's timestamp must be the wall time of its exposure, and its frame ID names the world it is expressed in:
 * the first pose in a frame aligns that world to the current estimate, so the poses need not be in the robot's world frame.
 * </p>
 * <p>
 * The yo-variable {@code applyEKFPelvisPoseCorrection} chooses, at runtime, whether the corrected pose replaces the
 * estimator's pelvis pose, so that the rest of the estimator and the controller use it, or is only published through
 * {@link #setCorrectedPosePublisher(Consumer)}. It is published either way. The estimator recomputes its pelvis pose from its
 * own state every tick, before any correction, so applying this correction never feeds back into the motion the filter
 * predicts from.
 * </p>
 */
public class EKFPelvisPoseCorrection implements PelvisPoseHistoryCorrectionInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final FloatingJointBasics rootJoint;
   private final LongSupplier monotonicClock;
   private final FusedPoseEstimator fusedPoseEstimator;
   private final RigidBodyTransform estimatorPelvisPose = new RigidBodyTransform();
   private final RigidBodyTransform correctedPelvisPose = new RigidBodyTransform();
   private final YoBoolean applyCorrection = new YoBoolean("applyEKFPelvisPoseCorrection", registry);
   private final YoDouble publishPeriod = new YoDouble("ekfCorrectedPelvisPosePublishPeriod", registry);
   private final YoFramePoseUsingYawPitchRoll yoCorrectedPelvisPose = new YoFramePoseUsingYawPitchRoll("ekfCorrectedPelvisPose", worldFrame, registry);
   private final StampedPosePacket correctedPosePacket = new StampedPosePacket();
   private PelvisPoseCorrectionCommunicatorInterface poseMeasurementSubscriber;
   private Consumer<StampedPosePacket> correctedPosePublisher;
   private long lastPublishMonotonicTime;
   private boolean hasPublished;

   /**
    * @param rootJoint      the estimator's root joint, which holds the pelvis pose to correct.
    * @param monotonicClock the estimator's monotonic time, in ns; the wall time comes from {@link #doControl(long)}.
    */
   public EKFPelvisPoseCorrection(FloatingJointBasics rootJoint,
                                  LongSupplier monotonicClock,
                                  FusedPoseEstimatorParameters parameters,
                                  YoRegistry parentRegistry)
   {
      this.rootJoint = rootJoint;
      this.monotonicClock = monotonicClock;
      fusedPoseEstimator = new FusedPoseEstimator(parameters);
      applyCorrection.set(true);
      publishPeriod.set(0.02);
      correctedPosePacket.getFrameId().append(worldFrame.getName());
      correctedPosePacket.setConfidenceFactor(1.0);
      parentRegistry.addChild(registry);
   }

   /**
    * Receives the corrected pelvis pose at most once per {@code ekfCorrectedPelvisPosePublishPeriod}, on the estimator thread.
    * The packet is reused, so copy it before handing it to another thread.
    */
   public void setCorrectedPosePublisher(Consumer<StampedPosePacket> correctedPosePublisher)
   {
      this.correctedPosePublisher = correctedPosePublisher;
   }

   @Override
   public void setExternalPelvisCorrectorSubscriber(PelvisPoseCorrectionCommunicatorInterface externalPelvisPoseSubscriber)
   {
      poseMeasurementSubscriber = externalPelvisPoseSubscriber;
   }

   /** @param timestamp the estimator's wall time, in ns. */
   @Override
   public void doControl(long timestamp)
   {
      if (poseMeasurementSubscriber == null)
         return;

      long monotonicTime = monotonicClock.getAsLong();
      rootJoint.getJointConfiguration(estimatorPelvisPose);
      StampedPosePacket poseMeasurement = poseMeasurementSubscriber.hasNewPose() ? poseMeasurementSubscriber.getNewExternalPose() : null;
      fusedPoseEstimator.update(timestamp, monotonicTime, estimatorPelvisPose, poseMeasurement, correctedPelvisPose);
      yoCorrectedPelvisPose.set(correctedPelvisPose);

      if (applyCorrection.getValue())
      {
         rootJoint.setJointConfiguration(correctedPelvisPose);
         rootJoint.updateFrame();
      }

      publishCorrectedPose(timestamp, monotonicTime);
   }

   private void publishCorrectedPose(long wallTime, long monotonicTime)
   {
      if (correctedPosePublisher == null)
         return;
      if (hasPublished && monotonicTime - lastPublishMonotonicTime < (long) (publishPeriod.getValue() * 1.0e9))
         return;
      hasPublished = true;
      lastPublishMonotonicTime = monotonicTime;
      correctedPosePacket.setSequenceId(correctedPosePacket.getSequenceId() + 1);
      correctedPosePacket.setTimestamp(wallTime);
      correctedPosePacket.getPose().set(correctedPelvisPose);
      correctedPosePublisher.accept(correctedPosePacket);
   }

   /** Restarts the filter, for example after the estimator is reinitialized. */
   public void reset()
   {
      fusedPoseEstimator.reset();
   }

   public String getStatus()
   {
      return fusedPoseEstimator.getStatus();
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(newYoGraphicCoordinateSystem3D("ekfCorrectedPelvisPoseGraphic", yoCorrectedPelvisPose, 0.1, ColorDefinitions.Orange()));
      return group;
   }
}
