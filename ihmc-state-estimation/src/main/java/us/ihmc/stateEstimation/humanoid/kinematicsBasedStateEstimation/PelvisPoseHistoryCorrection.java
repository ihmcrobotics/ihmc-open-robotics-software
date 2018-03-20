package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.StampedPosePacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.PelvisPoseErrorPacket;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.humanoidRobotics.communication.subscribers.TimeStampedTransformBuffer;
import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;
import us.ihmc.robotics.math.YoReferencePose;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;

/**
 * Here ICP stands for Iterative Closest Point.
 *
 */

public class PelvisPoseHistoryCorrection implements PelvisPoseHistoryCorrectionInterface
{
   private static final boolean USE_ROTATION_CORRECTION = false;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final TimeStampedTransformBuffer stateEstimatorPelvisPoseBuffer;
   private PelvisPoseCorrectionCommunicatorInterface pelvisPoseCorrectionCommunicator;
   private final FloatingInverseDynamicsJoint rootJoint;
   private final ReferenceFrame pelvisReferenceFrame;
   private final YoVariableRegistry registry;
   private static final double DEFAULT_BREAK_FREQUENCY = 0.015;

   private final RigidBodyTransform pelvisPose = new RigidBodyTransform();
   private final RigidBodyTransform errorBetweenCurrentPositionAndCorrected = new RigidBodyTransform();
   private final RigidBodyTransform totalError = new RigidBodyTransform();
   private final RigidBodyTransform translationErrorInPastTransform = new RigidBodyTransform();
   private final RigidBodyTransform rotationErrorInPastTransform = new RigidBodyTransform();
   private final RigidBodyTransform interpolatedTranslationError = new RigidBodyTransform();
   private final RigidBodyTransform interpolatedRotationError = new RigidBodyTransform();

   private final TimeStampedTransform3D seTimeStampedPose = new TimeStampedTransform3D();

   /** expressed in worldFrame */
   private final YoReferencePose translationCorrection;

   /** expressed in nonCorrectedPelvisFrame */
   private final YoReferencePose orientationCorrection;

   /** expressed in translationCorrection */
   private final YoReferencePose nonCorrectedPelvis;

   /** expressed in worldFrame */
   private final YoReferencePose correctedPelvis;

   private final YoReferencePose pelvisStateAtLocalizationTimeTranslationFrame;
   private final YoReferencePose pelvisStateAtLocalizationTimeRotationFrame;
   private final YoReferencePose newLocalizationTranslationFrame;
   private final YoReferencePose newLocalizationRotationFrame;
   private final YoReferencePose totalRotationErrorFrame;
   private final YoReferencePose totalTranslationErrorFrame;
   private final YoReferencePose interpolatedRotationCorrectionFrame;
   private final YoReferencePose interpolatedTranslationCorrectionFrame;
   private final YoReferencePose interpolationRotationStartFrame;
   private final YoReferencePose interpolationTranslationStartFrame;
   private final Vector3D distanceToTravelVector = new Vector3D();
   private final AxisAngle angleToTravelAxis4d = new AxisAngle();

   private final YoLong seNonProcessedPelvisTimeStamp;

   private final AlphaFilteredYoVariable interpolationTranslationAlphaFilter;
   private final AlphaFilteredYoVariable interpolationRotationAlphaFilter;
   private final YoDouble confidenceFactor; // target for alpha filter
   private final YoDouble interpolationTranslationAlphaFilterBreakFrequency;
   private final YoDouble interpolationRotationAlphaFilterBreakFrequency;
   private final YoDouble distanceToTravel;
   private final YoDouble distanceTraveled;
   private final YoDouble angleToTravel;
   private final YoDouble angleTraveled;
   private final YoDouble previousTranslationClippedAlphaValue;
   private final YoDouble previousRotationClippedAlphaValue;
   private final YoDouble translationClippedAlphaValue;
   private final YoDouble rotationClippedAlphaValue;
   private final YoDouble maxTranslationVelocityClip;
   private final YoDouble maxRotationVelocityClip;
   private final YoDouble maxTranslationAlpha;
   private final YoDouble maxRotationAlpha;

   private final YoDouble interpolationTranslationAlphaFilterAlphaValue;
   private final YoDouble interpolationRotationAlphaFilterAlphaValue;

   private final YoBoolean manuallyTriggerLocalizationUpdate;
   private final YoDouble manualTranslationOffsetX, manualTranslationOffsetY, manualTranslationOffsetZ;
   private final YoDouble manualRotationOffsetInRadX, manualRotationOffsetInRadY, manualRotationOffsetInRadZ;

   private final double estimatorDT;
   private boolean sendCorrectionUpdate = false;
   private final Quaternion totalRotationError = new Quaternion();
   private final Vector3D totalTranslationError = new Vector3D();

   private final Vector3D localizationTranslationInPast = new Vector3D();
   private final Vector3D seTranslationInPast = new Vector3D();
   private final Quaternion seRotationInPast = new Quaternion();
   private final Quaternion localizationRotationInPast = new Quaternion();

   public PelvisPoseHistoryCorrection(FullInverseDynamicsStructure inverseDynamicsStructure, final double dt, YoVariableRegistry parentRegistry,
         int pelvisBufferSize)
   {
      this(inverseDynamicsStructure.getRootJoint(), dt, parentRegistry, pelvisBufferSize, null);
   }

   public PelvisPoseHistoryCorrection(FullInverseDynamicsStructure inverseDynamicsStructure,
         PelvisPoseCorrectionCommunicatorInterface externalPelvisPoseSubscriber, final double dt, YoVariableRegistry parentRegistry, int pelvisBufferSize)
   {
      this(inverseDynamicsStructure.getRootJoint(), dt, parentRegistry, pelvisBufferSize, externalPelvisPoseSubscriber);
   }

   public PelvisPoseHistoryCorrection(FloatingInverseDynamicsJoint sixDofJoint, final double estimatorDT, YoVariableRegistry parentRegistry, int pelvisBufferSize,
         PelvisPoseCorrectionCommunicatorInterface externalPelvisPoseSubscriber)
   {
      this.estimatorDT = estimatorDT;

      this.rootJoint = sixDofJoint;
      this.pelvisReferenceFrame = rootJoint.getFrameAfterJoint();
      this.pelvisPoseCorrectionCommunicator = externalPelvisPoseSubscriber;
      this.registry = new YoVariableRegistry(getClass().getSimpleName());
      parentRegistry.addChild(registry);

      stateEstimatorPelvisPoseBuffer = new TimeStampedTransformBuffer(pelvisBufferSize);

      pelvisStateAtLocalizationTimeTranslationFrame = new YoReferencePose("pelvisStateAtLocalizationTimeTranslationFrame", worldFrame, registry);
      pelvisStateAtLocalizationTimeRotationFrame = new YoReferencePose("pelvisStateAtLocalizationTimeRotationFrame", worldFrame, registry);

      newLocalizationTranslationFrame = new YoReferencePose("newLocalizationTranslationFrame", worldFrame, registry);
      newLocalizationRotationFrame = new YoReferencePose("newLocalizationRotationFrame", worldFrame, registry);

      interpolationTranslationStartFrame = new YoReferencePose("interpolationTranslationStartFrame", worldFrame, registry);
      interpolationRotationStartFrame = new YoReferencePose("interpolationRotationStartFrame", worldFrame, registry);

      totalTranslationErrorFrame = new YoReferencePose("totalTranslationErrorFrame", worldFrame, registry);
      totalRotationErrorFrame = new YoReferencePose("totalRotationErrorFrame", worldFrame, registry);

      interpolatedTranslationCorrectionFrame = new YoReferencePose("interpolatedTranslationCorrectionFrame", worldFrame, registry);
      interpolatedRotationCorrectionFrame = new YoReferencePose("interpolatedRotationCorrectionFrame", worldFrame, registry);

      translationCorrection = new YoReferencePose("translationCorrection", worldFrame, registry);
      nonCorrectedPelvis = new YoReferencePose("nonCorrectedPelvis", translationCorrection, registry);
      orientationCorrection = new YoReferencePose("orientationCorrection", nonCorrectedPelvis, registry);
      correctedPelvis = new YoReferencePose("correctedPelvis", worldFrame, registry);

      interpolationTranslationAlphaFilterAlphaValue = new YoDouble("interpolationTranslationAlphaFilterAlphaValue", registry);
      interpolationTranslationAlphaFilterBreakFrequency = new YoDouble("interpolationTranslationAlphaFilterBreakFrequency", registry);
      interpolationTranslationAlphaFilter = new AlphaFilteredYoVariable("PelvisTranslationErrorCorrectionAlphaFilter", registry,
            interpolationTranslationAlphaFilterAlphaValue);

      interpolationRotationAlphaFilterAlphaValue = new YoDouble("interpolationRotationAlphaFilterAlphaValue", registry);
      interpolationRotationAlphaFilterBreakFrequency = new YoDouble("interpolationRotationAlphaFilterBreakFrequency", registry);
      interpolationRotationAlphaFilter = new AlphaFilteredYoVariable("PelvisRotationErrorCorrectionAlphaFilter", registry,
            interpolationRotationAlphaFilterAlphaValue);

      interpolationTranslationAlphaFilterBreakFrequency.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            double alpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(interpolationTranslationAlphaFilterBreakFrequency.getDoubleValue(),
                  estimatorDT);
            interpolationTranslationAlphaFilterAlphaValue.set(alpha);
         }
      });
      interpolationTranslationAlphaFilterBreakFrequency.set(DEFAULT_BREAK_FREQUENCY);

      interpolationRotationAlphaFilterBreakFrequency.addVariableChangedListener(new VariableChangedListener()
      {

         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            double alpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(interpolationRotationAlphaFilterBreakFrequency.getDoubleValue(),
                  estimatorDT);
            interpolationRotationAlphaFilterAlphaValue.set(alpha);
         }
      });
      interpolationRotationAlphaFilterBreakFrequency.set(DEFAULT_BREAK_FREQUENCY);

      confidenceFactor = new YoDouble("PelvisErrorCorrectionConfidenceFactor", registry);

      seNonProcessedPelvisTimeStamp = new YoLong("seNonProcessedPelvis_timestamp", registry);

      translationClippedAlphaValue = new YoDouble("translationClippedAlphaValue", registry);
      rotationClippedAlphaValue = new YoDouble("rotationClippedAlphaValue", registry);
      distanceTraveled = new YoDouble("distanceTraveled", registry);
      angleTraveled = new YoDouble("angleTraveled", registry);
      maxTranslationVelocityClip = new YoDouble("maxTranslationVelocityClip", registry);
      maxTranslationVelocityClip.set(0.01);
      maxRotationVelocityClip = new YoDouble("maxRotationVelocityClip", registry);
      maxRotationVelocityClip.set(0.005); // TODO Determine a good default Value
      previousTranslationClippedAlphaValue = new YoDouble("previousTranslationClippedAlphaValue", registry);
      previousRotationClippedAlphaValue = new YoDouble("previousRotationClippedAlphaValue", registry);
      maxTranslationAlpha = new YoDouble("maxTranslationAlpha", registry);
      maxRotationAlpha = new YoDouble("maxRotationAlpha", registry);
      distanceToTravel = new YoDouble("distanceToTravel", registry);
      angleToTravel = new YoDouble("angleToTravel", registry);

      //      distanceError = new YoDouble("distanceError", registry);

      manuallyTriggerLocalizationUpdate = new YoBoolean("manuallyTriggerLocalizationUpdate", registry);

      manualTranslationOffsetX = new YoDouble("manualTranslationOffset_X", registry);
      manualTranslationOffsetY = new YoDouble("manualTranslationOffset_Y", registry);
      manualTranslationOffsetZ = new YoDouble("manualTranslationOffset_Z", registry);
      manualRotationOffsetInRadX = new YoDouble("manualRotationOffsetInRad_X", registry);
      manualRotationOffsetInRadY = new YoDouble("manualRotationOffsetInRad_Y", registry);
      manualRotationOffsetInRadZ = new YoDouble("manualRotationOffsetInRad_Z", registry);
      //defaultValues for testing with Atlas.
//      manualTranslationOffsetX.set(-0.11404);
//      manualTranslationOffsetY.set(0.00022);
//      manualTranslationOffsetZ.set(0.78931);
//      manualRotationOffsetInRadX.set(-0.00002);
//      manualRotationOffsetInRadY.set(0.00024);
//      manualRotationOffsetInRadZ.set(-0.00052);
   }

   /* (non-Javadoc)
    * @see us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.PelvisPoseHistoryCorrectionInterface#doControl(long)
    */
   @Override
   public void doControl(long timestamp)
   {
      if (pelvisPoseCorrectionCommunicator != null)
      {
         pelvisReferenceFrame.update();
         checkForManualTrigger();
         checkForNewPacket();

         interpolationTranslationAlphaFilter.update(confidenceFactor.getDoubleValue());
         interpolationRotationAlphaFilter.update(confidenceFactor.getDoubleValue());

         pelvisReferenceFrame.getTransformToParent(pelvisPose);
         addPelvisePoseToPelvisBuffer(pelvisPose, timestamp);

         nonCorrectedPelvis.setAndUpdate(pelvisPose);
         correctPelvisPose(pelvisPose);
         correctedPelvis.setAndUpdate(pelvisPose);

         rootJoint.setPositionAndRotation(pelvisPose);
         pelvisReferenceFrame.update();
         checkForNeedToSendCorrectionUpdate();
      }
   }

   private void checkForNeedToSendCorrectionUpdate()
   {
      if (sendCorrectionUpdate)
      {
         sendCorrectionUpdatePacket();
         sendCorrectionUpdate = false;
      }
   }

   /**
    * triggers manual localization offset
    */
   private void checkForManualTrigger()
   {
      if (manuallyTriggerLocalizationUpdate.getBooleanValue())
      {
         manuallyTriggerLocalizationUpdate();
         sendCorrectionUpdate = true;
      }
   }

   /**
    * poll for new packet, input for unit tests and real robot
    */
   private void checkForNewPacket()
   {
      if (pelvisPoseCorrectionCommunicator.hasNewPose())
      {
         processNewPacket();
         sendCorrectionUpdate = true;
      }
   }

   /**
    * Updates max velocity clipping, interpolates from where we were 
    * at the last correction tick to the goal and updates the pelvis
    * @param pelvisPose - non corrected pelvis position
    */
   private void correctPelvisPose(RigidBodyTransform pelvisPose)
   {
      updateTranslationalMaxVelocityClip();
      updateRotationalMaxVelocityClip();

      interpolatedRotationCorrectionFrame.interpolate(interpolationRotationStartFrame, totalRotationErrorFrame, rotationClippedAlphaValue.getDoubleValue());

      interpolatedTranslationCorrectionFrame.interpolate(interpolationTranslationStartFrame, totalTranslationErrorFrame,
            translationClippedAlphaValue.getDoubleValue());

      if (USE_ROTATION_CORRECTION)
         interpolatedRotationCorrectionFrame.getTransformToParent(interpolatedRotationError);
      else
         interpolatedRotationError.setIdentity();

      orientationCorrection.setAndUpdate(interpolatedRotationError);

      interpolatedTranslationCorrectionFrame.getTransformToParent(interpolatedTranslationError);
      translationCorrection.setAndUpdate(interpolatedTranslationError);

      orientationCorrection.getTransformToDesiredFrame(pelvisPose, worldFrame);
   }

   /**
    * clips max translational velocity 
    */
   private void updateTranslationalMaxVelocityClip()
   {
      interpolatedTranslationCorrectionFrame.getTransformToDesiredFrame(errorBetweenCurrentPositionAndCorrected, worldFrame);
      errorBetweenCurrentPositionAndCorrected.getTranslation(distanceToTravelVector);
      distanceToTravel.set(distanceToTravelVector.length());
      maxTranslationAlpha.set((estimatorDT * maxTranslationVelocityClip.getDoubleValue() / distanceToTravel.getDoubleValue())
            + previousTranslationClippedAlphaValue.getDoubleValue());
      translationClippedAlphaValue.set(MathTools.clamp(interpolationTranslationAlphaFilter.getDoubleValue(), 0.0, maxTranslationAlpha.getDoubleValue()));
      previousTranslationClippedAlphaValue.set(translationClippedAlphaValue.getDoubleValue());
   }

   /**
    * clips max rotational velocity 
    */
   private void updateRotationalMaxVelocityClip()
   {
      interpolatedRotationCorrectionFrame.getTransformToDesiredFrame(errorBetweenCurrentPositionAndCorrected, worldFrame);
      errorBetweenCurrentPositionAndCorrected.getRotation(angleToTravelAxis4d);
      angleToTravel.set(angleToTravelAxis4d.getAngle());
      maxRotationAlpha.set((estimatorDT * maxRotationVelocityClip.getDoubleValue() / angleToTravel.getDoubleValue())
            + previousRotationClippedAlphaValue.getDoubleValue());
      rotationClippedAlphaValue.set(MathTools.clamp(interpolationRotationAlphaFilter.getDoubleValue(), 0.0, maxRotationAlpha.getDoubleValue()));
      previousRotationClippedAlphaValue.set(rotationClippedAlphaValue.getDoubleValue());
   }

   /**
    * adds noncorrected pelvis poses to buffer for pelvis pose lookups in past
    * @param pelvisPose non-corrected pelvis pose
    * @param timeStamp robot timestamp of pelvis pose
    */
   private void addPelvisePoseToPelvisBuffer(RigidBodyTransform pelvisPose, long timeStamp)
   {
      seNonProcessedPelvisTimeStamp.set(timeStamp);
      stateEstimatorPelvisPoseBuffer.put(pelvisPose, timeStamp);
   }

   private final TimeStampedTransform3D timeStampedExternalPose = new TimeStampedTransform3D();
   /**
    * pulls the corrected pose from the buffer, check that the nonprocessed buffer has
    * corresponding pelvis poses and calculates the total error
    */
   private void processNewPacket()
   {
      StampedPosePacket newPacket = pelvisPoseCorrectionCommunicator.getNewExternalPose();
      timeStampedExternalPose.setTransform3D(newPacket.getPose());
      timeStampedExternalPose.setTimeStamp(newPacket.getTimeStamp());

      if (stateEstimatorPelvisPoseBuffer.isInRange(timeStampedExternalPose.getTimeStamp()))
      {
         double confidence = newPacket.getConfidenceFactor();
         confidence = MathTools.clamp(confidence, 0.0, 1.0);
         confidenceFactor.set(confidence);
         addNewExternalPose(timeStampedExternalPose);
      }
   }

   /**
    * sets initials for correction and calculates error in past
    */
   private void addNewExternalPose(TimeStampedTransform3D newPelvisPoseWithTime)
   {
      previousTranslationClippedAlphaValue.set(0.0);
      interpolationTranslationAlphaFilter.set(0.0);
      distanceTraveled.set(0.0);

      previousRotationClippedAlphaValue.set(0.0);
      interpolationRotationAlphaFilter.set(0.0);
      angleTraveled.set(0.0);

      calculateAndStoreErrorInPast(newPelvisPoseWithTime);
      interpolationRotationStartFrame.setAndUpdate(interpolatedRotationCorrectionFrame.getTransformToParent());
      interpolationTranslationStartFrame.setAndUpdate(interpolatedTranslationCorrectionFrame.getTransformToParent());
   }

   /**
    * Calculates the difference between the external at t with the state estimated pelvis pose at t and stores it
    * @param localizationPose - the corrected pelvis pose
    */
   public void calculateAndStoreErrorInPast(TimeStampedTransform3D timestampedlocalizationPose)
   {
      long timeStamp = timestampedlocalizationPose.getTimeStamp();
      RigidBodyTransform localizationPose = timestampedlocalizationPose.getTransform3D();

      localizationPose.getTranslation(localizationTranslationInPast);
      newLocalizationTranslationFrame.setAndUpdate(localizationTranslationInPast);

      localizationPose.getRotation(localizationRotationInPast);
      newLocalizationRotationFrame.setAndUpdate(localizationRotationInPast);

      stateEstimatorPelvisPoseBuffer.findTransform(timeStamp, seTimeStampedPose);
      RigidBodyTransform sePose = seTimeStampedPose.getTransform3D();

      sePose.getTranslation(seTranslationInPast);
      pelvisStateAtLocalizationTimeTranslationFrame.setAndUpdate(seTranslationInPast);

      sePose.getRotation(seRotationInPast);
      pelvisStateAtLocalizationTimeRotationFrame.setAndUpdate(seRotationInPast);

      newLocalizationTranslationFrame.getTransformToDesiredFrame(translationErrorInPastTransform, pelvisStateAtLocalizationTimeTranslationFrame);
      newLocalizationRotationFrame.getTransformToDesiredFrame(rotationErrorInPastTransform, pelvisStateAtLocalizationTimeRotationFrame);

      totalTranslationErrorFrame.setAndUpdate(translationErrorInPastTransform);
      totalRotationErrorFrame.setAndUpdate(rotationErrorInPastTransform);
   }

   /* (non-Javadoc)
    * @see us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.PelvisPoseHistoryCorrectionInterface#manuallyTriggerLocalizationUpdate()
    */
   public void manuallyTriggerLocalizationUpdate()
   {
      confidenceFactor.set(1.0);

      RigidBodyTransform pelvisPose = new RigidBodyTransform();

      Quaternion rotation = new Quaternion();
      pelvisPose.getRotation(rotation);
      rotation.setYawPitchRoll(manualRotationOffsetInRadZ.getDoubleValue(), manualRotationOffsetInRadY.getDoubleValue(), manualRotationOffsetInRadX.getDoubleValue());
      pelvisPose.setRotation(rotation);

      Vector3D translation = new Vector3D();
      pelvisPose.getTranslation(translation);
      translation.setX(manualTranslationOffsetX.getDoubleValue());
      translation.setY(manualTranslationOffsetY.getDoubleValue());
      translation.setZ(manualTranslationOffsetZ.getDoubleValue());
      pelvisPose.setTranslation(translation);

      TimeStampedTransform3D manualTimeStampedTransform3D = new TimeStampedTransform3D(pelvisPose, stateEstimatorPelvisPoseBuffer.getNewestTimestamp());
      addNewExternalPose(manualTimeStampedTransform3D);
      manuallyTriggerLocalizationUpdate.set(false);
   }

   Vector3D translationalResidualError = new Vector3D();
   Vector3D translationalTotalError = new Vector3D();
   //TODO Check how to integrate the rotationCorrection here
   private void sendCorrectionUpdatePacket()
   {
      totalRotationErrorFrame.get(totalRotationError);
      totalTranslationErrorFrame.get(totalTranslationError);
      totalError.set(totalRotationError, totalTranslationError);
      
      errorBetweenCurrentPositionAndCorrected.getTranslation(translationalResidualError);
      
      double absoluteResidualError = translationalResidualError.length();
      
      totalError.getTranslation(translationalTotalError);
      
      double absoluteTotalError = translationalTotalError.length();

      PelvisPoseErrorPacket pelvisPoseErrorPacket = HumanoidMessageTools.createPelvisPoseErrorPacket((float) absoluteTotalError, (float) absoluteResidualError, false);
      pelvisPoseCorrectionCommunicator.sendPelvisPoseErrorPacket(pelvisPoseErrorPacket);
   }

   /* (non-Javadoc)
    * @see us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.PelvisPoseHistoryCorrectionInterface#setExternelPelvisCorrectorSubscriber(us.ihmc.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface)
    */
   @Override
   public void setExternalPelvisCorrectorSubscriber(PelvisPoseCorrectionCommunicatorInterface externalPelvisPoseSubscriber)
   {
      this.pelvisPoseCorrectionCommunicator = externalPelvisPoseSubscriber;
   }
}
