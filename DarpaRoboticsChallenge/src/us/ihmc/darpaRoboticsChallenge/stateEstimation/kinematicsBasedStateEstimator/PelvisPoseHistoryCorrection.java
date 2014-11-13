package us.ihmc.darpaRoboticsChallenge.stateEstimation.kinematicsBasedStateEstimator;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packets.StampedPosePacket;
import us.ihmc.communication.packets.sensing.PelvisPoseErrorPacket;
import us.ihmc.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.communication.subscribers.TimeStampedPelvisPoseBuffer;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.utilities.Axis;
import us.ihmc.utilities.kinematics.TimeStampedTransform3D;
import us.ihmc.utilities.kinematics.TransformInterpolationCalculator;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.math.geometry.RotationFunctions;
import us.ihmc.utilities.math.geometry.TransformTools;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.yoUtilities.dataStructure.listener.VariableChangedListener;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.LongYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;
import us.ihmc.yoUtilities.math.filters.AlphaFilteredYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFramePose;


public class PelvisPoseHistoryCorrection
{
   private final TimeStampedPelvisPoseBuffer stateEstimatorPelvisPoseBuffer;
   private PelvisPoseCorrectionCommunicatorInterface pelvisPoseCorrectionCommunicator;
   private final SixDoFJoint rootJoint;
   private final ReferenceFrame pelvisReferenceFrame;
   private final YoVariableRegistry registry;
   private static final double DEFAULT_BREAK_FREQUENCY = 0.015;
   
   private final RigidBodyTransform pelvisPose = new RigidBodyTransform();
   private final RigidBodyTransform interpolatedError = new RigidBodyTransform();
   private final RigidBodyTransform errorBetweenCurrentPositionAndCorrected = new RigidBodyTransform();
   private final RigidBodyTransform totalError = new RigidBodyTransform();

   RigidBodyTransform errorInPastTransform = new RigidBodyTransform();

   private final YoReferencePose nonCorrectedPelvis;
   private final YoReferencePose correctedPelvis;
   private final YoReferencePose seBackInTimeFrame;
   private final YoReferencePose localizationBackInTimeFrame;
   private final YoReferencePose totalErrorFrame;
   private final YoReferencePose interpolatedCorrectionFrame;
   private final YoReferencePose interpolationStartFrame;


   private final Vector3d distanceToTravelVector = new Vector3d();
   
   private final LongYoVariable seNonProcessedPelvisTimeStamp;

   private final AlphaFilteredYoVariable interpolationAlphaFilter;
   private final DoubleYoVariable confidenceFactor; // target for alpha filter
   private final DoubleYoVariable interpolationAlphaFilterBreakFrequency;
   private final DoubleYoVariable distanceToTravel;
   private final DoubleYoVariable distanceTraveled;
   private final DoubleYoVariable previousAlphaValue;
   private final DoubleYoVariable clippedAlphaValue;
   private final DoubleYoVariable maxVelocityClip;
   private final DoubleYoVariable maxAlpha;

   private final DoubleYoVariable interpolationAlphaFilterAlphaValue;

   private final BooleanYoVariable manuallyTriggerLocalizationUpdate;
   private final DoubleYoVariable manualTranslationOffsetX, manualTranslationOffsetY, manualTranslationOffsetZ;
   private final DoubleYoVariable manualRotationOffsetInRadX, manualRotationOffsetInRadY, manualRotationOffsetInRadZ;
   
   private final double estimatorDT;
   private boolean sendCorrectionUpdate = false;

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

   public PelvisPoseHistoryCorrection(SixDoFJoint sixDofJoint, final double estimatorDT, YoVariableRegistry parentRegistry, int pelvisBufferSize,
         PelvisPoseCorrectionCommunicatorInterface externalPelvisPoseSubscriber)
   {
      this.estimatorDT = estimatorDT;
      
      this.rootJoint = sixDofJoint;
      this.pelvisReferenceFrame = rootJoint.getFrameAfterJoint();
      this.pelvisPoseCorrectionCommunicator = externalPelvisPoseSubscriber;
      this.registry = new YoVariableRegistry("PelvisPoseHistoryCorrection");
      parentRegistry.addChild(registry);
      
      stateEstimatorPelvisPoseBuffer = new TimeStampedPelvisPoseBuffer(pelvisBufferSize);
      
      seBackInTimeFrame = new YoReferencePose("seBackInTimeFrame", ReferenceFrame.getWorldFrame(), registry);
      localizationBackInTimeFrame = new YoReferencePose("localizationBackInTimeFrame", ReferenceFrame.getWorldFrame(), registry);
      totalErrorFrame = new YoReferencePose("totalErrorFrame", pelvisReferenceFrame, registry);
      interpolationStartFrame = new YoReferencePose("interpolationStartFrame", pelvisReferenceFrame, registry);
      interpolatedCorrectionFrame = new YoReferencePose("interpolatedCorrectionFrame", pelvisReferenceFrame, registry);
      nonCorrectedPelvis = new YoReferencePose("nonCorrectedPelvis", ReferenceFrame.getWorldFrame(), registry);
      correctedPelvis = new YoReferencePose("correctedPelvis", ReferenceFrame.getWorldFrame(), registry);

      interpolationAlphaFilterAlphaValue = new DoubleYoVariable("interpolationAlphaFilterAlphaValue", registry);
      interpolationAlphaFilterBreakFrequency = new DoubleYoVariable("interpolationAlphaFilterBreakFrequency", registry);
      interpolationAlphaFilter = new AlphaFilteredYoVariable("PelvisErrorCorrectionAlphaFilter", registry, interpolationAlphaFilterAlphaValue);
      
      interpolationAlphaFilterBreakFrequency.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            double alpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequency(interpolationAlphaFilterBreakFrequency.getDoubleValue(), estimatorDT);
            interpolationAlphaFilter.setAlpha(alpha);
         }
      });
      
      interpolationAlphaFilterBreakFrequency.set(DEFAULT_BREAK_FREQUENCY);
      confidenceFactor = new DoubleYoVariable("PelvisErrorCorrectionConfidenceFactor", registry);

      seNonProcessedPelvisTimeStamp = new LongYoVariable("seNonProcessedPelvis_timestamp", registry);

      clippedAlphaValue = new DoubleYoVariable("clippedAlphaValue", registry);
      distanceTraveled = new DoubleYoVariable("distanceTraveled", registry);
      maxVelocityClip = new DoubleYoVariable("maxVelocityClip", registry);
      maxVelocityClip.set(0.01);
      previousAlphaValue = new DoubleYoVariable("previousAlphaValue", registry);
      maxAlpha = new DoubleYoVariable("maxAlpha", registry);
      distanceToTravel = new DoubleYoVariable("distanceToTravel", registry);
      //      distanceError = new DoubleYoVariable("distanceError", registry);
      
      manuallyTriggerLocalizationUpdate = new BooleanYoVariable("manuallyTriggerLocalizationUpdate", registry);

      manualTranslationOffsetX = new DoubleYoVariable("manualTranslationOffset_X", registry);
      manualTranslationOffsetY = new DoubleYoVariable("manualTranslationOffset_Y", registry);
      manualTranslationOffsetZ = new DoubleYoVariable("manualTranslationOffset_Z", registry);
      manualRotationOffsetInRadX = new DoubleYoVariable("manualRotationOffsetInRad_X", registry);
      manualRotationOffsetInRadY = new DoubleYoVariable("manualRotationOffsetInRad_Y", registry);
      manualRotationOffsetInRadZ = new DoubleYoVariable("manualRotationOffsetInRad_Z", registry);
   }
   
   /**
    * Converges the state estimator pelvis pose towards an external position provided by an external Pelvis Pose Subscriber
    * @param l 
    */
   public void doControl(long timestamp)
   {
      if (pelvisPoseCorrectionCommunicator != null)
      {
         checkForManualTrigger();
         checkForNewPacket();

         pelvisReferenceFrame.update();
         interpolationAlphaFilter.update(confidenceFactor.getDoubleValue());

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
      interpolatedCorrectionFrame.interpolate(interpolationStartFrame, totalErrorFrame, pelvisReferenceFrame, clippedAlphaValue.getDoubleValue());
      interpolatedCorrectionFrame.getTransformToParent(interpolatedError);
      pelvisPose.multiply(interpolatedError);
   }
   
   /**
    * clips max translational velocity 
    */
   private void updateTranslationalMaxVelocityClip()
   {
      interpolatedCorrectionFrame.getTransformToDesiredFrame(errorBetweenCurrentPositionAndCorrected, totalErrorFrame);
      errorBetweenCurrentPositionAndCorrected.getTranslation(distanceToTravelVector);
      distanceToTravel.set(distanceToTravelVector.length());
      maxAlpha.set((estimatorDT * maxVelocityClip.getDoubleValue() / distanceToTravel.getDoubleValue()) + previousAlphaValue.getDoubleValue());
      clippedAlphaValue.set(MathTools.clipToMinMax(interpolationAlphaFilter.getDoubleValue(), 0.0, maxAlpha.getDoubleValue()));
      previousAlphaValue.set(clippedAlphaValue.getDoubleValue());
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


   /**
    * pulls the corrected pose from the buffer, check that the nonprocessed buffer has
    * corresponding pelvis poses and calculates the total error
    */
   private void processNewPacket()
   {
      StampedPosePacket newPacket = pelvisPoseCorrectionCommunicator.getNewExternalPose();
      TimeStampedTransform3D timeStampedExternalPose = newPacket.getTransform();

      if (stateEstimatorPelvisPoseBuffer.isInRange(timeStampedExternalPose.getTimeStamp()))
      {
         double confidence = newPacket.getConfidenceFactor();
         confidence = MathTools.clipToMinMax(confidence, 0.0, 1.0);
         confidenceFactor.set(confidence);
         addNewExternalPose(timeStampedExternalPose);
      }
   }

   /**
    * sets initials for correction and calculates error in past
    */
   private void addNewExternalPose(TimeStampedTransform3D newPelvisPoseWithTime)
   {
      previousAlphaValue.set(0.0);
      interpolationAlphaFilter.set(0.0);
      distanceTraveled.set(0.0);
      calculateAndStoreErrorInPast(newPelvisPoseWithTime);
      interpolationStartFrame.setAndUpdate(interpolatedCorrectionFrame.getTransformToParent());
   }


   /**
    * Calculates the difference between the external at t with the state estimated pelvis pose at t and stores it
    * @param localizationPose - the corrected pelvis pose
    */
   public void calculateAndStoreErrorInPast(TimeStampedTransform3D localizationPose)
   {
      long timeStamp = localizationPose.getTimeStamp();
      TimeStampedTransform3D sePose = stateEstimatorPelvisPoseBuffer.interpolate(timeStamp);
      
      localizationBackInTimeFrame.setAndUpdate(localizationPose.getTransform3D());
      seBackInTimeFrame.setAndUpdate(sePose.getTransform3D());
      
      localizationBackInTimeFrame.getTransformToDesiredFrame(errorInPastTransform, seBackInTimeFrame);
      totalErrorFrame.setAndUpdate(errorInPastTransform);
   }

   public void manuallyTriggerLocalizationUpdate()
   {
      confidenceFactor.set(1.0);
      
      long midTimeStamp = stateEstimatorPelvisPoseBuffer.getOldestTimestamp()
            + ((stateEstimatorPelvisPoseBuffer.getNewestTimestamp() - stateEstimatorPelvisPoseBuffer.getOldestTimestamp()) / 2);
      RigidBodyTransform pelvisPose = new RigidBodyTransform(stateEstimatorPelvisPoseBuffer.interpolate(midTimeStamp).getTransform3D());

      TransformTools.rotate(pelvisPose, manualRotationOffsetInRadX.getDoubleValue(), Axis.X);
      TransformTools.rotate(pelvisPose, manualRotationOffsetInRadY.getDoubleValue(), Axis.Y);
      TransformTools.rotate(pelvisPose, manualRotationOffsetInRadZ.getDoubleValue(), Axis.Z);

      Vector3d translation = new Vector3d();
      pelvisPose.get(translation);
      translation.setX(translation.getX() - manualTranslationOffsetX.getDoubleValue());
      translation.setY(translation.getY() - manualTranslationOffsetY.getDoubleValue());
      translation.setZ(translation.getZ() - manualTranslationOffsetZ.getDoubleValue());
      pelvisPose.setTranslation(translation);

      TimeStampedTransform3D testTransform = new TimeStampedTransform3D(pelvisPose, stateEstimatorPelvisPoseBuffer.getNewestTimestamp());
      addNewExternalPose(testTransform);
      manuallyTriggerLocalizationUpdate.set(false);

   }
   
   private void sendCorrectionUpdatePacket()
   {
      totalErrorFrame.getTransformToParent(totalError);
      double maxCorrectionVelocity = maxVelocityClip.getDoubleValue();
      PelvisPoseErrorPacket pelvisPoseErrorPacket = new PelvisPoseErrorPacket(totalError, errorBetweenCurrentPositionAndCorrected, maxCorrectionVelocity);
      pelvisPoseCorrectionCommunicator.sendPelvisPoseErrorPacket(pelvisPoseErrorPacket);
   }

   public void setExternelPelvisCorrectorSubscriber(PelvisPoseCorrectionCommunicatorInterface externalPelvisPoseSubscriber)
   {
      this.pelvisPoseCorrectionCommunicator = externalPelvisPoseSubscriber;
   }
   
   private class YoReferencePose extends ReferenceFrame
   {
      private static final long serialVersionUID = -7908261385108357220L;

      private final YoFramePose yoFramePose;

      //Below are used for interpolation only
      private final TransformInterpolationCalculator transformInterpolationCalculator = new TransformInterpolationCalculator();
      private final RigidBodyTransform interpolationStartingPosition = new RigidBodyTransform();
      private final RigidBodyTransform interpolationGoalPosition = new RigidBodyTransform();
      private final RigidBodyTransform output = new RigidBodyTransform();
      
      //Below are used for updating YoFramePose only
      private final Quat4d rotation = new Quat4d();
      private final Vector3d translation = new Vector3d();
      private final double[] yawPitchRoll = new double[3];

      public YoReferencePose(String frameName, ReferenceFrame parentFrame, YoVariableRegistry registry)
      {
         super(frameName, parentFrame);
         yoFramePose = new YoFramePose(frameName + "_", this, registry);
      }

      @Override
      protected void updateTransformToParent(RigidBodyTransform transformToParent)
      {
         yoFramePose.getOrientation().getQuaternion(rotation);
         transformToParent.setRotation(rotation);
         YoFramePoint yoFramePoint = yoFramePose.getPosition();
         transformToParent.setTranslation(yoFramePoint.getX(), yoFramePoint.getY(), yoFramePoint.getZ());
      }
      
      public void setAndUpdate(RigidBodyTransform transform)
      {
         transform.get(rotation, translation);
         setAndUpdate(rotation, translation);
      }

      public void setAndUpdate(Quat4d newRotation, Vector3d newTranslation)
      {
         set(newRotation);
         set(newTranslation);
         update();
      }

      private void set(Quat4d newRotation)
      {
         RotationFunctions.setYawPitchRollBasedOnQuaternion(yawPitchRoll, newRotation);
         yoFramePose.setYawPitchRoll(yawPitchRoll);
      }

      private void set(Vector3d newTranslation)
      {
         yoFramePose.setXYZ(newTranslation.getX(), newTranslation.getY(), newTranslation.getZ());
      }

      public void interpolate(YoReferencePose start, YoReferencePose goal, ReferenceFrame referenceBetweenFrames, double alpha)
      {
         start.getTransformToDesiredFrame(interpolationStartingPosition, referenceBetweenFrames);
         goal.getTransformToDesiredFrame(interpolationGoalPosition, referenceBetweenFrames);
         
         transformInterpolationCalculator.computeInterpolation(interpolationStartingPosition, interpolationGoalPosition, output, alpha);
         setAndUpdate(output);
      }
   }
}
