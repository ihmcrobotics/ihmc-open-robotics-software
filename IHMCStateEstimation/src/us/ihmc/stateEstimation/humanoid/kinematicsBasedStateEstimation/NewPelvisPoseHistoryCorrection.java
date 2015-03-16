package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packets.StampedPosePacket;
import us.ihmc.communication.packets.sensing.PelvisPoseErrorPacket;
import us.ihmc.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.utilities.kinematics.TimeStampedTransform3D;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.math.geometry.RotationFunctions;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFramePose;

public class NewPelvisPoseHistoryCorrection implements PelvisPoseHistoryCorrectionInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double DEFAULT_BREAK_FREQUENCY = 0.6;

   private final YoVariableRegistry registry;
   
   private PelvisPoseCorrectionCommunicatorInterface pelvisPoseCorrectionCommunicator;
   
   private final SixDoFJoint rootJoint;
   private final ReferenceFrame pelvisReferenceFrame;
   private final ClippedSpeedOffsetErrorInterpolator offsetErrorInterpolator;
   private final OutdatedPoseToUpToDateReferenceFrameUpdater outdatedPoseUpdater;
   
   private final double estimatorDT;
   private boolean sendCorrectionUpdate = false;

   private final DoubleYoVariable alphaFilterBreakFrequency;
   
   private final DoubleYoVariable confidenceFactor;
   
   private final BooleanYoVariable manuallyTriggerLocalizationUpdate;
   private final DoubleYoVariable manualTranslationOffsetX, manualTranslationOffsetY, manualTranslationOffsetZ;
   private final DoubleYoVariable manualRotationOffsetInRadX, manualRotationOffsetInRadY, manualRotationOffsetInRadZ;
   
   private final FramePose iterativeClosestPointInPelvisReferenceFramePose;
   private final FramePose correctedPelvisPoseInPelvisReferenceFramePose;
   private final RigidBodyTransform stateEstimatorPelvisTransformInWorld = new RigidBodyTransform();
   private final RigidBodyTransform correctedPelvisTransformInWorldFrame = new RigidBodyTransform();
   private final RigidBodyTransform iterativeClosestPointTransformInWorldFrame = new RigidBodyTransform();
   private final RigidBodyTransform errorTransformBetweenCurrentPositionAndCorrected = new RigidBodyTransform();
   private final IntegerYoVariable pelvisBufferSize;

   private final FramePose stateEstimatorInWorldFramePose = new FramePose(worldFrame);
   private final YoFramePose yoStateEstimatorInWorldFramePose;
   private final YoFramePose yoIterativeClosestPointInPelvisReferenceFramePose;
   private final YoFramePose yoCorrectedPelvisPoseInPelvisReferenceFramePose;
   private final YoFramePose yoCorrectedPelvisPoseInWorldFrame;
   private final FramePose iterativeClosestPointInWorldFramePose = new FramePose(worldFrame);
   private final YoFramePose yoIterativeClosestPointPoseInWorldFrame;
   
   public NewPelvisPoseHistoryCorrection(FullInverseDynamicsStructure inverseDynamicsStructure, final double dt, YoVariableRegistry parentRegistry,
         int pelvisBufferSize)
   {
      this(inverseDynamicsStructure.getRootJoint(), dt, parentRegistry, pelvisBufferSize, null);
   }

   public NewPelvisPoseHistoryCorrection(FullInverseDynamicsStructure inverseDynamicsStructure,
         PelvisPoseCorrectionCommunicatorInterface externalPelvisPoseSubscriber, final double dt, YoVariableRegistry parentRegistry, int pelvisBufferSize)
   {
      this(inverseDynamicsStructure.getRootJoint(), dt, parentRegistry, pelvisBufferSize, externalPelvisPoseSubscriber);
   }
   
   public NewPelvisPoseHistoryCorrection(SixDoFJoint sixDofJoint, final double estimatorDT, YoVariableRegistry parentRegistry, int pelvisBufferSize,
         PelvisPoseCorrectionCommunicatorInterface externalPelvisPoseSubscriber)
   {
      this.estimatorDT = estimatorDT;

      this.rootJoint = sixDofJoint;
      this.pelvisReferenceFrame = rootJoint.getFrameAfterJoint();
      this.pelvisPoseCorrectionCommunicator = externalPelvisPoseSubscriber;
      this.registry = new YoVariableRegistry("newPelvisPoseHistoryCorrection");
      parentRegistry.addChild(registry);
      
      this.pelvisBufferSize = new IntegerYoVariable("pelvisBufferSize", registry);
      this.pelvisBufferSize.set(pelvisBufferSize);
      
      alphaFilterBreakFrequency = new DoubleYoVariable("alphaFilterBreakFrequency", registry);
      alphaFilterBreakFrequency.set(DEFAULT_BREAK_FREQUENCY);
      
      confidenceFactor = new DoubleYoVariable("PelvisErrorCorrectionConfidenceFactor", registry);
      
      
      offsetErrorInterpolator = new ClippedSpeedOffsetErrorInterpolator(registry, pelvisReferenceFrame, alphaFilterBreakFrequency, this.estimatorDT);
      outdatedPoseUpdater = new OutdatedPoseToUpToDateReferenceFrameUpdater(pelvisBufferSize, pelvisReferenceFrame);
      
      iterativeClosestPointInPelvisReferenceFramePose = new FramePose(pelvisReferenceFrame);
      correctedPelvisPoseInPelvisReferenceFramePose = new FramePose(pelvisReferenceFrame);
      
      //used only for feedback in SCS
      yoStateEstimatorInWorldFramePose = new YoFramePose("stateEstimatorInWorldFramePose", worldFrame, registry);
      yoIterativeClosestPointInPelvisReferenceFramePose = new YoFramePose("iterativeClosestPointInPelvisReferenceFramePose", pelvisReferenceFrame, registry);
      yoCorrectedPelvisPoseInPelvisReferenceFramePose = new YoFramePose("correctedPelvisPoseInPelvisReferenceFramePose", pelvisReferenceFrame, registry);
      yoCorrectedPelvisPoseInWorldFrame = new YoFramePose("correctedPelvisPoseInWorldFrame", worldFrame, registry);
      yoIterativeClosestPointPoseInWorldFrame = new YoFramePose("iterativeClosestPointPoseInWorldFrame", worldFrame, registry);
      
      manuallyTriggerLocalizationUpdate = new BooleanYoVariable("manuallyTriggerLocalizationUpdate", registry);
      manuallyTriggerLocalizationUpdate.set(false);
      
      manualTranslationOffsetX = new DoubleYoVariable("manualTranslationOffset_X", registry);
      manualTranslationOffsetY = new DoubleYoVariable("manualTranslationOffset_Y", registry);
      manualTranslationOffsetZ = new DoubleYoVariable("manualTranslationOffset_Z", registry);
      manualRotationOffsetInRadX = new DoubleYoVariable("manualRotationOffsetInRad_X", registry);
      manualRotationOffsetInRadY = new DoubleYoVariable("manualRotationOffsetInRad_Y", registry);
      manualRotationOffsetInRadZ = new DoubleYoVariable("manualRotationOffsetInRad_Z", registry);
//      //defaultValues for testing with Atlas.
//      manualTranslationOffsetX.set(-0.11404);
//      manualTranslationOffsetY.set(0.00022);
//      manualTranslationOffsetZ.set(0.78931);
//      manualRotationOffsetInRadX.set(-0.00002);
//      manualRotationOffsetInRadY.set(0.00024);
//      manualRotationOffsetInRadZ.set(-0.00052);
   }
   
   
   public void doControl(long timestamp)
   {
      if (pelvisPoseCorrectionCommunicator != null)
      {
         pelvisReferenceFrame.update();
         checkForManualTrigger();
         checkForNewPacket();

         pelvisReferenceFrame.getTransformToParent(stateEstimatorPelvisTransformInWorld);
         outdatedPoseUpdater.putUpToDateTransformInBuffer(stateEstimatorPelvisTransformInWorld, timestamp);
         
         offsetErrorInterpolator.interpolateError(correctedPelvisPoseInPelvisReferenceFramePose);
         /////for SCS feedback
         stateEstimatorInWorldFramePose.setPose(stateEstimatorPelvisTransformInWorld);
         yoStateEstimatorInWorldFramePose.set(stateEstimatorInWorldFramePose);
         yoCorrectedPelvisPoseInPelvisReferenceFramePose.set(correctedPelvisPoseInPelvisReferenceFramePose);
         /////
         
         updateCorrectedPelvis();
         pelvisReferenceFrame.update();
         checkForNeedToSendCorrectionUpdate();
      }
   }

   private void updateCorrectedPelvis()
   {
      correctedPelvisPoseInPelvisReferenceFramePose.changeFrame(worldFrame);
      ////// for SCS feedback
      yoCorrectedPelvisPoseInWorldFrame.set(correctedPelvisPoseInPelvisReferenceFramePose);
      //////
      correctedPelvisPoseInPelvisReferenceFramePose.getPose(correctedPelvisTransformInWorldFrame);
      correctedPelvisPoseInPelvisReferenceFramePose.changeFrame(pelvisReferenceFrame);
      
      rootJoint.setPositionAndRotation(correctedPelvisTransformInWorldFrame);
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
    * pulls the corrected pose from the buffer, check that the nonprocessed buffer has
    * corresponding pelvis poses and calculates the total error
    */
   private void processNewPacket()
   {
      StampedPosePacket newPacket = pelvisPoseCorrectionCommunicator.getNewExternalPose();
      TimeStampedTransform3D timeStampedExternalPose = newPacket.getTransform();

      if (outdatedPoseUpdater.upToDateTimeStampedBufferIsInRange(timeStampedExternalPose.getTimeStamp()))
      {
         double confidence = newPacket.getConfidenceFactor();
         confidence = MathTools.clipToMinMax(confidence, 0.0, 1.0);
         confidenceFactor.set(confidence);
         addNewExternalPose(timeStampedExternalPose);
      }
   }

   private void addNewExternalPose(TimeStampedTransform3D timeStampedExternalPose)
   {
      iterativeClosestPointTransformInWorldFrame.set(timeStampedExternalPose.getTransform3D());
      ////for SCS feedback
      iterativeClosestPointInWorldFramePose.setPose(iterativeClosestPointTransformInWorldFrame);
      yoIterativeClosestPointPoseInWorldFrame.set(iterativeClosestPointInWorldFramePose);
      ////
      outdatedPoseUpdater.updateOutdatedTransform(timeStampedExternalPose, iterativeClosestPointInPelvisReferenceFramePose);
      iterativeClosestPointInPelvisReferenceFramePose.getPose(errorTransformBetweenCurrentPositionAndCorrected);
      
      yoIterativeClosestPointInPelvisReferenceFramePose.set(iterativeClosestPointInPelvisReferenceFramePose);
      offsetErrorInterpolator.setInterpolatorInputs(correctedPelvisPoseInPelvisReferenceFramePose, iterativeClosestPointInPelvisReferenceFramePose, confidenceFactor.getDoubleValue());
   }
   
   private void checkForNeedToSendCorrectionUpdate()
   {
      if (sendCorrectionUpdate)
      {
         sendCorrectionUpdatePacket();
         sendCorrectionUpdate = false;
      }
   }
   
   private void sendCorrectionUpdatePacket()
   {
      PelvisPoseErrorPacket pelvisPoseErrorPacket = new PelvisPoseErrorPacket(iterativeClosestPointTransformInWorldFrame, errorTransformBetweenCurrentPositionAndCorrected);
      pelvisPoseCorrectionCommunicator.sendPelvisPoseErrorPacket(pelvisPoseErrorPacket);
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
   
   public void manuallyTriggerLocalizationUpdate()
   {
      confidenceFactor.set(1.0);

      RigidBodyTransform pelvisPose = new RigidBodyTransform();

      Quat4d rotation = new Quat4d();
      pelvisPose.getRotation(rotation);
      RotationFunctions.setQuaternionBasedOnYawPitchRoll(rotation, manualRotationOffsetInRadZ.getDoubleValue(), manualRotationOffsetInRadY.getDoubleValue(),
            manualRotationOffsetInRadX.getDoubleValue());
      pelvisPose.setRotation(rotation);

      Vector3d translation = new Vector3d();
      pelvisPose.get(translation);
      translation.setX(manualTranslationOffsetX.getDoubleValue());
      translation.setY(manualTranslationOffsetY.getDoubleValue());
      translation.setZ(manualTranslationOffsetZ.getDoubleValue());
      pelvisPose.setTranslation(translation);

      TimeStampedTransform3D manualTimeStampedTransform3D = new TimeStampedTransform3D(pelvisPose, (long) (outdatedPoseUpdater.getUpToDateTimeStampedBufferNewestTimestamp() - pelvisBufferSize.getIntegerValue() / 10));
      addNewExternalPose(manualTimeStampedTransform3D);
      manuallyTriggerLocalizationUpdate.set(false);
   }
   
   public void setExternalPelvisCorrectorSubscriber(PelvisPoseCorrectionCommunicatorInterface externalPelvisPoseSubscriber)
   {
      this.pelvisPoseCorrectionCommunicator = externalPelvisPoseSubscriber;
   }
}
