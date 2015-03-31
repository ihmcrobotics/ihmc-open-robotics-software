package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import us.ihmc.communication.packets.StampedPosePacket;
import us.ihmc.communication.packets.sensing.PelvisPoseErrorPacket;
import us.ihmc.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.utilities.kinematics.TimeStampedTransform3D;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFramePose;

public class NewPelvisPoseHistoryCorrection implements PelvisPoseHistoryCorrectionInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final boolean ENABLE_ROTATION_CORRECTION = false;
   
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
   
   private final FramePose iterativeClosestPointInPelvisReferenceFramePose;
   private final FramePose correctedPelvisPoseInPelvisReferenceFramePose;
   private final RigidBodyTransform stateEstimatorPelvisTransformInWorld = new RigidBodyTransform();
   private final RigidBodyTransform correctedPelvisTransformInWorldFrame = new RigidBodyTransform();
   private final RigidBodyTransform iterativeClosestPointTransformInWorldFrame = new RigidBodyTransform();
   private final RigidBodyTransform totalErrorBetweenPelvisAndLocalizationTransform = new RigidBodyTransform();
   private final RigidBodyTransform errorBetweenCurrentPositionAndCorrected = new RigidBodyTransform();
   private final IntegerYoVariable pelvisBufferSize;

   private final FramePose stateEstimatorInWorldFramePose = new FramePose(worldFrame);
   private final YoFramePose yoStateEstimatorInWorldFramePose;
   private final YoFramePose yoIterativeClosestPointInPelvisReferenceFramePose;
   private final YoFramePose yoCorrectedPelvisPoseInPelvisReferenceFramePose;
   private final YoFramePose yoCorrectedPelvisPoseInWorldFrame;
   private final FramePose iterativeClosestPointInWorldFramePose = new FramePose(worldFrame);
   private final YoFramePose yoIterativeClosestPointPoseInWorldFrame;
   
   private final ReferenceFrame IterativeClosestPointReferenceFrame;
   
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
      
      
      offsetErrorInterpolator = new ClippedSpeedOffsetErrorInterpolator(registry, pelvisReferenceFrame, alphaFilterBreakFrequency, this.estimatorDT, ENABLE_ROTATION_CORRECTION);
      outdatedPoseUpdater = new OutdatedPoseToUpToDateReferenceFrameUpdater(pelvisBufferSize, pelvisReferenceFrame);
      
      IterativeClosestPointReferenceFrame = outdatedPoseUpdater.getOutdatedReferenceFrameToBeUpdated();
      
      iterativeClosestPointInPelvisReferenceFramePose = new FramePose(pelvisReferenceFrame);
      correctedPelvisPoseInPelvisReferenceFramePose = new FramePose(pelvisReferenceFrame);
      
      //used only for feedback in SCS
      yoStateEstimatorInWorldFramePose = new YoFramePose("stateEstimatorInWorldFramePose", worldFrame, registry);
      yoIterativeClosestPointInPelvisReferenceFramePose = new YoFramePose("iterativeClosestPointInPelvisReferenceFramePose", pelvisReferenceFrame, registry);
      yoCorrectedPelvisPoseInPelvisReferenceFramePose = new YoFramePose("correctedPelvisPoseInPelvisReferenceFramePose", pelvisReferenceFrame, registry);
      yoCorrectedPelvisPoseInWorldFrame = new YoFramePose("correctedPelvisPoseInWorldFrame", worldFrame, registry);
      yoIterativeClosestPointPoseInWorldFrame = new YoFramePose("iterativeClosestPointPoseInWorldFrame", worldFrame, registry);
   }
   
   
   public void doControl(long timestamp)
   {
      if (pelvisPoseCorrectionCommunicator != null)
      {
         pelvisReferenceFrame.update();
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
      correctedPelvisPoseInPelvisReferenceFramePose.getPose(errorBetweenCurrentPositionAndCorrected);
      
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
      outdatedPoseUpdater.updateOutdatedTransform(timeStampedExternalPose);
      iterativeClosestPointInPelvisReferenceFramePose.setToZero(IterativeClosestPointReferenceFrame);
      iterativeClosestPointInPelvisReferenceFramePose.changeFrame(pelvisReferenceFrame);
      iterativeClosestPointInPelvisReferenceFramePose.getPose(totalErrorBetweenPelvisAndLocalizationTransform);
      
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
      PelvisPoseErrorPacket pelvisPoseErrorPacket = new PelvisPoseErrorPacket(totalErrorBetweenPelvisAndLocalizationTransform, errorBetweenCurrentPositionAndCorrected);
      pelvisPoseCorrectionCommunicator.sendPelvisPoseErrorPacket(pelvisPoseErrorPacket);
   }
   
   public void setExternalPelvisCorrectorSubscriber(PelvisPoseCorrectionCommunicatorInterface externalPelvisPoseSubscriber)
   {
      this.pelvisPoseCorrectionCommunicator = externalPelvisPoseSubscriber;
   }
}
