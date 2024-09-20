package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import controller_msgs.msg.dds.PelvisPoseErrorPacket;
import ihmc_common_msgs.msg.dds.StampedPosePacket;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.rotationConversion.YawPitchRollConversion;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class NewPelvisPoseHistoryCorrection implements PelvisPoseHistoryCorrectionInterface
{
   private final YoBoolean enableProcessNewPackets;

   private static final boolean ENABLE_GRAPHICS = true;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry;

   private PelvisPoseCorrectionCommunicatorInterface pelvisPoseCorrectionCommunicator;

   private final FloatingJointBasics rootJoint;
   private final ReferenceFrame pelvisReferenceFrame;

   private final CorrectedPelvisPoseErrorTooBigChecker correctedPelvisPoseErrorTooBigChecker;
   private final ClippedSpeedOffsetErrorInterpolator offsetErrorInterpolator;
   private final OutdatedPoseToUpToDateReferenceFrameUpdater outdatedPoseUpdater;

   private boolean hasMapBeenReset = false;

   private final double estimatorDT;
   private boolean sendCorrectionUpdate = false;

   private final YoDouble confidenceFactor;

   private final RigidBodyTransform stateEstimatorPelvisTransformInWorld = new RigidBodyTransform();
   private final RigidBodyTransform localizationTransformInWorld = new RigidBodyTransform();
   private final RigidBodyTransform correctedPelvisTransformInWorldFrame = new RigidBodyTransform();

   private final RigidBodyTransform totalErrorBetweenPelvisAndLocalizationTransform = new RigidBodyTransform();
   private final RigidBodyTransform errorBetweenCorrectedAndLocalizationTransform = new RigidBodyTransform();
   private final Vector3D totalErrorTranslation = new Vector3D();
   private final Quaternion totalErrorRotation = new Quaternion();
   private final YoDouble totalErrorTranslation_X;
   private final YoDouble totalErrorTranslation_Y;
   private final YoDouble totalErrorTranslation_Z;
   private final YawPitchRoll totalErrorYawPitchRoll = new YawPitchRoll();
   private final YoDouble totalErrorRotation_Yaw;
   private final YoDouble totalErrorRotation_Pitch;
   private final YoDouble totalErrorRotation_Roll;

   private final YoInteger pelvisBufferSize;

   private final FramePose3D stateEstimatorInWorldFramePose = new FramePose3D(worldFrame);
   private final YoFramePoseUsingYawPitchRoll yoStateEstimatorInWorldFramePose;
   private final YoFramePoseUsingYawPitchRoll yoCorrectedPelvisPoseInWorldFrame;
   private final FramePose3D iterativeClosestPointInWorldFramePose = new FramePose3D(worldFrame);
   private final YoFramePoseUsingYawPitchRoll yoIterativeClosestPointPoseInWorldFrame;

   private final ReferenceFrame iterativeClosestPointReferenceFrame;
   private final FramePose3D correctedPelvisPoseInWorldFrame = new FramePose3D(worldFrame);

   private final YoBoolean hasOneIcpPacketEverBeenReceived;

   private final Vector3D localizationTranslation = new Vector3D();
   private final Vector3D correctedPelvisTranslation = new Vector3D();
   private final Vector3D errorBetweenCorrectedAndLocalizationTransform_Translation = new Vector3D();

   private final FrameQuaternion localizationOrientation = new FrameQuaternion(worldFrame);
   private final FrameQuaternion correctedPelvisOrientation = new FrameQuaternion(worldFrame);
   private final FrameQuaternion errorBetweenCorrectedAndLocalizationTransform_Rotation = new FrameQuaternion(worldFrame);
   private final Quaternion errorBetweenCorrectedAndLocalizationQuaternion_Rotation = new Quaternion();

   private final YoBoolean isErrorTooBig;
   private final TimeStampedTransform3D timeStampedTransform3DToPack = new TimeStampedTransform3D();

   public NewPelvisPoseHistoryCorrection(FullInverseDynamicsStructure inverseDynamicsStructure, final double dt, YoRegistry parentRegistry,
                                         YoGraphicsListRegistry yoGraphicsListRegistry, int pelvisBufferSize,
                                         ClippedSpeedOffsetErrorInterpolatorParameters parameters)
   {
      this(inverseDynamicsStructure.getRootJoint(), dt, parentRegistry, pelvisBufferSize, yoGraphicsListRegistry, null, parameters);
   }

   public NewPelvisPoseHistoryCorrection(FullInverseDynamicsStructure inverseDynamicsStructure,
                                         PelvisPoseCorrectionCommunicatorInterface externalPelvisPoseSubscriber, final double dt,
                                         YoRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry, int pelvisBufferSize,
                                         ClippedSpeedOffsetErrorInterpolatorParameters parameters)
   {
      this(inverseDynamicsStructure.getRootJoint(), dt, parentRegistry, pelvisBufferSize, yoGraphicsListRegistry, externalPelvisPoseSubscriber, parameters);
   }

   public NewPelvisPoseHistoryCorrection(FloatingJointBasics sixDofJoint, final double estimatorDT, YoRegistry parentRegistry, int pelvisBufferSize,
                                         YoGraphicsListRegistry yoGraphicsListRegistry, PelvisPoseCorrectionCommunicatorInterface externalPelvisPoseSubscriber,
                                         ClippedSpeedOffsetErrorInterpolatorParameters parameters)
   {
      this.estimatorDT = estimatorDT;

      this.rootJoint = sixDofJoint;
      this.pelvisReferenceFrame = rootJoint.getFrameAfterJoint();
      this.pelvisPoseCorrectionCommunicator = externalPelvisPoseSubscriber;
      this.registry = new YoRegistry("newPelvisPoseHistoryCorrection");

      // TODO do not merge, temporarily adding to reduce yovariables
//      parentRegistry.addChild(registry);

      enableProcessNewPackets = new YoBoolean("enableProcessNewPackets", registry);
      enableProcessNewPackets.set(true);

      this.pelvisBufferSize = new YoInteger("pelvisBufferSize", registry);
      this.pelvisBufferSize.set(pelvisBufferSize);

      confidenceFactor = new YoDouble("PelvisErrorCorrectionConfidenceFactor", registry);

      correctedPelvisPoseErrorTooBigChecker = new CorrectedPelvisPoseErrorTooBigChecker(registry);

      offsetErrorInterpolator = new ClippedSpeedOffsetErrorInterpolator(registry, pelvisReferenceFrame, this.estimatorDT, parameters);

      outdatedPoseUpdater = new OutdatedPoseToUpToDateReferenceFrameUpdater(pelvisBufferSize, pelvisReferenceFrame);

      iterativeClosestPointReferenceFrame = outdatedPoseUpdater.getLocalizationReferenceFrameToBeUpdated();

      //used only for feedback in SCS
      yoStateEstimatorInWorldFramePose = new YoFramePoseUsingYawPitchRoll("stateEstimatorInWorldFramePose", worldFrame, registry);
      yoCorrectedPelvisPoseInWorldFrame = new YoFramePoseUsingYawPitchRoll("correctedPelvisPoseInWorldFrame", worldFrame, registry);
      yoIterativeClosestPointPoseInWorldFrame = new YoFramePoseUsingYawPitchRoll("iterativeClosestPointPoseInWorldFrame", worldFrame, registry);

      totalErrorTranslation_X = new YoDouble("totalErrorTranslation_X", registry);
      totalErrorTranslation_Y = new YoDouble("totalErrorTranslation_Y", registry);
      totalErrorTranslation_Z = new YoDouble("totalErrorTranslation_Z", registry);
      totalErrorRotation_Yaw = new YoDouble("totalErrorRotation_Yaw", registry);
      totalErrorRotation_Pitch = new YoDouble("totalErrorRotation_Pitch", registry);
      totalErrorRotation_Roll = new YoDouble("totalErrorRotation_Roll", registry);

      hasOneIcpPacketEverBeenReceived = new YoBoolean("hasOneIcpPacketEverBeenReceived", registry);
      hasOneIcpPacketEverBeenReceived.set(false);

      isErrorTooBig = new YoBoolean("isErrorTooBig", registry);
      isErrorTooBig.set(false);

      if (ENABLE_GRAPHICS && yoGraphicsListRegistry != null)
      {
         YoGraphicCoordinateSystem yoCorrectedPelvisPoseInWorldFrameGraphic = new YoGraphicCoordinateSystem("yoCorrectedPelvisPoseInWorldFrameGraphic",
                                                                                                            yoCorrectedPelvisPoseInWorldFrame,
                                                                                                            0.1,
                                                                                                            YoAppearance.Yellow());
         yoGraphicsListRegistry.registerYoGraphic("yoCorrectedPelvisPoseInWorldFrame", yoCorrectedPelvisPoseInWorldFrameGraphic);

         YoGraphicCoordinateSystem yoIterativeClosestPointPoseInWorldFrameGraphic = new YoGraphicCoordinateSystem("yoIterativeClosestPointPoseInWorldFrameGraphic",
                                                                                                                  yoIterativeClosestPointPoseInWorldFrame,
                                                                                                                  0.1,
                                                                                                                  YoAppearance.Red());
         yoGraphicsListRegistry.registerYoGraphic("yoIterativeClosestPointPoseInWorldFrameGraphic", yoIterativeClosestPointPoseInWorldFrameGraphic);

         YoGraphicCoordinateSystem yoStateEstimatorInWorldFramePoseGraphic = new YoGraphicCoordinateSystem("yoStateEstimatorInWorldFramePoseGraphic",
                                                                                                           yoStateEstimatorInWorldFramePose,
                                                                                                           0.1,
                                                                                                           YoAppearance.Gray());
         yoGraphicsListRegistry.registerYoGraphic("yoCorrectedPelvisPoseInWorldFrame", yoStateEstimatorInWorldFramePoseGraphic);
      }
   }

   @Override
   public void doControl(long timestamp)
   {
      if (pelvisPoseCorrectionCommunicator != null)
      {
         pelvisReferenceFrame.update();
         checkForNewPacket();

         pelvisReferenceFrame.getTransformToDesiredFrame(stateEstimatorPelvisTransformInWorld, worldFrame);
         outdatedPoseUpdater.putStateEstimatorTransformInBuffer(stateEstimatorPelvisTransformInWorld, timestamp);

         offsetErrorInterpolator.interpolateError(correctedPelvisPoseInWorldFrame);
         /////for SCS feedback
         stateEstimatorInWorldFramePose.set(stateEstimatorPelvisTransformInWorld);
         yoStateEstimatorInWorldFramePose.set(stateEstimatorInWorldFramePose);
         /////

         updateCorrectedPelvis();
         pelvisReferenceFrame.update();
         checkForNeedToSendCorrectionUpdate();
      }
   }

   private void updateCorrectedPelvis()
   {
      if (!hasOneIcpPacketEverBeenReceived.getBooleanValue())
         correctedPelvisPoseInWorldFrame.set(stateEstimatorPelvisTransformInWorld);

      correctedPelvisPoseInWorldFrame.get(correctedPelvisTransformInWorldFrame);

      correctedPelvisTranslation.set(correctedPelvisTransformInWorldFrame.getTranslation());
      localizationTranslation.set(iterativeClosestPointInWorldFramePose.getPosition());

      localizationOrientation.setIncludingFrame(iterativeClosestPointInWorldFramePose.getOrientation());
      correctedPelvisOrientation.setIncludingFrame(worldFrame, correctedPelvisTransformInWorldFrame.getRotation());

      errorBetweenCorrectedAndLocalizationTransform_Rotation.difference(correctedPelvisOrientation, localizationOrientation);
      errorBetweenCorrectedAndLocalizationQuaternion_Rotation.set(errorBetweenCorrectedAndLocalizationTransform_Rotation);

      errorBetweenCorrectedAndLocalizationTransform_Translation.sub(localizationTranslation, correctedPelvisTranslation);

      ////// for SCS feedback
      yoCorrectedPelvisPoseInWorldFrame.set(correctedPelvisPoseInWorldFrame);
      //////
      errorBetweenCorrectedAndLocalizationTransform.getTranslation().set(errorBetweenCorrectedAndLocalizationTransform_Translation);
      errorBetweenCorrectedAndLocalizationTransform.getRotation().set(errorBetweenCorrectedAndLocalizationQuaternion_Rotation);

      rootJoint.setJointConfiguration(correctedPelvisTransformInWorldFrame);
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

   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final Vector3D tempTranslation = new Vector3D();
   private final Quaternion tempRotation = new Quaternion();
   private final TimeStampedTransform3D timeStampedExternalPose = new TimeStampedTransform3D();

   /**
    * pulls the corrected pose from the buffer, check that the nonprocessed buffer has corresponding
    * pelvis poses and calculates the total error
    */
   private void processNewPacket()
   {
      StampedPosePacket newPacket = pelvisPoseCorrectionCommunicator.getNewExternalPose();
      if (enableProcessNewPackets.getBooleanValue())
      {
         timeStampedExternalPose.setTransform3D(newPacket.getPose());
         timeStampedExternalPose.setTimeStamp(newPacket.getTimestamp());

         if (outdatedPoseUpdater.stateEstimatorTimeStampedBufferIsInRange(timeStampedExternalPose.getTimeStamp()))
         {
            outdatedPoseUpdater.getStateEstimatorTransform(timeStampedExternalPose.getTimeStamp(), timeStampedTransform3DToPack);
            RigidBodyTransform stateEstimatorPose = timeStampedTransform3DToPack.getTransform3D();
            RigidBodyTransform localizationPose = timeStampedExternalPose.getTransform3D();

            // Get difference between the localization pose and the state estimator pose, then check for "deadband"
            tempTransform.set(stateEstimatorPose);
            tempTransform.invert();
            tempTransform.multiply(localizationPose);
            tempTranslation.set(tempTransform.getTranslation());
            tempRotation.set(tempTransform.getRotation());

            // If we are in the deadband just return

            if (offsetErrorInterpolator.isWithinDeadband(tempTranslation, tempRotation))
            {
               return;
            }

            if (!hasOneIcpPacketEverBeenReceived.getBooleanValue())
               hasOneIcpPacketEverBeenReceived.set(true);
            double confidence = newPacket.getConfidenceFactor();
            confidence = MathTools.clamp(confidence, 0.0, 1.0);
            confidenceFactor.set(confidence);
            addNewExternalPose(timeStampedExternalPose);
         }
         else
         {
            System.err.println("Error in NewPelvisPoseHistoryCorrection: pelvisPoseBuffer is out of range.");
            System.err.println("timeStampedExternalPose.getTimeStamp() = " + timeStampedExternalPose.getTimeStamp());
            System.err.println("consider increasing the size of the buffer");
         }
      }
   }

   private void addNewExternalPose(TimeStampedTransform3D timeStampedExternalPose)
   {
      outdatedPoseUpdater.updateLocalizationTransform(timeStampedExternalPose);
      iterativeClosestPointReferenceFrame.update();
      iterativeClosestPointInWorldFramePose.setToZero(iterativeClosestPointReferenceFrame);
      iterativeClosestPointInWorldFramePose.changeFrame(worldFrame);
      iterativeClosestPointInWorldFramePose.get(localizationTransformInWorld);

      //for feedback in the UI
      outdatedPoseUpdater.getTotalErrorTransform(totalErrorBetweenPelvisAndLocalizationTransform);

      ////for SCS feedback
      yoIterativeClosestPointPoseInWorldFrame.set(iterativeClosestPointInWorldFramePose);
      totalErrorTranslation.set(totalErrorBetweenPelvisAndLocalizationTransform.getTranslation());
      totalErrorTranslation_X.set(totalErrorTranslation.getX());
      totalErrorTranslation_Y.set(totalErrorTranslation.getY());
      totalErrorTranslation_Z.set(totalErrorTranslation.getZ());
      totalErrorRotation.set(totalErrorBetweenPelvisAndLocalizationTransform.getRotation());
      YawPitchRollConversion.convertQuaternionToYawPitchRoll(totalErrorRotation, totalErrorYawPitchRoll);
      totalErrorRotation_Yaw.set(totalErrorYawPitchRoll.getYaw());
      totalErrorRotation_Pitch.set(totalErrorYawPitchRoll.getPitch());
      totalErrorRotation_Roll.set(totalErrorYawPitchRoll.getRoll());
      /////

      if (correctedPelvisPoseErrorTooBigChecker.checkIfErrorIsTooBig(correctedPelvisPoseInWorldFrame, iterativeClosestPointInWorldFramePose, true))
      {
         requestLocalizationReset();
         isErrorTooBig.set(true);
      }
      else
      {
         offsetErrorInterpolator.setInterpolatorInputs(correctedPelvisPoseInWorldFrame,
                                                       iterativeClosestPointInWorldFramePose,
                                                       confidenceFactor.getDoubleValue());
         isErrorTooBig.set(false);
      }
   }

   private void requestLocalizationReset()
   {
      pelvisPoseCorrectionCommunicator.sendLocalizationResetRequest(HumanoidMessageTools.createLocalizationPacket(true, true));
      hasMapBeenReset = true;
   }

   private void checkForNeedToSendCorrectionUpdate()
   {
      if (sendCorrectionUpdate)
      {
         sendCorrectionUpdatePacket();
         sendCorrectionUpdate = false;
      }
   }

   Vector3D translationalResidualError = new Vector3D();
   Vector3D translationalTotalError = new Vector3D();

   private void sendCorrectionUpdatePacket()
   {
      translationalResidualError.set(errorBetweenCorrectedAndLocalizationTransform.getTranslation());
      translationalTotalError.set(totalErrorBetweenPelvisAndLocalizationTransform.getTranslation());

      double absoluteResidualError = translationalResidualError.length();
      double absoluteTotalError = translationalTotalError.length();

      PelvisPoseErrorPacket pelvisPoseErrorPacket = HumanoidMessageTools.createPelvisPoseErrorPacket((float) absoluteResidualError,
                                                                                                     (float) absoluteTotalError,
                                                                                                     hasMapBeenReset);
      hasMapBeenReset = false;
      pelvisPoseCorrectionCommunicator.sendPelvisPoseErrorPacket(pelvisPoseErrorPacket);
   }

   public void setExternalPelvisCorrectorSubscriber(PelvisPoseCorrectionCommunicatorInterface externalPelvisPoseSubscriber)
   {
      this.pelvisPoseCorrectionCommunicator = externalPelvisPoseSubscriber;
   }
}
