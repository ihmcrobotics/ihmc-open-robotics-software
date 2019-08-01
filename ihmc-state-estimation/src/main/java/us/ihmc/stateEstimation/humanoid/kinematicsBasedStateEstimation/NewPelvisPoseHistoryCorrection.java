package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import controller_msgs.msg.dds.PelvisPoseErrorPacket;
import controller_msgs.msg.dds.StampedPosePacket;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.rotationConversion.YawPitchRollConversion;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.variable.YoInteger;

public class NewPelvisPoseHistoryCorrection implements PelvisPoseHistoryCorrectionInterface
{
   private static final boolean DEFAULT_INTERPOLATE_AND_FILTER_ERROR = true;

   //TODO: Put parameters in a parameters class.
   private static final double Z_DEADZONE_SIZE = 0.014;
   private static final double Y_DEADZONE_SIZE = 0.014;
   private static final double X_DEADZONE_SIZE = 0.014;
   private static final double YAW_DEADZONE_IN_DEGREES = 1.0;
   private static final double DEFAULT_BREAK_FREQUENCY = 0.6;
   private static final boolean ENABLE_ROTATION_CORRECTION = true;
   
   private final YoBoolean enableProcessNewPackets;
   
   private static final boolean ENABLE_GRAPHICS = true;
   
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry;
   
   private PelvisPoseCorrectionCommunicatorInterface pelvisPoseCorrectionCommunicator;
   
   private final FloatingJointBasics rootJoint;
   private final ReferenceFrame pelvisReferenceFrame;
   
   private final CorrectedPelvisPoseErrorTooBigChecker correctedPelvisPoseErrorTooBigChecker;
   private final ClippedSpeedOffsetErrorInterpolator offsetErrorInterpolator;
   private final OutdatedPoseToUpToDateReferenceFrameUpdater outdatedPoseUpdater;
   
   private boolean hasMapBeenReset = false;
   
   private final double estimatorDT;
   private boolean sendCorrectionUpdate = false;
   
   //Deadzone translation variables
   private final YoDouble xDeadzoneSize;
   private final YoDouble yDeadzoneSize;
   private final YoDouble zDeadzoneSize;
   private final YoDouble yawDeadzoneSize;

   private final YoDouble alphaFilterBreakFrequency;
   
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
   private final double[] totalErrorYawPitchRoll = new double[3];
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
   
   public NewPelvisPoseHistoryCorrection(FullInverseDynamicsStructure inverseDynamicsStructure, final double dt, YoVariableRegistry parentRegistry,
         YoGraphicsListRegistry yoGraphicsListRegistry, int pelvisBufferSize)
   {
      this(inverseDynamicsStructure.getRootJoint(), dt, parentRegistry, pelvisBufferSize, yoGraphicsListRegistry, null);
   }

   public NewPelvisPoseHistoryCorrection(FullInverseDynamicsStructure inverseDynamicsStructure,
         PelvisPoseCorrectionCommunicatorInterface externalPelvisPoseSubscriber, final double dt, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry, int pelvisBufferSize)
   {
      this(inverseDynamicsStructure.getRootJoint(), dt, parentRegistry, pelvisBufferSize, yoGraphicsListRegistry, externalPelvisPoseSubscriber);
   }
   
   public NewPelvisPoseHistoryCorrection(FloatingJointBasics sixDofJoint, final double estimatorDT, YoVariableRegistry parentRegistry, int pelvisBufferSize,
         YoGraphicsListRegistry yoGraphicsListRegistry, PelvisPoseCorrectionCommunicatorInterface externalPelvisPoseSubscriber)
   {
      
      this.estimatorDT = estimatorDT;

      this.rootJoint = sixDofJoint;
      this.pelvisReferenceFrame = rootJoint.getFrameAfterJoint();
      this.pelvisPoseCorrectionCommunicator = externalPelvisPoseSubscriber;
      this.registry = new YoVariableRegistry("newPelvisPoseHistoryCorrection");
      parentRegistry.addChild(registry);
      
      xDeadzoneSize = new YoDouble("xDeadzoneSize", registry);
      xDeadzoneSize.set(X_DEADZONE_SIZE);
      yDeadzoneSize = new YoDouble("yDeadzoneSize", registry);
      yDeadzoneSize.set(Y_DEADZONE_SIZE);
      zDeadzoneSize = new YoDouble("zDeadzoneSize", registry);
      zDeadzoneSize.set(Z_DEADZONE_SIZE);
      yawDeadzoneSize = new YoDouble("yawDeadzoneSize", registry);
      yawDeadzoneSize.set(Math.toRadians(YAW_DEADZONE_IN_DEGREES));
      
      enableProcessNewPackets = new YoBoolean("enableProcessNewPackets", registry);
      enableProcessNewPackets.set(true);
      
      this.pelvisBufferSize = new YoInteger("pelvisBufferSize", registry);
      this.pelvisBufferSize.set(pelvisBufferSize);
      
      alphaFilterBreakFrequency = new YoDouble("alphaFilterBreakFrequency", registry);
      alphaFilterBreakFrequency.set(DEFAULT_BREAK_FREQUENCY);
      
      confidenceFactor = new YoDouble("PelvisErrorCorrectionConfidenceFactor", registry);

      correctedPelvisPoseErrorTooBigChecker = new CorrectedPelvisPoseErrorTooBigChecker(registry);
      offsetErrorInterpolator = new ClippedSpeedOffsetErrorInterpolator(registry, pelvisReferenceFrame, alphaFilterBreakFrequency, this.estimatorDT, ENABLE_ROTATION_CORRECTION);
     
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
      
      if(ENABLE_GRAPHICS && yoGraphicsListRegistry != null)
      {
         YoGraphicCoordinateSystem yoCorrectedPelvisPoseInWorldFrameGraphic = new YoGraphicCoordinateSystem("yoCorrectedPelvisPoseInWorldFrameGraphic", yoCorrectedPelvisPoseInWorldFrame, 0.1, YoAppearance.Yellow());
         yoGraphicsListRegistry.registerYoGraphic("yoCorrectedPelvisPoseInWorldFrame", yoCorrectedPelvisPoseInWorldFrameGraphic);
         
         YoGraphicCoordinateSystem yoIterativeClosestPointPoseInWorldFrameGraphic = new YoGraphicCoordinateSystem("yoIterativeClosestPointPoseInWorldFrameGraphic", yoIterativeClosestPointPoseInWorldFrame, 0.1, YoAppearance.Red());
         yoGraphicsListRegistry.registerYoGraphic("yoIterativeClosestPointPoseInWorldFrameGraphic", yoIterativeClosestPointPoseInWorldFrameGraphic);
         
         YoGraphicCoordinateSystem yoStateEstimatorInWorldFramePoseGraphic = new YoGraphicCoordinateSystem("yoStateEstimatorInWorldFramePoseGraphic", yoStateEstimatorInWorldFramePose, 0.1, YoAppearance.Gray());
         yoGraphicsListRegistry.registerYoGraphic("yoCorrectedPelvisPoseInWorldFrame", yoStateEstimatorInWorldFramePoseGraphic);
      }
   }
   
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
      if(!hasOneIcpPacketEverBeenReceived.getBooleanValue())
         correctedPelvisPoseInWorldFrame.set(stateEstimatorPelvisTransformInWorld);
      
      correctedPelvisPoseInWorldFrame.get(correctedPelvisTransformInWorldFrame);
      
      correctedPelvisTransformInWorldFrame.getTranslation(correctedPelvisTranslation);
      localizationTranslation.set(iterativeClosestPointInWorldFramePose.getPosition());
      
      localizationOrientation.setIncludingFrame(iterativeClosestPointInWorldFramePose.getOrientation());
      correctedPelvisOrientation.setIncludingFrame(worldFrame, correctedPelvisTransformInWorldFrame.getRotationMatrix());
      
      errorBetweenCorrectedAndLocalizationTransform_Rotation.difference(correctedPelvisOrientation, localizationOrientation);
      errorBetweenCorrectedAndLocalizationQuaternion_Rotation.set(errorBetweenCorrectedAndLocalizationTransform_Rotation);

      errorBetweenCorrectedAndLocalizationTransform_Translation.sub(localizationTranslation, correctedPelvisTranslation);
      
      ////// for SCS feedback
      yoCorrectedPelvisPoseInWorldFrame.set(correctedPelvisPoseInWorldFrame);
      //////
      errorBetweenCorrectedAndLocalizationTransform.setTranslation(errorBetweenCorrectedAndLocalizationTransform_Translation);
      errorBetweenCorrectedAndLocalizationTransform.setRotation(errorBetweenCorrectedAndLocalizationQuaternion_Rotation);
      
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
    * pulls the corrected pose from the buffer, check that the nonprocessed buffer has
    * corresponding pelvis poses and calculates the total error
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
            
            //get difference between the localization pose and the state estimator pose, then check for "deadband"
            tempTransform.set(stateEstimatorPose);
            tempTransform.invert();
            tempTransform.multiply(localizationPose);
            tempTransform.getTranslation(tempTranslation);
            tempTransform.getRotation(tempRotation);
            
            //if we are in the deadband just return
            boolean withinXDeadband = Math.abs(tempTranslation.getX()) < xDeadzoneSize.getDoubleValue();
            boolean withinYDeadBand = Math.abs(tempTranslation.getY()) < yDeadzoneSize.getDoubleValue();
            boolean withinZDeadband = Math.abs(tempTranslation.getZ()) < zDeadzoneSize.getDoubleValue();
            boolean withinYawDeadband = Math.abs(tempRotation.getYaw()) < yawDeadzoneSize.getDoubleValue();
            if(withinXDeadband && withinYDeadBand && withinZDeadband && withinYawDeadband)
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
      totalErrorBetweenPelvisAndLocalizationTransform.getTranslation(totalErrorTranslation);
      totalErrorTranslation_X.set(totalErrorTranslation.getX());
      totalErrorTranslation_Y.set(totalErrorTranslation.getY());
      totalErrorTranslation_Z.set(totalErrorTranslation.getZ());
      totalErrorBetweenPelvisAndLocalizationTransform.getRotation(totalErrorRotation);
      YawPitchRollConversion.convertQuaternionToYawPitchRoll(totalErrorRotation, totalErrorYawPitchRoll);
      totalErrorRotation_Yaw.set(totalErrorYawPitchRoll[0]);
      totalErrorRotation_Pitch.set(totalErrorYawPitchRoll[1]);
      totalErrorRotation_Roll.set(totalErrorYawPitchRoll[2]);
      /////

      if(correctedPelvisPoseErrorTooBigChecker.checkIfErrorIsTooBig(correctedPelvisPoseInWorldFrame, iterativeClosestPointInWorldFramePose, true))
      {
         requestLocalizationReset();
         isErrorTooBig.set(true);
      }
      else
      {
         offsetErrorInterpolator.setInterpolatorInputs(correctedPelvisPoseInWorldFrame, iterativeClosestPointInWorldFramePose, confidenceFactor.getDoubleValue());
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
      errorBetweenCorrectedAndLocalizationTransform.getTranslation(translationalResidualError);
      totalErrorBetweenPelvisAndLocalizationTransform.getTranslation(translationalTotalError);
      
      double absoluteResidualError = translationalResidualError.length();
      double absoluteTotalError = translationalTotalError.length();

      PelvisPoseErrorPacket pelvisPoseErrorPacket = HumanoidMessageTools.createPelvisPoseErrorPacket((float) absoluteResidualError, (float) absoluteTotalError, hasMapBeenReset);
      hasMapBeenReset = false;
      pelvisPoseCorrectionCommunicator.sendPelvisPoseErrorPacket(pelvisPoseErrorPacket);
   }
   
   public void setExternalPelvisCorrectorSubscriber(PelvisPoseCorrectionCommunicatorInterface externalPelvisPoseSubscriber)
   {
      this.pelvisPoseCorrectionCommunicator = externalPelvisPoseSubscriber;
   }
}
