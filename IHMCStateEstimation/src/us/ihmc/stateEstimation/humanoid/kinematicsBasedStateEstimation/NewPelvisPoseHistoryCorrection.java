package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.StampedPosePacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.LocalizationPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.PelvisPoseErrorPacket;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.geometry.TransformTools;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;

public class NewPelvisPoseHistoryCorrection implements PelvisPoseHistoryCorrectionInterface
{
   
   private static final double Z_DEADZONE_SIZE = 0.014;
   private static final double Y_DEADZONE_SIZE = 0.014;
   private static final double X_DEADZONE_SIZE = 0.014;
   private static final double YAW_DEADZONE_IN_DEGREES = 1.0;
   
   private final BooleanYoVariable enableProcessNewPackets;
   
   private static final boolean ENABLE_GRAPHICS = true;
   
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final boolean ENABLE_ROTATION_CORRECTION = true;
   
   private static final double DEFAULT_BREAK_FREQUENCY = 0.6;

   private final YoVariableRegistry registry;
   
   private PelvisPoseCorrectionCommunicatorInterface pelvisPoseCorrectionCommunicator;
   
   private final FloatingInverseDynamicsJoint rootJoint;
   private final ReferenceFrame pelvisReferenceFrame;
   private final ClippedSpeedOffsetErrorInterpolator offsetErrorInterpolator;
   private final OutdatedPoseToUpToDateReferenceFrameUpdater outdatedPoseUpdater;
   
   private boolean hasMapBeenReset = false;
   
   private final double estimatorDT;
   private boolean sendCorrectionUpdate = false;
   
   //Deadzone translation variables
   private final DoubleYoVariable xDeadzoneSize;
   private final DoubleYoVariable yDeadzoneSize;
   private final DoubleYoVariable zDeadzoneSize;
   private final DoubleYoVariable yawDeadzoneSize;

   private final DoubleYoVariable alphaFilterBreakFrequency;
   
   private final DoubleYoVariable confidenceFactor;
   
   private final RigidBodyTransform stateEstimatorPelvisTransformInWorld = new RigidBodyTransform();
   private final RigidBodyTransform localizationTransformInWorld = new RigidBodyTransform();
   private final RigidBodyTransform correctedPelvisTransformInWorldFrame = new RigidBodyTransform();
   
   private final RigidBodyTransform totalErrorBetweenPelvisAndLocalizationTransform = new RigidBodyTransform();
   private final RigidBodyTransform errorBetweenCorrectedAndLocalizationTransform = new RigidBodyTransform();
   private final Vector3d totalErrorTranslation = new Vector3d(); 
   private final Quat4d totalErrorRotation = new Quat4d(); 
   private final DoubleYoVariable totalErrorTranslation_X;
   private final DoubleYoVariable totalErrorTranslation_Y;
   private final DoubleYoVariable totalErrorTranslation_Z;
   private final double[] totalErrorYawPitchRoll = new double[3];
   private final DoubleYoVariable totalErrorRotation_Yaw;
   private final DoubleYoVariable totalErrorRotation_Pitch;
   private final DoubleYoVariable totalErrorRotation_Roll;
   
   private final IntegerYoVariable pelvisBufferSize;
   
   private final FramePose stateEstimatorInWorldFramePose = new FramePose(worldFrame);
   private final YoFramePose yoStateEstimatorInWorldFramePose;
   private final YoFramePose yoCorrectedPelvisPoseInWorldFrame;
   private final FramePose iterativeClosestPointInWorldFramePose = new FramePose(worldFrame);
   private final YoFramePose yoIterativeClosestPointPoseInWorldFrame;
   
   private final ReferenceFrame iterativeClosestPointReferenceFrame;
   private final FramePose correctedPelvisPoseInWorldFrame = new FramePose(worldFrame);
   
   private final BooleanYoVariable hasOneIcpPacketEverBeenReceived;
   
   private final Vector3d localizationTranslation = new Vector3d();
   private final Vector3d correctedPelvisTranslation = new Vector3d();
   private final Vector3d errorBetweenCorrectedAndLocalizationTransform_Translation = new Vector3d();
   
   private final FrameOrientation localizationOrientation = new FrameOrientation(worldFrame);
   private final FrameOrientation correctedPelvisOrientation = new FrameOrientation(worldFrame);
   private final FrameOrientation errorBetweenCorrectedAndLocalizationTransform_Rotation = new FrameOrientation(worldFrame);
   private final Quat4d errorBetweenCorrectedAndLocalizationQuaternion_Rotation = new Quat4d();
   
   private final BooleanYoVariable isErrorTooBig;
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
   
   public NewPelvisPoseHistoryCorrection(FloatingInverseDynamicsJoint sixDofJoint, final double estimatorDT, YoVariableRegistry parentRegistry, int pelvisBufferSize,
         YoGraphicsListRegistry yoGraphicsListRegistry, PelvisPoseCorrectionCommunicatorInterface externalPelvisPoseSubscriber)
   {
      
      this.estimatorDT = estimatorDT;

      this.rootJoint = sixDofJoint;
      this.pelvisReferenceFrame = rootJoint.getFrameAfterJoint();
      this.pelvisPoseCorrectionCommunicator = externalPelvisPoseSubscriber;
      this.registry = new YoVariableRegistry("newPelvisPoseHistoryCorrection");
      parentRegistry.addChild(registry);
      
      xDeadzoneSize = new DoubleYoVariable("xDeadzoneSize", registry);
      xDeadzoneSize.set(X_DEADZONE_SIZE);
      yDeadzoneSize = new DoubleYoVariable("yDeadzoneSize", registry);
      yDeadzoneSize.set(Y_DEADZONE_SIZE);
      zDeadzoneSize = new DoubleYoVariable("zDeadzoneSize", registry);
      zDeadzoneSize.set(Z_DEADZONE_SIZE);
      yawDeadzoneSize = new DoubleYoVariable("yawDeadzoneSize", registry);
      yawDeadzoneSize.set(Math.toRadians(YAW_DEADZONE_IN_DEGREES));
      
      enableProcessNewPackets = new BooleanYoVariable("enableProcessNewPackets", registry);
      enableProcessNewPackets.set(true);
      
      this.pelvisBufferSize = new IntegerYoVariable("pelvisBufferSize", registry);
      this.pelvisBufferSize.set(pelvisBufferSize);
      
      alphaFilterBreakFrequency = new DoubleYoVariable("alphaFilterBreakFrequency", registry);
      alphaFilterBreakFrequency.set(DEFAULT_BREAK_FREQUENCY);
      
      confidenceFactor = new DoubleYoVariable("PelvisErrorCorrectionConfidenceFactor", registry);
      
      offsetErrorInterpolator = new ClippedSpeedOffsetErrorInterpolator(registry, pelvisReferenceFrame, alphaFilterBreakFrequency, this.estimatorDT, ENABLE_ROTATION_CORRECTION);
      outdatedPoseUpdater = new OutdatedPoseToUpToDateReferenceFrameUpdater(pelvisBufferSize, pelvisReferenceFrame);
      
      iterativeClosestPointReferenceFrame = outdatedPoseUpdater.getLocalizationReferenceFrameToBeUpdated();
      
      //used only for feedback in SCS
      yoStateEstimatorInWorldFramePose = new YoFramePose("stateEstimatorInWorldFramePose", worldFrame, registry);
      yoCorrectedPelvisPoseInWorldFrame = new YoFramePose("correctedPelvisPoseInWorldFrame", worldFrame, registry);
      yoIterativeClosestPointPoseInWorldFrame = new YoFramePose("iterativeClosestPointPoseInWorldFrame", worldFrame, registry);
      
      totalErrorTranslation_X = new DoubleYoVariable("totalErrorTranslation_X", registry);
      totalErrorTranslation_Y = new DoubleYoVariable("totalErrorTranslation_Y", registry);
      totalErrorTranslation_Z = new DoubleYoVariable("totalErrorTranslation_Z", registry);
      totalErrorRotation_Yaw = new DoubleYoVariable("totalErrorRotation_Yaw", registry);
      totalErrorRotation_Pitch = new DoubleYoVariable("totalErrorRotation_Pitch", registry);
      totalErrorRotation_Roll = new DoubleYoVariable("totalErrorRotation_Roll", registry);
      
      hasOneIcpPacketEverBeenReceived = new BooleanYoVariable("hasOneIcpPacketEverBeenReceived", registry);
      hasOneIcpPacketEverBeenReceived.set(false);
      
      isErrorTooBig = new BooleanYoVariable("isErrorTooBig", registry);
      isErrorTooBig.set(false);
      
      if(ENABLE_GRAPHICS && yoGraphicsListRegistry != null)
      {
         YoGraphicCoordinateSystem yoCorrectedPelvisPoseInWorldFrameGraphic = new YoGraphicCoordinateSystem("yoCorrectedPelvisPoseInWorldFrameGraphic", yoCorrectedPelvisPoseInWorldFrame, 0.5, YoAppearance.Yellow());
         yoGraphicsListRegistry.registerYoGraphic("yoCorrectedPelvisPoseInWorldFrame", yoCorrectedPelvisPoseInWorldFrameGraphic);
         
         YoGraphicCoordinateSystem yoIterativeClosestPointPoseInWorldFrameGraphic = new YoGraphicCoordinateSystem("yoIterativeClosestPointPoseInWorldFrameGraphic", yoIterativeClosestPointPoseInWorldFrame, 0.5, YoAppearance.Red());
         yoGraphicsListRegistry.registerYoGraphic("yoIterativeClosestPointPoseInWorldFrameGraphic", yoIterativeClosestPointPoseInWorldFrameGraphic);
         
         YoGraphicCoordinateSystem yoStateEstimatorInWorldFramePoseGraphic = new YoGraphicCoordinateSystem("yoStateEstimatorInWorldFramePoseGraphic", yoStateEstimatorInWorldFramePose, 0.5, YoAppearance.Gray());
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
         stateEstimatorInWorldFramePose.setPose(stateEstimatorPelvisTransformInWorld);
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
         correctedPelvisPoseInWorldFrame.setPose(stateEstimatorPelvisTransformInWorld);
      
      correctedPelvisPoseInWorldFrame.getPose(correctedPelvisTransformInWorldFrame);
      
      correctedPelvisTransformInWorldFrame.getTranslation(correctedPelvisTranslation);
      iterativeClosestPointInWorldFramePose.getPosition(localizationTranslation);
      
      iterativeClosestPointInWorldFramePose.getOrientationIncludingFrame(localizationOrientation);
      correctedPelvisOrientation.setIncludingFrame(worldFrame, correctedPelvisTransformInWorldFrame);
      
      errorBetweenCorrectedAndLocalizationTransform_Rotation.setOrientationFromOneToTwo(localizationOrientation, correctedPelvisOrientation);
      errorBetweenCorrectedAndLocalizationTransform_Rotation.getQuaternion(errorBetweenCorrectedAndLocalizationQuaternion_Rotation);

      errorBetweenCorrectedAndLocalizationTransform_Translation.sub(localizationTranslation, correctedPelvisTranslation);
      
      ////// for SCS feedback
      yoCorrectedPelvisPoseInWorldFrame.set(correctedPelvisPoseInWorldFrame);
      //////
      errorBetweenCorrectedAndLocalizationTransform.setTranslation(errorBetweenCorrectedAndLocalizationTransform_Translation);
      errorBetweenCorrectedAndLocalizationTransform.setRotation(errorBetweenCorrectedAndLocalizationQuaternion_Rotation);
      
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

   private final RigidBodyTransform tempTransform = new RigidBodyTransform(); 
   private final Vector3d tempTranslation = new Vector3d();
   private final Quat4d tempRotation = new Quat4d();
   /**
    * pulls the corrected pose from the buffer, check that the nonprocessed buffer has
    * corresponding pelvis poses and calculates the total error
    */
   private void processNewPacket()
   {
      StampedPosePacket newPacket = pelvisPoseCorrectionCommunicator.getNewExternalPose();
      if (enableProcessNewPackets.getBooleanValue())
      {
         TimeStampedTransform3D timeStampedExternalPose = newPacket.getTransform();

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
            boolean withinYawDeadband = Math.abs(RotationTools.computeYaw(tempRotation)) < yawDeadzoneSize.getDoubleValue();
            if(withinXDeadband && withinYDeadBand && withinZDeadband && withinYawDeadband)
            {
               return;
            }
            
            
            if (!hasOneIcpPacketEverBeenReceived.getBooleanValue())
               hasOneIcpPacketEverBeenReceived.set(true);
            double confidence = newPacket.getConfidenceFactor();
            confidence = MathTools.clipToMinMax(confidence, 0.0, 1.0);
            confidenceFactor.set(confidence);
            addNewExternalPose(timeStampedExternalPose);
         }
         else
         {
            System.err.println("Error in NewPelvisPoseHistoryCorrection: pelvisPoseBuffer is out of range.");
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
      iterativeClosestPointInWorldFramePose.getPose(localizationTransformInWorld);

      //for feedback in the UI
      outdatedPoseUpdater.getTotalErrorTransform(totalErrorBetweenPelvisAndLocalizationTransform);
      
      ////for SCS feedback
      yoIterativeClosestPointPoseInWorldFrame.set(iterativeClosestPointInWorldFramePose);
      totalErrorBetweenPelvisAndLocalizationTransform.getTranslation(totalErrorTranslation);
      totalErrorTranslation_X.set(totalErrorTranslation.getX());
      totalErrorTranslation_Y.set(totalErrorTranslation.getY());
      totalErrorTranslation_Z.set(totalErrorTranslation.getZ());
      totalErrorBetweenPelvisAndLocalizationTransform.getRotation(totalErrorRotation);
      RotationTools.convertQuaternionToYawPitchRoll(totalErrorRotation, totalErrorYawPitchRoll);
      totalErrorRotation_Yaw.set(totalErrorYawPitchRoll[0]);
      totalErrorRotation_Pitch.set(totalErrorYawPitchRoll[1]);
      totalErrorRotation_Roll.set(totalErrorYawPitchRoll[2]);
      /////
      
      if(offsetErrorInterpolator.checkIfErrorIsTooBig(correctedPelvisPoseInWorldFrame, iterativeClosestPointInWorldFramePose, true))
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
      pelvisPoseCorrectionCommunicator.sendLocalizationResetRequest(new LocalizationPacket(true, true));
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
   
   Vector3d translationalResidualError = new Vector3d();
   Vector3d translationalTotalError = new Vector3d();
   
   private void sendCorrectionUpdatePacket()
   {
      errorBetweenCorrectedAndLocalizationTransform.getTranslation(translationalResidualError);
      totalErrorBetweenPelvisAndLocalizationTransform.getTranslation(translationalTotalError);
      
      double absoluteResidualError = translationalResidualError.length();
      double absoluteTotalError = translationalTotalError.length();

      PelvisPoseErrorPacket pelvisPoseErrorPacket = new PelvisPoseErrorPacket((float) absoluteResidualError, (float) absoluteTotalError, hasMapBeenReset);
      hasMapBeenReset = false;
      pelvisPoseCorrectionCommunicator.sendPelvisPoseErrorPacket(pelvisPoseErrorPacket);
   }
   
   public void setExternalPelvisCorrectorSubscriber(PelvisPoseCorrectionCommunicatorInterface externalPelvisPoseSubscriber)
   {
      this.pelvisPoseCorrectionCommunicator = externalPelvisPoseSubscriber;
   }
}
