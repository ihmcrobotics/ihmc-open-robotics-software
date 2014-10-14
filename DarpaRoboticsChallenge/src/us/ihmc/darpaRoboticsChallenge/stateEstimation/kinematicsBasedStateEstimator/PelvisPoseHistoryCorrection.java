package us.ihmc.darpaRoboticsChallenge.stateEstimation.kinematicsBasedStateEstimator;

import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packets.StampedPosePacket;
import us.ihmc.communication.subscribers.ExternalPelvisPoseSubscriberInterface;
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
import us.ihmc.yoUtilities.math.frames.YoFrameQuaternion;


public class PelvisPoseHistoryCorrection
{
   
   private final TimeStampedPelvisPoseBuffer stateEstimatorPelvisPoseBuffer;
   private ExternalPelvisPoseSubscriberInterface externalPelvisPoseSubscriber;
   private final SixDoFJoint rootJoint;
   private final ReferenceFrame rootJointFrame;
   private final YoVariableRegistry registry;
   
   private static final double DEFAULT_BREAK_FREQUENCY = 0.015;
   
   private final TimeStampedTransform3D[] errorBuffer = new TimeStampedTransform3D[3];
   private int bufferIndex = 0;

   private final RigidBodyTransform interpolatedError = new RigidBodyTransform();
   private final RigidBodyTransform previousInterpolatedError = new RigidBodyTransform();
   private final RigidBodyTransform interpolatorStartingPosition = new RigidBodyTransform();
   private final RigidBodyTransform pelvisPose = new RigidBodyTransform();
   private final RigidBodyTransform correctedPelvisPoseWorkingArea = new RigidBodyTransform();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();

   private final AlphaFilteredYoVariable interpolationAlphaFilter;
   private final DoubleYoVariable confidenceFactor; // target for alpha filter
   private final DoubleYoVariable interpolationAlphaFilterBreakFrequency;

   private final Vector3d tempVector = new Vector3d();
   private final Quat4d rot = new Quat4d();
   private final double[] tempRots = new double[3];
   private final TransformInterpolationCalculator transformInterpolationCalculator = new TransformInterpolationCalculator();

   private final YoFramePoint externalPelvisPosition;
   private final YoFrameQuaternion externalPelvisQuaternion;
   private final DoubleYoVariable externalPelvisPitch;
   private final DoubleYoVariable externalPelvisRoll;
   private final DoubleYoVariable externalPelvisYaw;
   
   private final YoFramePoint totalErrorPelvisPosition;
   private final YoFrameQuaternion totalErrorPelvisQuaternion;
   private final DoubleYoVariable totalErrorPelvisPitch;
   private final DoubleYoVariable totalErrorPelvisRoll;
   private final DoubleYoVariable totalErrorPelvisYaw;
   
   private final YoFramePoint interpolatedPelvisErrorPosition;
   private final YoFrameQuaternion interpolatedPelvisErrorQuaternion;
   private final DoubleYoVariable interpolatedPelvisErrorPitch;
   private final DoubleYoVariable interpolatedPelvisErrorRoll;
   private final DoubleYoVariable interpolatedPelvisErrorYaw;
   
   private final YoFramePoint previousInterpolatedPelvisErrorPosition;
   private final YoFrameQuaternion previousInterpolatedPelvisErrorQuaternion;
   private final DoubleYoVariable previousInterpolatedPelvisErrorPitch;
   private final DoubleYoVariable previousInterpolatedPelvisErrorRoll;
   private final DoubleYoVariable previousInterpolatedPelvisErrorYaw;
   
   private final YoFramePoint seNonProcessedPelvisPosition;
   private final YoFrameQuaternion seNonProcessedPelvisQuaternion;
   private final DoubleYoVariable seNonProcessedPelvisPitch;
   private final DoubleYoVariable seNonProcessedPelvisRoll;
   private final DoubleYoVariable seNonProcessedPelvisYaw;
   private final LongYoVariable seNonProcessedPelvisTimeStamp;
   
   private final YoFramePoint correctedPelvisPosition;
   private final YoFrameQuaternion correctedPelvisQuaternion;
   private final DoubleYoVariable correctedPelvisPitch;
   private final DoubleYoVariable correctedPelvisRoll;
   private final DoubleYoVariable correctedPelvisYaw;

   private final DoubleYoVariable interpolationAlphaFilterAlphaValue;

   public PelvisPoseHistoryCorrection(FullInverseDynamicsStructure inverseDynamicsStructure,
         final double dt, YoVariableRegistry parentRegistry, int pelvisBufferSize)
   {
      this(inverseDynamicsStructure, null, dt, parentRegistry, pelvisBufferSize);
   }
   
   public PelvisPoseHistoryCorrection(FullInverseDynamicsStructure inverseDynamicsStructure,
         ExternalPelvisPoseSubscriberInterface externalPelvisPoseSubscriber, final double dt, YoVariableRegistry parentRegistry, int pelvisBufferSize)
   {
      this.rootJoint = inverseDynamicsStructure.getRootJoint();
      this.rootJointFrame = rootJoint.getFrameAfterJoint();
      this.externalPelvisPoseSubscriber = externalPelvisPoseSubscriber;
      this.registry = new YoVariableRegistry("PelvisPoseHistoryCorrection");
      parentRegistry.addChild(registry);
      
      for (int i = 0; i < errorBuffer.length; i++)
      {
         errorBuffer[i] = new TimeStampedTransform3D();
      }
      
      stateEstimatorPelvisPoseBuffer = new TimeStampedPelvisPoseBuffer(pelvisBufferSize);
      
      interpolationAlphaFilterAlphaValue = new DoubleYoVariable("interpolationAlphaFilterAlphaValue", registry);
      interpolationAlphaFilterBreakFrequency = new DoubleYoVariable("interpolationAlphaFilterBreakFrequency", registry);
      interpolationAlphaFilter = new AlphaFilteredYoVariable("PelvisErrorCorrectionAlphaFilter", registry, interpolationAlphaFilterAlphaValue);
      
      interpolationAlphaFilterBreakFrequency.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            double alpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequency(interpolationAlphaFilterBreakFrequency.getDoubleValue(), dt);
            interpolationAlphaFilter.setAlpha(alpha);
         }
      });
      
      interpolationAlphaFilterBreakFrequency.set(DEFAULT_BREAK_FREQUENCY);
      confidenceFactor = new DoubleYoVariable("PelvisErrorCorrectionConfidenceFactor", registry);

      externalPelvisPosition = new YoFramePoint("newExternalPelvis_position", ReferenceFrame.getWorldFrame(), registry);
      externalPelvisQuaternion = new YoFrameQuaternion("newExternalPelvis_quaternion", ReferenceFrame.getWorldFrame(), registry);
      externalPelvisYaw = new DoubleYoVariable("newExternalPelvis_yaw", registry);
      externalPelvisPitch = new DoubleYoVariable("newExternalPelvis_pitch", registry);
      externalPelvisRoll = new DoubleYoVariable("newExternalPelvis_roll", registry);
      
      totalErrorPelvisPosition = new YoFramePoint("totalErrorPelvis_position", ReferenceFrame.getWorldFrame(), registry);
      totalErrorPelvisQuaternion = new YoFrameQuaternion("totalErrorPelvis_quaternion", ReferenceFrame.getWorldFrame(), registry);
      totalErrorPelvisYaw = new DoubleYoVariable("totalErrorPelvis_yaw", registry);
      totalErrorPelvisPitch = new DoubleYoVariable("totalErrorPelvis_pitch", registry);
      totalErrorPelvisRoll = new DoubleYoVariable("totalErrorPelvis_roll", registry);
      
      interpolatedPelvisErrorPosition = new YoFramePoint("interpolatedPelvisError_position", ReferenceFrame.getWorldFrame(), registry);
      interpolatedPelvisErrorQuaternion = new YoFrameQuaternion("interpolatedPelvisError_quaternion", ReferenceFrame.getWorldFrame(), registry);
      interpolatedPelvisErrorYaw = new DoubleYoVariable("interpolatedPelvisError_yaw", registry);
      interpolatedPelvisErrorPitch = new DoubleYoVariable("interpolatedPelvisError_pitch", registry);
      interpolatedPelvisErrorRoll = new DoubleYoVariable("interpolatedPelvisError_roll", registry);
      
      previousInterpolatedPelvisErrorPosition = new YoFramePoint("previousInterpolatedPelvisError_position", ReferenceFrame.getWorldFrame(), registry);
      previousInterpolatedPelvisErrorQuaternion = new YoFrameQuaternion("previousInterpolatedPelvisError_quaternion", ReferenceFrame.getWorldFrame(), registry);
      previousInterpolatedPelvisErrorYaw = new DoubleYoVariable("previousInterpolatedPelvisError_yaw", registry);
      previousInterpolatedPelvisErrorPitch = new DoubleYoVariable("previousInterpolatedPelvisError_pitch", registry);
      previousInterpolatedPelvisErrorRoll = new DoubleYoVariable("previousInterpolatedPelvisError_roll", registry);
      
      seNonProcessedPelvisPosition = new YoFramePoint("seNonProcessedPelvis_position", ReferenceFrame.getWorldFrame(), registry);
      seNonProcessedPelvisQuaternion = new YoFrameQuaternion("seNonProcessedPelvis_quaternion", ReferenceFrame.getWorldFrame(), registry);
      seNonProcessedPelvisYaw = new DoubleYoVariable("seNonProcessedPelvis_yaw", registry);
      seNonProcessedPelvisPitch = new DoubleYoVariable("seNonProcessedPelvis_pitch", registry);
      seNonProcessedPelvisRoll = new DoubleYoVariable("seNonProcessedPelvis_roll", registry);
      seNonProcessedPelvisTimeStamp = new LongYoVariable("seNonProcessedPelvis_timestamp", registry);
      
      correctedPelvisPosition = new YoFramePoint("correctedPelvis_position", ReferenceFrame.getWorldFrame(), registry);
      correctedPelvisQuaternion = new YoFrameQuaternion("correctedPelvis_quaternion", ReferenceFrame.getWorldFrame(), registry);
      correctedPelvisYaw = new DoubleYoVariable("correctedPelvis_yaw", registry);
      correctedPelvisPitch = new DoubleYoVariable("correctedPelvis_pitch", registry);
      correctedPelvisRoll = new DoubleYoVariable("correctedPelvis_roll", registry);
      
      setupYoVariableManualTranslationBoolean();
   }
   
   /**
    * Converges the state estimator pelvis pose towards an external position provided by an external Pelvis Pose Subscriber
    * @param l 
    */
   public void doControl(long visionSensorTimestamp)
   {
      if (externalPelvisPoseSubscriber != null)
      {
         if (externalPelvisPoseSubscriber.hasNewPose())
         {
            processNewPacket();
         }

         rootJointFrame.getTransformToParent(pelvisPose);
         interpolationAlphaFilter.update(confidenceFactor.getDoubleValue());

         addPelvisePoseToPelvisBuffer(visionSensorTimestamp);
         updateNonProcessedYoVariables();
         calculateCorrectedPelvisPose();
         updateProcessedYoVariables();

         rootJoint.setPositionAndRotation(pelvisPose);
         rootJointFrame.update();
      }
   }
   
   private void calculateCorrectedPelvisPose()
   {
      correctedPelvisPoseWorkingArea.set(getInterpolatedPelvisError());
      correctedPelvisPoseWorkingArea.multiply(pelvisPose);
      pelvisPose.set(correctedPelvisPoseWorkingArea);
   }
   
   /**
    * Calculates the instantaneous offset for this tick
    * @return
    */
   private RigidBodyTransform getInterpolatedPelvisError()
   {
      //first interpolate the total error
      RigidBodyTransform totalError = getTotalError();
      updateTotalErrorYoVariables(totalError);

      transformInterpolationCalculator.computeInterpolation(interpolatorStartingPosition, totalError, interpolatedError,
            interpolationAlphaFilter.getDoubleValue());
      
      updateInterpoledPelvisErrorYoVariables();
      previousInterpolatedError.set(interpolatedError);
      return interpolatedError;
   }
   
   /**
    * Returns the difference between the pelvis pose provided by an external source at t with the state estimated pelvis pose at t
    * @return external pelvis pose - state estimator pelvis pose
    */
   private RigidBodyTransform getTotalError()
   {
      int index = bufferIndex - 1;
      if (index < 0)
      {
         index = errorBuffer.length - 1;
      }
      return errorBuffer[index].getTransform3D();
   }

   private void addPelvisePoseToPelvisBuffer(long timeStamp)
   {
      seNonProcessedPelvisTimeStamp.set(timeStamp);
      stateEstimatorPelvisPoseBuffer.put(pelvisPose, timeStamp);
   }

   private void processNewPacket()
   {
      StampedPosePacket newPacket = externalPelvisPoseSubscriber.getNewExternalPose();
      TimeStampedTransform3D timeStampedExternalPose = newPacket.getTransform();
      if (stateEstimatorPelvisPoseBuffer.isInRange(timeStampedExternalPose.getTimeStamp()))
      {
         RigidBodyTransform newPelvisPose = timeStampedExternalPose.getTransform3D();
         updateExternalPelvisPositionYoVariables(newPelvisPose);
         
         double confidence = newPacket.getConfidenceFactor();
         confidence = MathTools.clipToMinMax(confidence, 0.0, 1.0);
         confidenceFactor.set(confidence);
         
         addNewExternalPose(timeStampedExternalPose);
         
         interpolatorStartingPosition.set(previousInterpolatedError);
         previousInterpolatedError.setIdentity();
         
         updatePreviousInterpoledPelvisErrorYoVariables();
      }
   }

   /**
    * Get a corrected pelvis pose that relates to a pose in the past and add it to the buffer.
    */
   private void addNewExternalPose(TimeStampedTransform3D newPelvisPoseWithTime)
   {
      interpolationAlphaFilter.set(0);
      calculateErrorInPast(newPelvisPoseWithTime, errorBuffer[bufferIndex]);
      incrementBufferIndex();
   }

   private void incrementBufferIndex()
   {
      bufferIndex++;
      if (bufferIndex >= errorBuffer.length)
      {
         bufferIndex = 0;
      }
   }

   Matrix3d rotationMatrix = new Matrix3d();
   /**
    * Calculates the difference between the external at t with the state estimated pelvis pose at t and stores it in the target
    * @param localizationPose - the corrected pelvis pose
    * @param errorToPack - Corrected pelvis pose - state estimator pelvis pose 
    */
   public void calculateErrorInPast(TimeStampedTransform3D localizationPose, TimeStampedTransform3D errorToPack)
   {
      long timeStamp = localizationPose.getTimeStamp();
      errorToPack.setTimeStamp(timeStamp);
      TimeStampedTransform3D sePose = stateEstimatorPelvisPoseBuffer.interpolate(timeStamp);
      RigidBodyTransform error = localizationPose.getTransform3D();
      tempTransform.set(sePose.getTransform3D());
      tempTransform.invert();
      error.multiply(tempTransform);
      
      error.get(rotationMatrix);
      RotationFunctions.setYawPitchRoll(rotationMatrix, RotationFunctions.getYaw(rotationMatrix), 0, 0);
      error.setRotation(rotationMatrix);
      
      errorToPack.setTransform3D(error);
   }
   

   private void updateNonProcessedYoVariables()
   {
      pelvisPose.get(tempVector);
      pelvisPose.get(rot);
      
      seNonProcessedPelvisPosition.set(tempVector);
      seNonProcessedPelvisQuaternion.set(rot);
      seNonProcessedPelvisQuaternion.getYawPitchRoll(tempRots);
      seNonProcessedPelvisYaw.set(tempRots[0]);
      seNonProcessedPelvisPitch.set(tempRots[1]);
      seNonProcessedPelvisRoll.set(tempRots[2]);
   }
   
   private void updateInterpoledPelvisErrorYoVariables()
   {
      interpolatedError.get(tempVector);
      interpolatedError.get(rot);
      
      interpolatedPelvisErrorPosition.set(tempVector);
      interpolatedPelvisErrorQuaternion.set(rot);
      interpolatedPelvisErrorQuaternion.getYawPitchRoll(tempRots);
      interpolatedPelvisErrorYaw.set(tempRots[0]);
      interpolatedPelvisErrorPitch.set(tempRots[1]);
      interpolatedPelvisErrorRoll.set(tempRots[2]);
   }
   
   private void updatePreviousInterpoledPelvisErrorYoVariables()
   {
      previousInterpolatedError.get(tempVector);
      previousInterpolatedError.get(rot);
      
      previousInterpolatedPelvisErrorPosition.set(tempVector);
      previousInterpolatedPelvisErrorQuaternion.set(rot);
      previousInterpolatedPelvisErrorQuaternion.getYawPitchRoll(tempRots);
      previousInterpolatedPelvisErrorYaw.set(tempRots[0]);
      previousInterpolatedPelvisErrorPitch.set(tempRots[1]);
      previousInterpolatedPelvisErrorRoll.set(tempRots[2]);
   }
   
   private void updateExternalPelvisPositionYoVariables(RigidBodyTransform externalPelvisPose)
   {
      externalPelvisPose.get(tempVector);
      externalPelvisPose.get(rot);
      
      externalPelvisPosition.set(tempVector);
      externalPelvisQuaternion.set(rot);
      externalPelvisQuaternion.getYawPitchRoll(tempRots);
      externalPelvisYaw.set(tempRots[0]);
      externalPelvisPitch.set(tempRots[1]);
      externalPelvisRoll.set(tempRots[2]);
   }
   
   private void updateTotalErrorYoVariables(RigidBodyTransform totalError)
   {
      totalError.get(tempVector);
      totalError.get(rot);
      
      totalErrorPelvisPosition.set(tempVector);
      totalErrorPelvisQuaternion.set(rot);
      totalErrorPelvisQuaternion.getYawPitchRoll(tempRots);
      totalErrorPelvisYaw.set(tempRots[0]);
      totalErrorPelvisPitch.set(tempRots[1]);
      totalErrorPelvisRoll.set(tempRots[2]);
   }
   
   private void updateProcessedYoVariables()
   {
      pelvisPose.get(tempVector);
      pelvisPose.get(rot);
      
      correctedPelvisPosition.set(tempVector);
      correctedPelvisQuaternion.set(rot);
      correctedPelvisQuaternion.getYawPitchRoll(tempRots);
      correctedPelvisYaw.set(tempRots[0]);
      correctedPelvisPitch.set(tempRots[1]);
      correctedPelvisRoll.set(tempRots[2]);
   }

   private void setupYoVariableManualTranslationBoolean()
   {
      final DoubleYoVariable manualTranslationOffsetX = new DoubleYoVariable("manualTranslationOffset_X", registry);
      final DoubleYoVariable manualTranslationOffsetY = new DoubleYoVariable("manualTranslationOffset_Y", registry);
      final DoubleYoVariable manualTranslationOffsetZ = new DoubleYoVariable("manualTranslationOffset_Z", registry);
      final DoubleYoVariable manualRotationOffsetInRadX = new DoubleYoVariable("manualRotationOffsetInRad_X", registry);
      final DoubleYoVariable manualRotationOffsetInRadY = new DoubleYoVariable("manualRotationOffsetInRad_Y", registry);
      final DoubleYoVariable manualRotationOffsetInRadZ = new DoubleYoVariable("manualRotationOffsetInRad_Z", registry);
      
      final BooleanYoVariable randomlyGenerateATransform = new BooleanYoVariable("PelvisErrorCorrector_RandomlyGenerateOffset", registry);
      randomlyGenerateATransform.addVariableChangedListener(new VariableChangedListener()
      {

         @Override
         public void variableChanged(YoVariable<?> v)
         {
            if (randomlyGenerateATransform.getBooleanValue())
            {
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
               randomlyGenerateATransform.set(false);
            }
         }
      });
   }

   public void setExternelPelvisCorrectorSubscriber(ExternalPelvisPoseSubscriberInterface externalPelvisPoseSubscriber)
   {
      this.externalPelvisPoseSubscriber = externalPelvisPoseSubscriber;
   }
}
