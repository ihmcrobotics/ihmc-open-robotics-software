package us.ihmc.wholeBodyController.diagnostics;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.HashSet;
import java.util.List;
import java.util.Objects;
import java.util.Set;
import java.util.logging.Level;
import java.util.logging.Logger;

import org.apache.commons.math3.stat.descriptive.moment.Mean;
import org.apache.commons.math3.stat.descriptive.moment.StandardDeviation;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGenerator;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGeneratorMode;
import us.ihmc.sensorProcessing.diagnostic.DelayEstimatorBetweenTwoSignals;
import us.ihmc.sensorProcessing.diagnostic.IMUSensorValidityChecker;
import us.ihmc.sensorProcessing.diagnostic.OneDoFJointSensorValidityChecker;
import us.ihmc.sensorProcessing.diagnostic.OrientationAngularVelocityConsistencyChecker;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.wholeBodyController.diagnostics.utils.DiagnosticTask;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class PelvisIMUCheckUpDiagnosticTask extends DiagnosticTask
{
   private Logger logger;
   private final NumberFormat doubleFormat = new DecimalFormat("0.000;-0.000");

   private final YoRegistry registry;

   private final String imuName;
   private final IMUSensorReadOnly imuSensor;

   private final EnumMap<Axis3D, List<OneDoFJointSensorValidityChecker>> jointValidityCheckers = new EnumMap<>(Axis3D.class);
   private final IMUSensorValidityChecker validityChecker;
   private final OrientationAngularVelocityConsistencyChecker orientationVelocityConsistency;
   private final DelayEstimatorBetweenTwoSignals delayEstimator;

   private final YoFunctionGenerator functionGenerator;
   private final YoDouble checkUpDuration;

   private final YoFrameVector3D imuAngularVelocityInPelvis;
   private final EnumMap<Axis3D, YoDouble> meanOfJointVelocities = new EnumMap<>(Axis3D.class);

   private final YoDouble rampDuration;
   private final EnumMap<Axis3D, YoDouble> ramps = new EnumMap<>(Axis3D.class);

   private final DiagnosticParameters diagnosticParameters;

   private final EnumMap<Axis3D, List<OneDoFJointBasics>> jointsToWiggleLists = new EnumMap<>(Axis3D.class);
   private final EnumMap<Axis3D, Set<OneDoFJointBasics>> jointsToWiggle = new EnumMap<>(Axis3D.class);
   private final EnumMap<Axis3D, YoDouble> desiredJointPositionOffsets = new EnumMap<>(Axis3D.class);
   private final EnumMap<Axis3D, YoDouble> desiredJointVelocityOffsets = new EnumMap<>(Axis3D.class);

   private final EnumMap<Axis3D, YoDouble> axisEvaluationStartTime = new EnumMap<>(Axis3D.class);
   private final EnumMap<Axis3D, YoDouble> axisEvaluationEndTime = new EnumMap<>(Axis3D.class);

   private final Mean velocityToOrientationQualityMeanCalculator = new Mean();
   private final YoDouble velocityToOrientationQualityMean;
   private final StandardDeviation velocityToOrientationQualityStandardDeviationCalculator = new StandardDeviation();
   private final YoDouble velocityToOrientationQualityStandardDeviation;

   private final Mean velocityToOrientationDelayMeanCalculator = new Mean();
   private final YoDouble velocityToOrientationDelayMean;
   private final StandardDeviation velocityToOrientationDelayStandardDeviationCalculator = new StandardDeviation();
   private final YoDouble velocityToOrientationDelayStandardDeviation;

   private final Mean velocityIMUVsJointQualityMeanCalculator = new Mean();
   private final YoDouble velocityIMUVsJointQualityMean;
   private final StandardDeviation velocityIMUVsJointQualityStandardDeviationCalculator = new StandardDeviation();
   private final YoDouble velocityIMUVsJointQualityStandardDeviation;

   private final Mean velocityIMUVsJointDelayMeanCalculator = new Mean();
   private final YoDouble velocityIMUVsJointDelayMean;
   private final StandardDeviation velocityIMUVsJointDelayStandardDeviationCalculator = new StandardDeviation();
   private final YoDouble velocityIMUVsJointDelayStandardDeviation;

   public static class PelvisIMUCheckUpParameters
   {
      private String pelvisIMUName;
      private String spineYawJointName, spinePitchJointName, spineRollJointName;
      private String leftHipYawJointName, leftHipPitchJointName, leftHipRollJointName;
      private String rightHipYawJointName, rightHipPitchJointName, rightHipRollJointName;

      public PelvisIMUCheckUpParameters()
      {
      }

      public void setPelvisIMUName(String pelvisIMUName)
      {
         this.pelvisIMUName = pelvisIMUName;
      }

      public void setSpineJointNames(String spineYawJointName, String spinePitchJointName, String spineRollJointName)
      {
         this.spineYawJointName = spineYawJointName;
         this.spinePitchJointName = spinePitchJointName;
         this.spineRollJointName = spineRollJointName;
      }

      public void setLeftHipJointNames(String leftHipYawJointName, String leftHipPitchJointName, String leftHipRollJointName)
      {
         this.leftHipYawJointName = leftHipYawJointName;
         this.leftHipPitchJointName = leftHipPitchJointName;
         this.leftHipRollJointName = leftHipRollJointName;
      }

      public void setRightHipJointNames(String rightHipYawJointName, String rightHipPitchJointName, String rightHipRollJointName)
      {
         this.rightHipYawJointName = rightHipYawJointName;
         this.rightHipPitchJointName = rightHipPitchJointName;
         this.rightHipRollJointName = rightHipRollJointName;
      }

      public String getPelvisIMUName()
      {
         return pelvisIMUName;
      }

      public String getSpineYawJointName()
      {
         return spineYawJointName;
      }

      public String getSpinePitchJointName()
      {
         return spinePitchJointName;
      }

      public String getSpineRollJointName()
      {
         return spineRollJointName;
      }

      public String getLeftHipYawJointName()
      {
         return leftHipYawJointName;
      }

      public String getLeftHipPitchJointName()
      {
         return leftHipPitchJointName;
      }

      public String getLeftHipRollJointName()
      {
         return leftHipRollJointName;
      }

      public String getRightHipYawJointName()
      {
         return rightHipYawJointName;
      }

      public String getRightHipPitchJointName()
      {
         return rightHipPitchJointName;
      }

      public String getRightHipRollJointName()
      {
         return rightHipRollJointName;
      }
   }

   public PelvisIMUCheckUpDiagnosticTask(PelvisIMUCheckUpParameters checkUpParameters, DiagnosticControllerToolbox toolbox)
   {
      RigidBodyBasics pelvis = toolbox.getRootJoint().getSuccessor();

      String nameSuffix = "CheckUp";
      imuName = checkUpParameters.getPelvisIMUName();
      imuSensor = toolbox.getIMUSensorReadOnly(imuName);

      Objects.requireNonNull(imuSensor, "Could not find IMU sensor from name: " + imuName);

      if (imuSensor.getMeasurementLink() != pelvis)
         throw new RuntimeException("The IMU: " + imuName + " is not attached to the pelvis, cannot create check up diagnostic for it.");

      registry = new YoRegistry(imuName + nameSuffix);
      diagnosticParameters = toolbox.getDiagnosticParameters();

      for (Axis3D axis : Axis3D.values)
      {
         YoDouble desiredJointPositionOffset = new YoDouble("q_off_d_pelvis" + axis + nameSuffix, registry);
         YoDouble desiredJointVelocityOffset = new YoDouble("qd_off_d_pelvis" + axis + nameSuffix, registry);
         desiredJointPositionOffsets.put(axis, desiredJointPositionOffset);
         desiredJointVelocityOffsets.put(axis, desiredJointVelocityOffset);

         YoDouble sumOfQd = new YoDouble("qd_w" + axis + "_fromJoints", registry);
         meanOfJointVelocities.put(axis, sumOfQd);
      }

      ReferenceFrame pelvisFrame = pelvis.getBodyFixedFrame();
      imuAngularVelocityInPelvis = new YoFrameVector3D("qd_w", imuName + "PelvisFrame", pelvisFrame, registry);

      checkUpDuration = new YoDouble(imuName + nameSuffix + "Duration", registry);
      checkUpDuration.set(diagnosticParameters.getJointCheckUpDuration());

      rampDuration = new YoDouble(imuName + nameSuffix + "SignalRampDuration", registry);
      rampDuration.set(0.2 * checkUpDuration.getDoubleValue());

      double startTime = 0.0;
      double endTime = checkUpDuration.getDoubleValue() + 2.0 * rampDuration.getDoubleValue();

      for (Axis3D axis : Axis3D.values)
      {
         ramps.put(axis, new YoDouble(imuName + nameSuffix + "SignalRamp" + axis, registry));

         YoDouble yoStartTime = new YoDouble(imuName + nameSuffix + "EvalutionStartTimeAxis" + axis, registry);
         YoDouble yoEndTime = new YoDouble(imuName + nameSuffix + "EvalutionEndTimeAxis" + axis, registry);

         yoStartTime.set(startTime);
         yoEndTime.set(endTime);

         startTime += checkUpDuration.getDoubleValue() + rampDuration.getDoubleValue();
         endTime += checkUpDuration.getDoubleValue() + rampDuration.getDoubleValue();

         axisEvaluationStartTime.put(axis, yoStartTime);
         axisEvaluationEndTime.put(axis, yoEndTime);
      }

      validityChecker = toolbox.getIMUSensorValidityChecker(imuName);
      orientationVelocityConsistency = toolbox.getIMUOrientationAngularVelocityConsistencyChecker(imuName);
      delayEstimator = new DelayEstimatorBetweenTwoSignals(imuName + nameSuffix + "DelayAgainstJointSensors", toolbox.getDT(), registry);
      delayEstimator.setAlphaFilterBreakFrequency(diagnosticParameters.getDelayEstimatorFilterBreakFrequency());
      double maxAbsoluteLead = diagnosticParameters.getDelayEstimatorMaximumLead();
      double maxAbsoluteLag = diagnosticParameters.getDelayEstimatorMaximumLag();
      double observationWindow = diagnosticParameters.getDelayEstimatorObservationWindow();
      delayEstimator.setEstimationParameters(maxAbsoluteLead, maxAbsoluteLag, observationWindow);

      velocityToOrientationQualityMean = new YoDouble(imuName + nameSuffix + "VelocityToOrientationQualityMean", registry);
      velocityToOrientationQualityStandardDeviation = new YoDouble(imuName + nameSuffix + "VelocityToOrientationQualityStandardDeviation", registry);

      velocityToOrientationDelayMean = new YoDouble(imuName + nameSuffix + "VelocityToOrientationDelayMean", registry);
      velocityToOrientationDelayStandardDeviation = new YoDouble(imuName + nameSuffix + "VelocityToOrientationDelayStandardDeviation", registry);

      velocityIMUVsJointQualityMean = new YoDouble(imuName + nameSuffix + "VelocityIMUVsJointQualityMean", registry);
      velocityIMUVsJointQualityStandardDeviation = new YoDouble(imuName + nameSuffix + "VelocityIMUVsJointQualityStandardDeviation", registry);

      velocityIMUVsJointDelayMean = new YoDouble(imuName + nameSuffix + "VelocityIMUVsJointDelayMean", registry);
      velocityIMUVsJointDelayStandardDeviation = new YoDouble(imuName + nameSuffix + "VelocityIMUVsJointDelayStandardDeviation", registry);

      YoDouble yoTime = toolbox.getYoTime();
      functionGenerator = new YoFunctionGenerator(imuName + nameSuffix, yoTime, registry);
      functionGenerator.setAmplitude(diagnosticParameters.getCheckUpOscillationPositionAmplitude());
      functionGenerator.setFrequency(diagnosticParameters.getCheckUpOscillationPositionFrequency());
      functionGenerator.setResetTime(checkUpDuration.getDoubleValue());
      functionGenerator.setMode(YoFunctionGeneratorMode.SINE);

      Set<OneDoFJointBasics> yawJointsAttachedToPelvis = new HashSet<>();
      Set<OneDoFJointBasics> pitchJointsAttachedToPelvis = new HashSet<>();
      Set<OneDoFJointBasics> rollJointsAttachedToPelvis = new HashSet<>();

      yawJointsAttachedToPelvis.addAll(toolbox.getJoints(checkUpParameters.getSpineYawJointName(),
                                                         checkUpParameters.getLeftHipYawJointName(),
                                                         checkUpParameters.getRightHipYawJointName()));
      pitchJointsAttachedToPelvis.addAll(toolbox.getJoints(checkUpParameters.getSpinePitchJointName(),
                                                           checkUpParameters.getLeftHipPitchJointName(),
                                                           checkUpParameters.getRightHipPitchJointName()));
      rollJointsAttachedToPelvis.addAll(toolbox.getJoints(checkUpParameters.getSpineRollJointName(),
                                                          checkUpParameters.getLeftHipRollJointName(),
                                                          checkUpParameters.getRightHipRollJointName()));

      jointsToWiggle.put(Axis3D.X, rollJointsAttachedToPelvis);
      jointsToWiggle.put(Axis3D.Y, pitchJointsAttachedToPelvis);
      jointsToWiggle.put(Axis3D.Z, yawJointsAttachedToPelvis);
      jointsToWiggleLists.put(Axis3D.X, new ArrayList<>(rollJointsAttachedToPelvis));
      jointsToWiggleLists.put(Axis3D.Y, new ArrayList<>(pitchJointsAttachedToPelvis));
      jointsToWiggleLists.put(Axis3D.Z, new ArrayList<>(yawJointsAttachedToPelvis));

      for (Axis3D axis : Axis3D.values)
      {
         List<OneDoFJointSensorValidityChecker> jointValidityCheckerList = new ArrayList<>();
         for (OneDoFJointBasics joint : jointsToWiggle.get(axis))
         {
            Objects.requireNonNull(toolbox.getJointSensorValidityChecker(joint), "Did not find joint sensor validity checker for joint: " + joint.getName());
            jointValidityCheckerList.add(toolbox.getJointSensorValidityChecker(joint));
         }
         jointValidityCheckers.put(axis, jointValidityCheckerList);
      }
   }

   public void setupForLogging()
   {
      logger = Logger.getLogger(registry.getName());
   }

   @Override
   public void doTransitionIntoAction()
   {
      if (logger != null)
         logger.info("Starting check up for the IMU: " + imuName);

      for (Axis3D axis : Axis3D.values)
         ramps.get(axis).set(0.0);
   }

   private boolean enableEstimators = true;
   private boolean disableEstimators = true;
   private Axis3D currentAxis = null;

   @Override
   public void doAction()
   {
      currentAxis = null;

      for (Axis3D axis : Axis3D.values)
      {
         double startTime = axisEvaluationStartTime.get(axis).getDoubleValue();
         double endTime = axisEvaluationEndTime.get(axis).getDoubleValue();
         double currentTime = getTimeInCurrentTask();
         double ramp;

         if (currentTime < startTime || currentTime > endTime)
            ramp = 0.0;
         else if (currentTime < startTime + rampDuration.getDoubleValue())
            ramp = (currentTime - startTime) / rampDuration.getDoubleValue();
         else if (currentTime > endTime - rampDuration.getDoubleValue())
         {
            if (disableEstimators)
            {
               reportCheckUpResults(axis);
               disableEstimators(axis);

               disableEstimators = false;
               enableEstimators = true;
            }
            ramp = 1.0 - (currentTime - endTime + rampDuration.getDoubleValue()) / rampDuration.getDoubleValue();
         }
         else
         {
            if (currentAxis != null)
               throw new RuntimeException("Should be evaluating only one axis at a time.");

            currentAxis = axis;
            if (enableEstimators)
            {
               enableEstimators(axis);
               enableEstimators = false;
               disableEstimators = true;
            }
            ramp = 1.0;
         }

         ramps.get(axis).set(MathTools.clamp(ramp, 0.0, 1.0));
      }

      FrameVector3D tempAngularVelocity = new FrameVector3D(imuSensor.getMeasurementFrame());
      tempAngularVelocity.set(imuSensor.getAngularVelocityMeasurement());
      imuAngularVelocityInPelvis.setMatchingFrame(tempAngularVelocity);

      // Really hackish, but it should work.
      // The idea is to compare the joint velocities against the IMU velocity to look for delay.
      // By taking the negative average the velocity should be pretty close to the provided by the IMU.
      // If close enough the delay estimator should work, allowing us to compare IMU and joint sensors.
      for (Axis3D axis : Axis3D.values)
      {
         meanOfJointVelocities.get(axis).set(0.0);
         for (int i = 0; i < jointsToWiggleLists.get(axis).size(); i++)
            meanOfJointVelocities.get(axis).add(jointsToWiggleLists.get(axis).get(i).getQd());
         meanOfJointVelocities.get(axis).mul(-1.0 / jointsToWiggleLists.get(axis).size());
      }

      updateDesiredJointOffsets();

      if (currentAxis == null)
         return;

      if (orientationVelocityConsistency.isEstimatingDelay(currentAxis))
      {
         velocityToOrientationQualityMeanCalculator.increment(orientationVelocityConsistency.getCorrelation(currentAxis));
         velocityToOrientationQualityMean.set(velocityToOrientationQualityMeanCalculator.getResult());
         velocityToOrientationQualityStandardDeviationCalculator.increment(orientationVelocityConsistency.getCorrelation(currentAxis));
         velocityToOrientationQualityStandardDeviation.set(velocityToOrientationQualityStandardDeviationCalculator.getResult());

         velocityToOrientationDelayMeanCalculator.increment(orientationVelocityConsistency.getEstimatedDelay(currentAxis));
         velocityToOrientationDelayMean.set(velocityToOrientationDelayMeanCalculator.getResult());
         velocityToOrientationDelayStandardDeviationCalculator.increment(orientationVelocityConsistency.getEstimatedDelay(currentAxis));
         velocityToOrientationDelayStandardDeviation.set(velocityToOrientationDelayStandardDeviationCalculator.getResult());
      }

      double referenceSignalCurrentPosition = meanOfJointVelocities.get(currentAxis).getDoubleValue();
      double delayedSignalCurrentPosition = imuAngularVelocityInPelvis.getElement(currentAxis.ordinal());
      delayEstimator.update(referenceSignalCurrentPosition, delayedSignalCurrentPosition);

      if (delayEstimator.isEstimatingDelay())
      {
         velocityIMUVsJointQualityMeanCalculator.increment(delayEstimator.getCorrelationCoefficient());
         velocityIMUVsJointQualityMean.set(velocityIMUVsJointQualityMeanCalculator.getResult());
         velocityIMUVsJointQualityStandardDeviationCalculator.increment(delayEstimator.getCorrelationCoefficient());
         velocityIMUVsJointQualityStandardDeviation.set(velocityIMUVsJointQualityStandardDeviationCalculator.getResult());

         velocityIMUVsJointDelayMeanCalculator.increment(delayEstimator.getEstimatedDelay());
         velocityIMUVsJointDelayMean.set(velocityIMUVsJointDelayMeanCalculator.getResult());
         velocityIMUVsJointDelayStandardDeviationCalculator.increment(delayEstimator.getEstimatedDelay());
         velocityIMUVsJointDelayStandardDeviation.set(velocityIMUVsJointDelayStandardDeviationCalculator.getResult());
      }
   }

   private void enableEstimators(Axis3D currentAxis)
   {
      if (currentAxis != null)
      {
         List<OneDoFJointSensorValidityChecker> jointValidityCheckerList = jointValidityCheckers.get(currentAxis);
         for (int i = 0; i < jointValidityCheckerList.size(); i++)
            jointValidityCheckerList.get(i).enable();
      }
      else
      {
         for (Axis3D axis : Axis3D.values)
         {
            List<OneDoFJointSensorValidityChecker> jointValidityCheckerList = jointValidityCheckers.get(axis);
            for (int i = 0; i < jointValidityCheckerList.size(); i++)
               jointValidityCheckerList.get(i).enable();
         }
      }

      validityChecker.enable();
      orientationVelocityConsistency.enable();
      delayEstimator.enable();
   }

   private void disableEstimators(Axis3D currentAxis)
   {
      if (currentAxis != null)
      {
         List<OneDoFJointSensorValidityChecker> jointValidityCheckerList = jointValidityCheckers.get(currentAxis);
         for (int i = 0; i < jointValidityCheckerList.size(); i++)
            jointValidityCheckerList.get(i).disable();
      }
      else
      {
         for (Axis3D axis : Axis3D.values)
         {
            List<OneDoFJointSensorValidityChecker> jointValidityCheckerList = jointValidityCheckers.get(axis);
            for (int i = 0; i < jointValidityCheckerList.size(); i++)
               jointValidityCheckerList.get(i).disable();
         }
      }

      validityChecker.disable();
      orientationVelocityConsistency.disable();
      delayEstimator.disable();
      velocityToOrientationQualityMeanCalculator.clear();
      velocityToOrientationQualityStandardDeviationCalculator.clear();
      velocityToOrientationDelayMeanCalculator.clear();
      velocityToOrientationDelayStandardDeviationCalculator.clear();

      velocityIMUVsJointQualityMeanCalculator.clear();
      velocityIMUVsJointQualityStandardDeviationCalculator.clear();
      velocityIMUVsJointDelayMeanCalculator.clear();
      velocityIMUVsJointDelayStandardDeviationCalculator.clear();
   }

   private void updateDesiredJointOffsets()
   {
      for (Axis3D axis : Axis3D.values)
      {
         double positionOffset = ramps.get(axis).getDoubleValue() * functionGenerator.getValue(getTimeInCurrentTask());
         double velocityOffset = ramps.get(axis).getDoubleValue() * functionGenerator.getValueDot();

         desiredJointPositionOffsets.get(axis).set(positionOffset);
         desiredJointVelocityOffsets.get(axis).set(velocityOffset);
      }
   }

   @Override
   public void doTransitionOutOfAction()
   {
      if (logger != null)
         logger.info("Done with check up for the IMU: " + imuName);

      for (Axis3D axis : Axis3D.values)
         ramps.get(axis).set(0.0);
   }

   private void reportCheckUpResults(Axis3D axis)
   {
      if (logger == null)
         return;

      Level logLevel;

      //////////////////////////////////////////////////////////////////////////////////////////
      //// Report results of comparing the IMU orientation against the IMU angular velocity ////
      //////////////////////////////////////////////////////////////////////////////////////////

      if (velocityToOrientationQualityMean.getDoubleValue() < diagnosticParameters.getBadCorrelation())
         logLevel = Level.SEVERE;
      else if (velocityToOrientationQualityMean.getDoubleValue() < diagnosticParameters.getGoodCorrelation())
         logLevel = Level.WARNING;
      else
         logLevel = Level.INFO;

      String velocityToOrientationQualityMeanFormatted = doubleFormat.format(velocityToOrientationQualityMean.getDoubleValue());
      String velocityToOrientationQualityStandardDeviationFormatted = doubleFormat.format(velocityToOrientationQualityStandardDeviation.getDoubleValue());
      logger.log(logLevel,
                 "Velocity (qd_w" + axis + ") signal quality against orientation for the IMU: " + imuName + " equals "
                       + velocityToOrientationQualityMeanFormatted + " second (+/-" + velocityToOrientationQualityStandardDeviationFormatted
                       + "). Note: 0 means orientation and velocity are completely inconsistent, and 1 they're perfectly matching.");

      if (velocityToOrientationDelayMean.getDoubleValue() > diagnosticParameters.getBadDelay())
         logLevel = Level.SEVERE;
      else if (velocityToOrientationDelayMean.getDoubleValue() > diagnosticParameters.getGoodDelay())
         logLevel = Level.WARNING;
      else
         logLevel = Level.INFO;

      String velocityToOrientationDelayMeanFormatted = doubleFormat.format(velocityToOrientationDelayMean.getDoubleValue());
      String velocityToOrientationDelayStandardDeviationFormatted = doubleFormat.format(velocityToOrientationDelayStandardDeviation.getDoubleValue());
      logger.log(logLevel,
                 "Estimated velocity (qd_w" + axis + ") delay w.r.t. orientation for the IMU: " + imuName + " equals " + velocityToOrientationDelayMeanFormatted
                       + " second (+/-" + velocityToOrientationDelayStandardDeviationFormatted + ").");

      ///////////////////////////////////////////////////////////////////////
      //// Report results of comparing the IMU against the joint sensors ////
      ///////////////////////////////////////////////////////////////////////

      if (velocityIMUVsJointQualityMean.getDoubleValue() < diagnosticParameters.getBadCorrelation())
         logLevel = Level.SEVERE;
      else if (velocityIMUVsJointQualityMean.getDoubleValue() < diagnosticParameters.getGoodCorrelation())
         logLevel = Level.WARNING;
      else
         logLevel = Level.INFO;

      String velocityIMUVsJointQualityMeanFormatted = doubleFormat.format(velocityIMUVsJointQualityMean.getDoubleValue());
      String velocityIMUVsJointQualityStandardDeviationFormatted = doubleFormat.format(velocityIMUVsJointQualityStandardDeviation.getDoubleValue());
      logger.log(logLevel,
                 "IMU Velocity (qd_w" + axis + ") signal quality against joint velocity: " + imuName + " equals " + velocityIMUVsJointQualityMeanFormatted
                       + " second (+/-" + velocityIMUVsJointQualityStandardDeviationFormatted
                       + "). Note: 0 means orientation and velocity are completely inconsistent, and 1 they're perfectly matching.");

      if (velocityIMUVsJointDelayMean.getDoubleValue() > diagnosticParameters.getBadDelay())
         logLevel = Level.SEVERE;
      else if (velocityIMUVsJointDelayMean.getDoubleValue() > diagnosticParameters.getGoodDelay())
         logLevel = Level.WARNING;
      else
         logLevel = Level.INFO;

      String velocityIMUVsJointDelayMeanFormatted = doubleFormat.format(velocityIMUVsJointDelayMean.getDoubleValue());
      String velocityIMUVsJointDelayStandardDeviationFormatted = doubleFormat.format(velocityIMUVsJointDelayStandardDeviation.getDoubleValue());
      logger.log(logLevel,
                 "Estimated IMU velocity (qd_w" + axis + ") delay w.r.t. joint velocity: " + imuName + " equals " + velocityIMUVsJointDelayMeanFormatted
                       + " second (+/-" + velocityIMUVsJointDelayStandardDeviationFormatted + ").");
   }

   @Override
   public boolean isDone()
   {
      if (validityChecker.sensorsCannotBeTrusted())
      {
         disableEstimators(null);
         if (logger != null)
            logger.severe(imuName + " IMU sensor can't be trusted, skipping to the next diagnostic task.");
         return true;
      }

      if (currentAxis != null)
      {
         List<OneDoFJointSensorValidityChecker> jointValidityCheckerList = jointValidityCheckers.get(currentAxis);
         for (int i = 0; i < jointValidityCheckerList.size(); i++)
         {
            if (jointValidityCheckerList.get(i).sensorsCannotBeTrusted())
            {
               disableEstimators(null);
               if (logger != null)
                  logger.severe(jointsToWiggleLists.get(currentAxis).get(i).getName() + " sensors can't be trusted, skipping to the next diagnostic task.");
               return true;
            }
         }
      }

      return getTimeInCurrentTask() >= axisEvaluationEndTime.get(Axis3D.Z).getDoubleValue();
   }

   @Override
   public boolean abortRequested()
   {
      return false;
   }

   @Override
   public double getDesiredJointPositionOffset(OneDoFJointBasics joint)
   {
      for (Axis3D axis : Axis3D.values)
      {
         if (jointsToWiggle.get(axis).contains(joint))
            return desiredJointPositionOffsets.get(axis).getDoubleValue();
      }
      return 0.0;
   }

   @Override
   public double getDesiredJointVelocityOffset(OneDoFJointBasics joint)
   {
      for (Axis3D axis : Axis3D.values)
      {
         if (jointsToWiggle.get(axis).contains(joint))
            return desiredJointVelocityOffsets.get(axis).getDoubleValue();
      }
      return 0.0;
   }

   @Override
   public void attachParentYoVariableRegistry(YoRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
   }

   @Override
   public String getName()
   {
      return registry.getName();
   }
}
