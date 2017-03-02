package us.ihmc.wholeBodyController.diagnostics;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.logging.Level;
import java.util.logging.Logger;

import org.apache.commons.math3.stat.descriptive.moment.Mean;
import org.apache.commons.math3.stat.descriptive.moment.StandardDeviation;

import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.Direction;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGenerator;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGeneratorMode;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.diagnostic.DelayEstimatorBetweenTwoSignals;
import us.ihmc.sensorProcessing.diagnostic.DiagnosticParameters;
import us.ihmc.sensorProcessing.diagnostic.IMUSensorValidityChecker;
import us.ihmc.sensorProcessing.diagnostic.OneDoFJointSensorValidityChecker;
import us.ihmc.sensorProcessing.diagnostic.OrientationAngularVelocityConsistencyChecker;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.wholeBodyController.diagnostics.utils.DiagnosticTask;

public class PelvisIMUCheckUpDiagnosticTask extends DiagnosticTask
{
   private Logger logger;
   private final NumberFormat doubleFormat = new DecimalFormat("0.000;-0.000");

   private final YoVariableRegistry registry;

   private final IMUDefinition imuDefinition;
   private final IMUSensorReadOnly imuSensor;

   private final EnumMap<Direction, List<OneDoFJointSensorValidityChecker>> jointValidityCheckers = new EnumMap<>(Direction.class);
   private final IMUSensorValidityChecker validityChecker;
   private final OrientationAngularVelocityConsistencyChecker orientationVelocityConsistency;
   private final DelayEstimatorBetweenTwoSignals delayEstimator;

   private final YoFunctionGenerator functionGenerator;
   private final DoubleYoVariable checkUpDuration;

   private final YoFrameVector imuAngularVelocityInPelvis;
   private final EnumMap<Direction, DoubleYoVariable> meanOfJointVelocities = new EnumMap<>(Direction.class);

   private final DoubleYoVariable rampDuration;
   private final EnumMap<Direction, DoubleYoVariable> ramps = new EnumMap<>(Direction.class);

   private final DiagnosticParameters diagnosticParameters;

   private final EnumMap<Direction, List<OneDoFJoint>> jointsToWiggleLists = new EnumMap<>(Direction.class);
   private final EnumMap<Direction, Set<OneDoFJoint>> jointsToWiggle = new EnumMap<>(Direction.class);
   private final EnumMap<Direction, DoubleYoVariable> desiredJointPositionOffsets = new EnumMap<>(Direction.class);
   private final EnumMap<Direction, DoubleYoVariable> desiredJointVelocityOffsets = new EnumMap<>(Direction.class);

   private final EnumMap<Direction, DoubleYoVariable> axisEvaluationStartTime = new EnumMap<>(Direction.class);
   private final EnumMap<Direction, DoubleYoVariable> axisEvaluationEndTime = new EnumMap<>(Direction.class);

   private final Mean velocityToOrientationQualityMeanCalculator = new Mean();
   private final DoubleYoVariable velocityToOrientationQualityMean;
   private final StandardDeviation velocityToOrientationQualityStandardDeviationCalculator = new StandardDeviation();
   private final DoubleYoVariable velocityToOrientationQualityStandardDeviation;

   private final Mean velocityToOrientationDelayMeanCalculator = new Mean();
   private final DoubleYoVariable velocityToOrientationDelayMean;
   private final StandardDeviation velocityToOrientationDelayStandardDeviationCalculator = new StandardDeviation();
   private final DoubleYoVariable velocityToOrientationDelayStandardDeviation;

   private final Mean velocityIMUVsJointQualityMeanCalculator = new Mean();
   private final DoubleYoVariable velocityIMUVsJointQualityMean;
   private final StandardDeviation velocityIMUVsJointQualityStandardDeviationCalculator = new StandardDeviation();
   private final DoubleYoVariable velocityIMUVsJointQualityStandardDeviation;

   private final Mean velocityIMUVsJointDelayMeanCalculator = new Mean();
   private final DoubleYoVariable velocityIMUVsJointDelayMean;
   private final StandardDeviation velocityIMUVsJointDelayStandardDeviationCalculator = new StandardDeviation();
   private final DoubleYoVariable velocityIMUVsJointDelayStandardDeviation;

   public PelvisIMUCheckUpDiagnosticTask(String imuToCheck, DiagnosticControllerToolbox toolbox)
   {
      this(findIMUDefinition(imuToCheck, toolbox), toolbox);
   }

   public PelvisIMUCheckUpDiagnosticTask(IMUDefinition imuToCheck, DiagnosticControllerToolbox toolbox)
   {
      FullHumanoidRobotModel fullRobotModel = toolbox.getFullRobotModel();
      RigidBody pelvis = fullRobotModel.getPelvis();
      if (imuToCheck.getRigidBody() != pelvis)
         throw new RuntimeException("The IMU: " + imuToCheck.getName() + " is not attached to the pelvis, cannot create check up diagnostic for it.");

      this.imuDefinition = imuToCheck;
      String imuName = imuDefinition.getName();
      String nameSuffix = "CheckUp";

      imuSensor = toolbox.getIMUSensorReadOnly(imuName);

      registry = new YoVariableRegistry(imuName + nameSuffix);
      diagnosticParameters = toolbox.getDiagnosticParameters();

      for (Direction direction : Direction.values)
      {
         DoubleYoVariable desiredJointPositionOffset = new DoubleYoVariable("q_off_d_pelvis" + direction + nameSuffix, registry);
         DoubleYoVariable desiredJointVelocityOffset = new DoubleYoVariable("qd_off_d_pelvis" + direction + nameSuffix, registry);
         desiredJointPositionOffsets.put(direction, desiredJointPositionOffset);
         desiredJointVelocityOffsets.put(direction, desiredJointVelocityOffset);

         DoubleYoVariable sumOfQd = new DoubleYoVariable("qd_w" + direction + "_fromJoints", registry);
         meanOfJointVelocities.put(direction, sumOfQd);
      }

      ReferenceFrame pelvisFrame = pelvis.getBodyFixedFrame();
      imuAngularVelocityInPelvis = new YoFrameVector("qd_w", imuName + "PelvisFrame", pelvisFrame, registry);

      checkUpDuration = new DoubleYoVariable(imuName + nameSuffix + "Duration", registry);
      checkUpDuration.set(diagnosticParameters.getJointCheckUpDuration());

      rampDuration = new DoubleYoVariable(imuName + nameSuffix + "SignalRampDuration", registry);
      rampDuration.set(0.2 * checkUpDuration.getDoubleValue());

      double startTime = 0.0;
      double endTime = checkUpDuration.getDoubleValue() + 2.0 * rampDuration.getDoubleValue();

      for (Direction direction : Direction.values)
      {
         ramps.put(direction, new DoubleYoVariable(imuName + nameSuffix + "SignalRamp" + direction, registry));

         DoubleYoVariable yoStartTime = new DoubleYoVariable(imuName + nameSuffix + "EvalutionStartTimeAxis" + direction, registry);
         DoubleYoVariable yoEndTime = new DoubleYoVariable(imuName + nameSuffix + "EvalutionEndTimeAxis" + direction, registry);

         yoStartTime.set(startTime);
         yoEndTime.set(endTime);
         
         startTime += checkUpDuration.getDoubleValue() + rampDuration.getDoubleValue();
         endTime += checkUpDuration.getDoubleValue() + rampDuration.getDoubleValue();

         axisEvaluationStartTime.put(direction, yoStartTime);
         axisEvaluationEndTime.put(direction, yoEndTime);
      }

      validityChecker = toolbox.getIMUSensorValidityChecker(imuName);
      orientationVelocityConsistency = toolbox.getIMUOrientationAngularVelocityConsistencyChecker(imuName);
      delayEstimator = new DelayEstimatorBetweenTwoSignals(imuName + nameSuffix + "DelayAgainstJointSensors", toolbox.getDT(), registry);
      delayEstimator.setAlphaFilterBreakFrequency(diagnosticParameters.getDelayEstimatorFilterBreakFrequency());
      double maxAbsoluteLead = diagnosticParameters.getDelayEstimatorMaximumLead();
      double maxAbsoluteLag = diagnosticParameters.getDelayEstimatorMaximumLag();
      double observationWindow = diagnosticParameters.getDelayEstimatorObservationWindow();
      delayEstimator.setEstimationParameters(maxAbsoluteLead, maxAbsoluteLag, observationWindow);

      velocityToOrientationQualityMean = new DoubleYoVariable(imuName + nameSuffix + "VelocityToOrientationQualityMean", registry);
      velocityToOrientationQualityStandardDeviation = new DoubleYoVariable(imuName + nameSuffix + "VelocityToOrientationQualityStandardDeviation", registry);

      velocityToOrientationDelayMean = new DoubleYoVariable(imuName + nameSuffix + "VelocityToOrientationDelayMean", registry);
      velocityToOrientationDelayStandardDeviation = new DoubleYoVariable(imuName + nameSuffix + "VelocityToOrientationDelayStandardDeviation", registry);
      
      velocityIMUVsJointQualityMean = new DoubleYoVariable(imuName + nameSuffix + "VelocityIMUVsJointQualityMean", registry);
      velocityIMUVsJointQualityStandardDeviation = new DoubleYoVariable(imuName + nameSuffix + "VelocityIMUVsJointQualityStandardDeviation", registry);
      
      velocityIMUVsJointDelayMean = new DoubleYoVariable(imuName + nameSuffix + "VelocityIMUVsJointDelayMean", registry);
      velocityIMUVsJointDelayStandardDeviation = new DoubleYoVariable(imuName + nameSuffix + "VelocityIMUVsJointDelayStandardDeviation", registry);

      DoubleYoVariable yoTime = toolbox.getYoTime();
      functionGenerator = new YoFunctionGenerator(imuName + nameSuffix, yoTime, registry);
      functionGenerator.setAmplitude(diagnosticParameters.getCheckUpOscillationPositionAmplitude());
      functionGenerator.setFrequency(diagnosticParameters.getCheckUpOscillationPositionFrequency());
      functionGenerator.setResetTime(checkUpDuration.getDoubleValue());
      functionGenerator.setMode(YoFunctionGeneratorMode.SINE);

      Set<OneDoFJoint> yawJointsAttachedToPelvis = new HashSet<>();
      Set<OneDoFJoint> pitchJointsAttachedToPelvis = new HashSet<>();
      Set<OneDoFJoint> rollJointsAttachedToPelvis = new HashSet<>();

      yawJointsAttachedToPelvis.add(fullRobotModel.getSpineJoint(SpineJointName.SPINE_YAW));
      pitchJointsAttachedToPelvis.add(fullRobotModel.getSpineJoint(SpineJointName.SPINE_PITCH));
      rollJointsAttachedToPelvis.add(fullRobotModel.getSpineJoint(SpineJointName.SPINE_ROLL));
      for (RobotSide robotSide : RobotSide.values)
      {
         yawJointsAttachedToPelvis.add(fullRobotModel.getLegJoint(robotSide, LegJointName.HIP_YAW));
         pitchJointsAttachedToPelvis.add(fullRobotModel.getLegJoint(robotSide, LegJointName.HIP_PITCH));
         rollJointsAttachedToPelvis.add(fullRobotModel.getLegJoint(robotSide, LegJointName.HIP_ROLL));
      }

      jointsToWiggle.put(Direction.X, rollJointsAttachedToPelvis);
      jointsToWiggle.put(Direction.Y, pitchJointsAttachedToPelvis);
      jointsToWiggle.put(Direction.Z, yawJointsAttachedToPelvis);
      jointsToWiggleLists.put(Direction.X, new ArrayList<>(rollJointsAttachedToPelvis));
      jointsToWiggleLists.put(Direction.Y, new ArrayList<>(pitchJointsAttachedToPelvis));
      jointsToWiggleLists.put(Direction.Z, new ArrayList<>(yawJointsAttachedToPelvis));

      for (Direction direction : Direction.values)
      {
         List<OneDoFJointSensorValidityChecker> jointValidityCheckerList = new ArrayList<>();
         for (OneDoFJoint joint : jointsToWiggle.get(direction))
         {
            jointValidityCheckerList.add(toolbox.getJointSensorValidityChecker(joint));
         }
         jointValidityCheckers.put(direction, jointValidityCheckerList);
      }
   }

   private static IMUDefinition findIMUDefinition(String imuName, DiagnosticControllerToolbox toolbox)
   {
      IMUDefinition imuDefinition = null;

      IMUDefinition[] imuDefinitions = toolbox.getFullRobotModel().getIMUDefinitions();

      for (int i = 0; i < imuDefinitions.length; i++)
      {
         if (imuDefinitions[i].getName().equals(imuName))
            imuDefinition = imuDefinitions[i];
      }

      if (imuDefinition == null)
         throw new RuntimeException("Could not find the IMUDefinition for the imu: " + imuName);

      return imuDefinition;
   }

   public void setupForLogging()
   {
      logger = Logger.getLogger(registry.getName());
   }

   @Override
   public void doTransitionIntoAction()
   {
      if (logger != null)
         logger.info("Starting check up for the IMU: " + imuDefinition.getName());

      for (Direction direction : Direction.values)
         ramps.get(direction).set(0.0);
   }

   private boolean enableEstimators = true;
   private boolean disableEstimators = true;
   private Direction currentDirection = null;

   @Override
   public void doAction()
   {
      currentDirection = null;

      for (Direction direction : Direction.values)
      {
         double startTime = axisEvaluationStartTime.get(direction).getDoubleValue();
         double endTime = axisEvaluationEndTime.get(direction).getDoubleValue();
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
               reportCheckUpResults(direction);
               disableEstimators(direction);

               disableEstimators = false;
               enableEstimators = true;
            }
            ramp = 1.0 - (currentTime - endTime + rampDuration.getDoubleValue()) / rampDuration.getDoubleValue();
         }
         else
         {
            if (currentDirection != null)
               throw new RuntimeException("Should be evaluating only one axis at a time.");

            currentDirection = direction;
            if (enableEstimators)
            {
               enableEstimators(direction);
               enableEstimators = false;
               disableEstimators = true;
            }
            ramp = 1.0;
         }

         ramps.get(direction).set(MathTools.clamp(ramp, 0.0, 1.0));
      }

      FrameVector tempAngularVelocity = new FrameVector(imuSensor.getMeasurementFrame());
      imuSensor.getAngularVelocityMeasurement(tempAngularVelocity.getVector());
      imuAngularVelocityInPelvis.setAndMatchFrame(tempAngularVelocity);

      // Really hackish, but it should work.
      // The idea is to compare the joint velocities against the IMU velocity to look for delay.
      // By taking the negative average the velocity should be pretty close to the provided by the IMU.
      // If close enough the delay estimator should work, allowing us to compare IMU and joint sensors.
      for (Direction direction : Direction.values)
      {
         meanOfJointVelocities.get(direction).set(0.0);
         for (int i = 0; i < jointsToWiggleLists.get(direction).size(); i++)
            meanOfJointVelocities.get(direction).add(jointsToWiggleLists.get(direction).get(i).getQd());
         meanOfJointVelocities.get(direction).mul(-1.0 / jointsToWiggleLists.get(direction).size());
      }

      updateDesiredJointOffsets();

      if (currentDirection == null)
         return;

      if (orientationVelocityConsistency.isEstimatingDelay(currentDirection))
      {
         velocityToOrientationQualityMeanCalculator.increment(orientationVelocityConsistency.getCorrelation(currentDirection));
         velocityToOrientationQualityMean.set(velocityToOrientationQualityMeanCalculator.getResult());
         velocityToOrientationQualityStandardDeviationCalculator.increment(orientationVelocityConsistency.getCorrelation(currentDirection));
         velocityToOrientationQualityStandardDeviation.set(velocityToOrientationQualityStandardDeviationCalculator.getResult());
         
         velocityToOrientationDelayMeanCalculator.increment(orientationVelocityConsistency.getEstimatedDelay(currentDirection));
         velocityToOrientationDelayMean.set(velocityToOrientationDelayMeanCalculator.getResult());
         velocityToOrientationDelayStandardDeviationCalculator.increment(orientationVelocityConsistency.getEstimatedDelay(currentDirection));
         velocityToOrientationDelayStandardDeviation.set(velocityToOrientationDelayStandardDeviationCalculator.getResult());
      }

      double referenceSignalCurrentPosition = meanOfJointVelocities.get(currentDirection).getDoubleValue();
      double delayedSignalCurrentPosition = imuAngularVelocityInPelvis.get(currentDirection);
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

   private void enableEstimators(Direction currentDirection)
   {
      if (currentDirection != null)
      {
         List<OneDoFJointSensorValidityChecker> jointValidityCheckerList = jointValidityCheckers.get(currentDirection);
         for (int i = 0; i < jointValidityCheckerList.size(); i++)
            jointValidityCheckerList.get(i).enable();
      }
      else
      {
         for (Direction direction : Direction.values)
         {
            List<OneDoFJointSensorValidityChecker> jointValidityCheckerList = jointValidityCheckers.get(direction);
            for (int i = 0; i < jointValidityCheckerList.size(); i++)
               jointValidityCheckerList.get(i).enable();
         }
      }

      validityChecker.enable();
      orientationVelocityConsistency.enable();
      delayEstimator.enable();
   }

   private void disableEstimators(Direction currentDirection)
   {
      if (currentDirection != null)
      {
         List<OneDoFJointSensorValidityChecker> jointValidityCheckerList = jointValidityCheckers.get(currentDirection);
         for (int i = 0; i < jointValidityCheckerList.size(); i++)
            jointValidityCheckerList.get(i).disable();
      }
      else
      {
         for (Direction direction : Direction.values)
         {
            List<OneDoFJointSensorValidityChecker> jointValidityCheckerList = jointValidityCheckers.get(direction);
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
      for (Direction direction : Direction.values)
      {
         double positionOffset = ramps.get(direction).getDoubleValue() * functionGenerator.getValue(getTimeInCurrentTask());
         double velocityOffset = ramps.get(direction).getDoubleValue() * functionGenerator.getValueDot();
         
         desiredJointPositionOffsets.get(direction).set(positionOffset);
         desiredJointVelocityOffsets.get(direction).set(velocityOffset);
      }
   }

   @Override
   public void doTransitionOutOfAction()
   {
      if (logger != null)
         logger.info("Done with check up for the IMU: " + imuDefinition.getName());

      for (Direction direction : Direction.values)
         ramps.get(direction).set(0.0);
   }

   private void reportCheckUpResults(Direction direction)
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
            "Velocity (qd_w" + direction + ") signal quality against orientation for the IMU: " + imuDefinition.getName() + " equals "
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
      logger.log(logLevel, "Estimated velocity (qd_w" + direction + ") delay w.r.t. orientation for the IMU: " + imuDefinition.getName() + " equals "
            + velocityToOrientationDelayMeanFormatted + " second (+/-" + velocityToOrientationDelayStandardDeviationFormatted + ").");

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
            "IMU Velocity (qd_w" + direction + ") signal quality against joint velocity: " + imuDefinition.getName() + " equals "
                  + velocityIMUVsJointQualityMeanFormatted + " second (+/-" + velocityIMUVsJointQualityStandardDeviationFormatted
                  + "). Note: 0 means orientation and velocity are completely inconsistent, and 1 they're perfectly matching.");

      if (velocityIMUVsJointDelayMean.getDoubleValue() > diagnosticParameters.getBadDelay())
         logLevel = Level.SEVERE;
      else if (velocityIMUVsJointDelayMean.getDoubleValue() > diagnosticParameters.getGoodDelay())
         logLevel = Level.WARNING;
      else
         logLevel = Level.INFO;

      String velocityIMUVsJointDelayMeanFormatted = doubleFormat.format(velocityIMUVsJointDelayMean.getDoubleValue());
      String velocityIMUVsJointDelayStandardDeviationFormatted = doubleFormat.format(velocityIMUVsJointDelayStandardDeviation.getDoubleValue());
      logger.log(logLevel, "Estimated IMU velocity (qd_w" + direction + ") delay w.r.t. joint velocity: " + imuDefinition.getName() + " equals "
            + velocityIMUVsJointDelayMeanFormatted + " second (+/-" + velocityIMUVsJointDelayStandardDeviationFormatted + ").");
   }

   @Override
   public boolean isDone()
   {
      if (validityChecker.sensorsCannotBeTrusted())
      {
         disableEstimators(null);
         if (logger != null)
            logger.severe(imuDefinition.getName() + " IMU sensor can't be trusted, skipping to the next diagnostic task.");
         return true;
      }

      if (currentDirection != null)
      {
         List<OneDoFJointSensorValidityChecker> jointValidityCheckerList = jointValidityCheckers.get(currentDirection);
         for (int i = 0; i < jointValidityCheckerList.size(); i++)
         {
            if (jointValidityCheckerList.get(i).sensorsCannotBeTrusted())
            {
               disableEstimators(null);
               if (logger != null)
                  logger.severe(imuDefinition.getName() + " IMU sensor can't be trusted, skipping to the next diagnostic task.");
               return true;
            }
         }
      }

      return getTimeInCurrentTask() >= axisEvaluationEndTime.get(Direction.Z).getDoubleValue();
   }

   @Override
   public boolean abortRequested()
   {
      return false;
   }

   @Override
   public double getDesiredJointPositionOffset(OneDoFJoint joint)
   {
      for (Direction direction : Direction.values)
      {
         if (jointsToWiggle.get(direction).contains(joint))
            return desiredJointPositionOffsets.get(direction).getDoubleValue();
      }
      return 0.0;
   }

   @Override
   public double getDesiredJointVelocityOffset(OneDoFJoint joint)
   {
      for (Direction direction : Direction.values)
      {
         if (jointsToWiggle.get(direction).contains(joint))
            return desiredJointVelocityOffsets.get(direction).getDoubleValue();
      }
      return 0.0;
   }

   @Override
   public void attachParentYoVariableRegistry(YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
   }

   @Override
   public String getName()
   {
      return registry.getName();
   }
}
