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

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.SdfLoader.partNames.SpineJointName;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.diagnostic.DelayEstimatorBetweenTwoSignals;
import us.ihmc.sensorProcessing.diagnostic.DiagnosticParameters;
import us.ihmc.sensorProcessing.diagnostic.IMUSensorValidityChecker;
import us.ihmc.sensorProcessing.diagnostic.OrientationAngularVelocityConsistencyChecker;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.simulationconstructionset.util.math.functionGenerator.YoFunctionGenerator;
import us.ihmc.simulationconstructionset.util.math.functionGenerator.YoFunctionGeneratorMode;
import us.ihmc.wholeBodyController.diagnostics.utils.DiagnosticTask;

public class PelvisIMUCheckUpDiagnosticTask extends DiagnosticTask
{
   private Logger logger;
   private final NumberFormat doubleFormat = new DecimalFormat("0.000;-0.000");

   private final YoVariableRegistry registry;

   private final IMUDefinition imuDefinition;
   private final IMUSensorReadOnly imuSensor;

   private final IMUSensorValidityChecker validityChecker;
   private final OrientationAngularVelocityConsistencyChecker orientationVelocityConsistency;
   private final DelayEstimatorBetweenTwoSignals delayEstimator;

   private final YoFunctionGenerator functionGenerator;
   private final DoubleYoVariable checkUpDuration;

   private final YoFrameVector imuAngularVelocityInPelvis;
   private final EnumMap<Axis, DoubleYoVariable> meanOfJointVelocities = new EnumMap<>(Axis.class);

   private final DoubleYoVariable rampDuration;
   private final EnumMap<Axis, DoubleYoVariable> ramps = new EnumMap<>(Axis.class);
   private final EnumMap<Axis, Axis> pelvisToIMUAxes = new EnumMap<>(Axis.class);

   private final DiagnosticParameters diagnosticParameters;

   private final EnumMap<Axis, List<OneDoFJoint>> jointsToWiggleLists = new EnumMap<>(Axis.class);
   private final EnumMap<Axis, Set<OneDoFJoint>> jointsToWiggle = new EnumMap<>(Axis.class);
   private final EnumMap<Axis, DoubleYoVariable> desiredJointPositionOffsets = new EnumMap<>(Axis.class);
   private final EnumMap<Axis, DoubleYoVariable> desiredJointVelocityOffsets = new EnumMap<>(Axis.class);

   private final EnumMap<Axis, DoubleYoVariable> axisEvaluationStartTime = new EnumMap<>(Axis.class);
   private final EnumMap<Axis, DoubleYoVariable> axisEvaluationEndTime = new EnumMap<>(Axis.class);

   private final Mean velocityToOrientationQualityMeanCalculator = new Mean();
   private final DoubleYoVariable velocityToOrientationQualityMean;
   private final Mean velocityToOrientationQualityStandardDeviationCalculator = new Mean();
   private final DoubleYoVariable velocityToOrientationQualityStandardDeviation;

   private final Mean velocityToOrientationDelayMeanCalculator = new Mean();
   private final DoubleYoVariable velocityToOrientationDelayMean;
   private final Mean velocityToOrientationDelayStandardDeviationCalculator = new Mean();
   private final DoubleYoVariable velocityToOrientationDelayStandardDeviation;

   public PelvisIMUCheckUpDiagnosticTask(String imuToCheck, DiagnosticControllerToolbox toolbox, YoVariableRegistry parentRegistry)
   {
      this(findIMUDefinition(imuToCheck, toolbox), toolbox, parentRegistry);
   }

   public PelvisIMUCheckUpDiagnosticTask(IMUDefinition imuToCheck, DiagnosticControllerToolbox toolbox, YoVariableRegistry parentRegistry)
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
      parentRegistry.addChild(registry);
      diagnosticParameters = toolbox.getDiagnosticParameters();

      for (Axis axis : Axis.values)
      {
         DoubleYoVariable desiredJointPositionOffset = new DoubleYoVariable("q_off_d_pelvis" + axis + nameSuffix, registry);
         DoubleYoVariable desiredJointVelocityOffset = new DoubleYoVariable("qd_off_d_pelvis" + axis + nameSuffix, registry);
         desiredJointPositionOffsets.put(axis, desiredJointPositionOffset);
         desiredJointVelocityOffsets.put(axis, desiredJointVelocityOffset);

         DoubleYoVariable sumOfQd = new DoubleYoVariable("qd_w" + axis + "_fromJoints", registry);
         meanOfJointVelocities.put(axis, sumOfQd);
      }

      ReferenceFrame pelvisFrame = pelvis.getBodyFixedFrame();
      imuAngularVelocityInPelvis = new YoFrameVector("qd_w", imuName + "PelvisFrame", pelvisFrame, registry);

      checkUpDuration = new DoubleYoVariable(imuName + nameSuffix + "Duration", registry);
      checkUpDuration.set(diagnosticParameters.getJointCheckUpDuration());

      rampDuration = new DoubleYoVariable(imuName + nameSuffix + "SignalRampDuration", registry);
      rampDuration.set(0.2 * checkUpDuration.getDoubleValue());

      double startTime = 0.0;
      double endTime = checkUpDuration.getDoubleValue() + 2.0 * rampDuration.getDoubleValue();

      for (Axis axis : Axis.values)
      {
         ramps.put(axis, new DoubleYoVariable(imuName + nameSuffix + "SignalRamp" + axis, registry));

         DoubleYoVariable yoStartTime = new DoubleYoVariable(imuName + nameSuffix + "EvalutionStartTimeAxis" + axis, registry);
         DoubleYoVariable yoEndTime = new DoubleYoVariable(imuName + nameSuffix + "EvalutionEndTimeAxis" + axis, registry);

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

      velocityToOrientationQualityMean = new DoubleYoVariable(imuName + nameSuffix + "VelocityToOrientationQualityMean", registry);
      velocityToOrientationQualityStandardDeviation = new DoubleYoVariable(imuName + nameSuffix + "VelocityToOrientationQualityStandardDeviation", registry);

      velocityToOrientationDelayMean = new DoubleYoVariable(imuName + nameSuffix + "VelocityToOrientationDelayMean", registry);
      velocityToOrientationDelayStandardDeviation = new DoubleYoVariable(imuName + nameSuffix + "VelocityToOrientationDelayStandardDeviation", registry);

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

      jointsToWiggle.put(Axis.X, rollJointsAttachedToPelvis);
      jointsToWiggle.put(Axis.Y, pitchJointsAttachedToPelvis);
      jointsToWiggle.put(Axis.Z, yawJointsAttachedToPelvis);
      jointsToWiggleLists.put(Axis.X, new ArrayList<>(rollJointsAttachedToPelvis));
      jointsToWiggleLists.put(Axis.Y, new ArrayList<>(pitchJointsAttachedToPelvis));
      jointsToWiggleLists.put(Axis.Z, new ArrayList<>(yawJointsAttachedToPelvis));

      buildPelvisToIMUAxesMap(pelvis);
   }

   /**
    * Pretty hackish and ugly but that'll do for now.
    * @param pelvis
    */
   private void buildPelvisToIMUAxesMap(RigidBody pelvis)
   {
      FrameVector xVector = new FrameVector(pelvis.getBodyFixedFrame(), 1.0, 0.0, 0.0);
      xVector.changeFrame(imuDefinition.getIMUFrame());
      xVector.absolute();
      if (xVector.getX() >= xVector.getY() && xVector.getX() >= xVector.getZ())
         pelvisToIMUAxes.put(Axis.X, Axis.X);
      else if (xVector.getY() >= xVector.getX() && xVector.getY() >= xVector.getZ())
         pelvisToIMUAxes.put(Axis.X, Axis.Y);
      else
         pelvisToIMUAxes.put(Axis.X, Axis.Z);

      FrameVector yVector = new FrameVector(pelvis.getBodyFixedFrame(), 0.0, 1.0, 0.0);
      yVector.changeFrame(imuDefinition.getIMUFrame());
      yVector.absolute();
      if (yVector.getX() >= yVector.getY() && yVector.getX() >= yVector.getZ())
         pelvisToIMUAxes.put(Axis.Y, Axis.X);
      else if (yVector.getY() >= yVector.getX() && yVector.getY() >= yVector.getZ())
         pelvisToIMUAxes.put(Axis.Y, Axis.Y);
      else
         pelvisToIMUAxes.put(Axis.Y, Axis.Z);

      FrameVector zVector = new FrameVector(pelvis.getBodyFixedFrame(), 0.0, 0.0, 1.0);
      zVector.changeFrame(imuDefinition.getIMUFrame());
      zVector.absolute();
      if (zVector.getX() >= zVector.getY() && zVector.getX() >= zVector.getZ())
         pelvisToIMUAxes.put(Axis.Z, Axis.X);
      else if (zVector.getY() >= zVector.getX() && zVector.getY() >= zVector.getZ())
         pelvisToIMUAxes.put(Axis.Z, Axis.Y);
      else
         pelvisToIMUAxes.put(Axis.Z, Axis.Z);
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

      for (Axis axis : Axis.values)
         ramps.get(axis).set(0.0);
   }

   private boolean enableEstimators = true;
   private boolean disableEstimators = true;

   @Override
   public void doAction()
   {
      Axis currentAxis = null;

      for (Axis axis : Axis.values)
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
               orientationVelocityConsistency.disableAll();
               delayEstimator.disable();
               velocityToOrientationQualityMeanCalculator.clear();
               velocityToOrientationQualityStandardDeviationCalculator.clear();
               velocityToOrientationDelayMeanCalculator.clear();
               velocityToOrientationDelayStandardDeviationCalculator.clear();
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
               orientationVelocityConsistency.enableAll();
               delayEstimator.enable();
               enableEstimators = false;
               disableEstimators = true;
            }
            ramp = 1.0;
         }

         ramps.get(axis).set(MathTools.clipToMinMax(ramp, 0.0, 1.0));
      }

      FrameVector tempAngularVelocity = new FrameVector(imuSensor.getMeasurementFrame());
      imuSensor.getAngularVelocityMeasurement(tempAngularVelocity.getVector());
      imuAngularVelocityInPelvis.setAndMatchFrame(tempAngularVelocity);

      // Really hackish, but it should work.
      // The idea is to compare the joint velocities against the IMU velocity to look for delay.
      // By taking the negative average the velocity should be pretty close to the provided by the IMU.
      // If close enough the delay estimator should work, allowing us to compare IMU and joint sensors.
      for (Axis axis : Axis.values)
      {
         meanOfJointVelocities.get(axis).set(0.0);
         for (int i = 0; i < jointsToWiggleLists.get(axis).size(); i++)
            meanOfJointVelocities.get(axis).add(jointsToWiggleLists.get(axis).get(i).getQd());
         meanOfJointVelocities.get(axis).mul(-1.0 / jointsToWiggleLists.get(axis).size());
      }

      updateDesiredJointOffsets();

      if (currentAxis == null)
         return;

      Axis imuAxis = pelvisToIMUAxes.get(currentAxis);

      if (orientationVelocityConsistency.isEstimatingDelay(imuAxis))
      {
         velocityToOrientationQualityMeanCalculator.increment(orientationVelocityConsistency.getCorrelation(imuAxis));
         velocityToOrientationQualityMean.set(velocityToOrientationQualityMeanCalculator.getResult());
         velocityToOrientationQualityStandardDeviationCalculator.increment(orientationVelocityConsistency.getCorrelation(imuAxis));
         velocityToOrientationQualityStandardDeviation.set(velocityToOrientationQualityStandardDeviationCalculator.getResult());
         
         velocityToOrientationDelayMeanCalculator.increment(orientationVelocityConsistency.getEstimatedDelay(imuAxis));
         velocityToOrientationDelayMean.set(velocityToOrientationDelayMeanCalculator.getResult());
         velocityToOrientationDelayStandardDeviationCalculator.increment(orientationVelocityConsistency.getEstimatedDelay(imuAxis));
         velocityToOrientationDelayStandardDeviation.set(velocityToOrientationDelayStandardDeviationCalculator.getResult());
      }
   }

   private void updateDesiredJointOffsets()
   {
      for (Axis axis : Axis.values)
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
         logger.info("Done with check up for the IMU: " + imuDefinition.getName());

      for (Axis axis : Axis.values)
         ramps.get(axis).set(0.0);
   }

   private void reportCheckUpResults(Axis axis)
   {
      if (logger == null)
         return;

      Axis imuAxis = pelvisToIMUAxes.get(axis);

      Level logLevel;

      if (velocityToOrientationQualityMean.getDoubleValue() < diagnosticParameters.getBadCorrelation())
         logLevel = Level.SEVERE;
      else if (velocityToOrientationQualityMean.getDoubleValue() < diagnosticParameters.getGoodCorrelation())
         logLevel = Level.WARNING;
      else
         logLevel = Level.INFO;

      String velocityQualityMeanFormatted = doubleFormat.format(velocityToOrientationQualityMean.getDoubleValue());
      String velocityQualityStandardDeviationFormatted = doubleFormat.format(velocityToOrientationQualityStandardDeviation.getDoubleValue());
      logger.log(logLevel,
            "Velocity (qd_w" + imuAxis + ") signal quality against orientation for the IMU: " + imuDefinition.getName() + " equals " + velocityQualityMeanFormatted + "(+/-"
                  + velocityQualityStandardDeviationFormatted
                  + "). Note: 0 means orientation and velocity are completely inconsistent, and 1 they're perfectly matching.");

      if (velocityToOrientationDelayMean.getDoubleValue() > diagnosticParameters.getBadDelay())
         logLevel = Level.SEVERE;
      else if (velocityToOrientationDelayMean.getDoubleValue() > diagnosticParameters.getGoodDelay())
         logLevel = Level.WARNING;
      else
         logLevel = Level.INFO;

      String velocityDelayMeanFormatted = doubleFormat.format(velocityToOrientationDelayMean.getDoubleValue());
      String velocityDelayStandardDeviationFormatted = doubleFormat.format(velocityToOrientationDelayStandardDeviation.getDoubleValue());
      logger.log(logLevel, "Estimated velocity (qd_w" + imuAxis + ") delay w.r.t. orientation for the IMU: " + imuDefinition.getName() + " equals " + velocityDelayMeanFormatted + "(+/-"
            + velocityDelayStandardDeviationFormatted + ").");
   }

   @Override
   public boolean isDone()
   {
      if (validityChecker.sensorsCannotBeTrusted())
      {
         if (logger != null)
            logger.severe(imuDefinition.getName() + " IMU sensor can't be trusted, skipping to the next diagnostic task.");
         return true;
      }

      return getTimeInCurrentTask() >= axisEvaluationEndTime.get(Axis.Z).getDoubleValue();
   }

   @Override
   public boolean abortRequested()
   {
      return false;
   }

   @Override
   public double getDesiredJointPositionOffset(OneDoFJoint joint)
   {
      for (Axis axis : Axis.values)
      {
         if (jointsToWiggle.get(axis).contains(joint))
            return desiredJointPositionOffsets.get(axis).getDoubleValue();
      }
      return 0.0;
   }

   @Override
   public double getDesiredJointVelocityOffset(OneDoFJoint joint)
   {
      for (Axis axis : Axis.values)
      {
         if (jointsToWiggle.get(axis).contains(joint))
            return desiredJointVelocityOffsets.get(axis).getDoubleValue();
      }
      return 0.0;}
}
