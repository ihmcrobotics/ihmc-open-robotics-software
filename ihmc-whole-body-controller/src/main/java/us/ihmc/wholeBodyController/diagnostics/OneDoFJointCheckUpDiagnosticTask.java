package us.ihmc.wholeBodyController.diagnostics;

import java.util.ArrayDeque;
import java.util.logging.Logger;

import org.apache.commons.math3.stat.descriptive.moment.Mean;
import org.apache.commons.math3.stat.descriptive.moment.StandardDeviation;

import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGenerator;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGeneratorMode;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.diagnostic.DiagnosticParameters;
import us.ihmc.sensorProcessing.diagnostic.OneDoFJointForceTrackingDelayEstimator;
import us.ihmc.sensorProcessing.diagnostic.OneDoFJointFourierAnalysis;
import us.ihmc.sensorProcessing.diagnostic.OneDoFJointSensorValidityChecker;
import us.ihmc.sensorProcessing.diagnostic.PositionVelocity1DConsistencyChecker;
import us.ihmc.wholeBodyController.diagnostics.utils.DiagnosticTask;

public class OneDoFJointCheckUpDiagnosticTask extends DiagnosticTask
{
   private Logger logger;

   private final YoVariableRegistry registry;

   private final OneDoFJoint joint;
   private final YoDouble desiredJointPositionOffset;
   private final YoDouble desiredJointVelocityOffset;
   private final YoDouble desiredJointTauOffset;

   private final OneDoFJointSensorValidityChecker validityChecker;
   private final PositionVelocity1DConsistencyChecker positionVelocityConsistency;
   private final OneDoFJointForceTrackingDelayEstimator forceTrackingDelay;
   private final OneDoFJointFourierAnalysis fourierAnalysis;

   private final YoFunctionGenerator functionGenerator;
   private final YoDouble checkUpDuration;

   private final YoDouble rampDuration;
   private final YoDouble ramp;

   private final DiagnosticParameters diagnosticParameters;

   private final Mean processedPositionQualityMeanCalculator = new Mean();
   private final YoDouble processedPositionQualityMean;
   private final StandardDeviation processedPositionQualityStandardDeviationCalculator = new StandardDeviation();
   private final YoDouble processedPositionQualityStandardDeviation;

   private final Mean processedPositionDelayMeanCalculator = new Mean();
   private final YoDouble processedPositionDelayMean;
   private final StandardDeviation processedPositionDelayStandardDeviationCalculator = new StandardDeviation();
   private final YoDouble processedPositionDelayStandardDeviation;

   private final Mean rawVelocityQualityMeanCalculator = new Mean();
   private final YoDouble rawVelocityQualityMean;
   private final StandardDeviation rawVelocityQualityStandardDeviationCalculator = new StandardDeviation();
   private final YoDouble rawVelocityQualityStandardDeviation;

   private final Mean rawVelocityDelayMeanCalculator = new Mean();
   private final YoDouble rawVelocityDelayMean;
   private final StandardDeviation rawVelocityDelayStandardDeviationCalculator = new StandardDeviation();
   private final YoDouble rawVelocityDelayStandardDeviation;

   private final Mean processedVelocityQualityMeanCalculator = new Mean();
   private final YoDouble processedVelocityQualityMean;
   private final StandardDeviation processedVelocityQualityStandardDeviationCalculator = new StandardDeviation();
   private final YoDouble processedVelocityQualityStandardDeviation;

   private final Mean processedVelocityDelayMeanCalculator = new Mean();
   private final YoDouble processedVelocityDelayMean;
   private final StandardDeviation processedVelocityDelayStandardDeviationCalculator = new StandardDeviation();
   private final YoDouble processedVelocityDelayStandardDeviation;

   private final Mean forceTrackingQualityMeanCalculator = new Mean();
   private final YoDouble forceTrackingQualityMean;
   private final StandardDeviation forceTrackingQualityStandardDeviationCalculator = new StandardDeviation();
   private final YoDouble forceTrackingQualityStandardDeviation;

   private final Mean forceTrackingDelayMeanCalculator = new Mean();
   private final YoDouble forceTrackingDelayMean;
   private final StandardDeviation forceTrackingDelayStandardDeviationCalculator = new StandardDeviation();
   private final YoDouble forceTrackingDelayStandardDeviation;

   private OneDoFJointCheckUpDiagnosticDataReporter dataReporter;

   public OneDoFJointCheckUpDiagnosticTask(OneDoFJoint jointToCheck, DiagnosticControllerToolbox toolbox)
   {
      this.joint = jointToCheck;
      String jointName = joint.getName();
      String nameSuffix = "CheckUp";
      registry = new YoVariableRegistry(jointName + nameSuffix);
      diagnosticParameters = toolbox.getDiagnosticParameters();

      desiredJointPositionOffset = new YoDouble("q_off_d_" + jointName + nameSuffix, registry);
      desiredJointVelocityOffset = new YoDouble("qd_off_d_" + jointName + nameSuffix, registry);
      desiredJointTauOffset = new YoDouble("tau_off_d_" + jointName + nameSuffix, registry);
      checkUpDuration = new YoDouble(jointName + nameSuffix + "Duration", registry);
      checkUpDuration.set(diagnosticParameters.getJointCheckUpDuration());

      rampDuration = new YoDouble(jointName + nameSuffix + "SignalRampDuration", registry);
      rampDuration.set(0.2 * checkUpDuration.getDoubleValue());
      ramp = new YoDouble(jointName + nameSuffix + "SignalRamp", registry);

      validityChecker = toolbox.getJointSensorValidityChecker(joint);
      positionVelocityConsistency = toolbox.getJointPositionVelocityConsistencyChecker(joint);
      forceTrackingDelay = toolbox.getJointForceTrackingDelayEstimator(joint);
      fourierAnalysis = toolbox.getJointFourierAnalysis(joint);

      processedPositionQualityMean = new YoDouble(jointName + nameSuffix + "ProcessedPositionQualityMean", registry);
      processedPositionQualityStandardDeviation = new YoDouble(jointName + nameSuffix + "ProcessedPositionQualityStandardDeviation", registry);

      processedPositionDelayMean = new YoDouble(jointName + nameSuffix + "ProcessedPositionDelayMean", registry);
      processedPositionDelayStandardDeviation = new YoDouble(jointName + nameSuffix + "ProcessedPositionDelayStandardDeviation", registry);

      rawVelocityQualityMean = new YoDouble(jointName + nameSuffix + "RawVelocityQualityMean", registry);
      rawVelocityQualityStandardDeviation = new YoDouble(jointName + nameSuffix + "RawVelocityQualityStandardDeviation", registry);

      rawVelocityDelayMean = new YoDouble(jointName + nameSuffix + "RawVelocityDelayMean", registry);
      rawVelocityDelayStandardDeviation = new YoDouble(jointName + nameSuffix + "RawVelocityDelayStandardDeviation", registry);

      processedVelocityQualityMean = new YoDouble(jointName + nameSuffix + "ProcessedVelocityQualityMean", registry);
      processedVelocityQualityStandardDeviation = new YoDouble(jointName + nameSuffix + "ProcessedVelocityQualityStandardDeviation", registry);

      processedVelocityDelayMean = new YoDouble(jointName + nameSuffix + "ProcessedVelocityDelayMean", registry);
      processedVelocityDelayStandardDeviation = new YoDouble(jointName + nameSuffix + "ProcessedVelocityDelayStandardDeviation", registry);

      forceTrackingQualityMean = new YoDouble(jointName + nameSuffix + "ForceTrackingQualityMean", registry);
      forceTrackingQualityStandardDeviation = new YoDouble(jointName + nameSuffix + "ForceTrackingQualityStandardDeviation", registry);

      forceTrackingDelayMean = new YoDouble(jointName + nameSuffix + "ForceTrackingDelayMean", registry);
      forceTrackingDelayStandardDeviation = new YoDouble(jointName + nameSuffix + "ForceTrackingDelayStandardDeviation", registry);

      YoDouble yoTime = toolbox.getYoTime();
      functionGenerator = new YoFunctionGenerator(jointName + nameSuffix, yoTime, registry);
      functionGenerator.setAmplitude(diagnosticParameters.getCheckUpOscillationPositionAmplitude());
      functionGenerator.setFrequency(diagnosticParameters.getCheckUpOscillationPositionFrequency());
      functionGenerator.setResetTime(checkUpDuration.getDoubleValue());
      functionGenerator.setMode(YoFunctionGeneratorMode.SINE);
   }

   public void setupForLogging()
   {
      logger = Logger.getLogger(registry.getName());
   }

   @Override
   public void doTransitionIntoAction()
   {
      if (logger != null)
         logger.info("Starting check up for the joint: " + joint.getName());

      ramp.set(0.0);
      processedPositionQualityMean.set(Double.NaN);
      processedPositionQualityStandardDeviation.set(Double.NaN);
      rawVelocityQualityMean.set(Double.NaN);
      rawVelocityQualityStandardDeviation.set(Double.NaN);
      processedVelocityQualityMean.set(Double.NaN);
      processedVelocityQualityStandardDeviation.set(Double.NaN);
      sendDataReporter = false;
   }

   private boolean enableEstimators = true;
   private boolean disableEstimators = true;

   @Override
   public void doAction()
   {
      if (getTimeInCurrentTask() < rampDuration.getDoubleValue())
         ramp.set(getTimeInCurrentTask() / rampDuration.getDoubleValue());
      else if (getTimeInCurrentTask() > checkUpDuration.getDoubleValue() + rampDuration.getDoubleValue())
      {
         if (disableEstimators)
         {
            reportCheckUpResults();
            disableEstimators();
            disableEstimators = false;
         }
         ramp.set(1.0 - (getTimeInCurrentTask() - checkUpDuration.getDoubleValue() - rampDuration.getDoubleValue()) / rampDuration.getDoubleValue());
      }
      else
      {
         if (enableEstimators)
         {
            enableEstimators();
            enableEstimators = false;
         }
         ramp.set(1.0);
      }

      ramp.set(MathTools.clamp(ramp.getDoubleValue(), 0.0, 1.0));

      double positionOffset = ramp.getDoubleValue() * functionGenerator.getValue(getTimeInCurrentTask());
      double velocityOffset = ramp.getDoubleValue() * functionGenerator.getValueDot();

      desiredJointPositionOffset.set(positionOffset);
      desiredJointVelocityOffset.set(velocityOffset);
      desiredJointTauOffset.set(0.0);

      if (positionVelocityConsistency.isEstimatingDelay())
      {
         processedPositionQualityMeanCalculator.increment(positionVelocityConsistency.getConsistencyQualityForProcessedPosition());
         processedPositionQualityMean.set(processedPositionQualityMeanCalculator.getResult());
         processedPositionQualityStandardDeviationCalculator.increment(positionVelocityConsistency.getConsistencyQualityForProcessedPosition());
         processedPositionQualityStandardDeviation.set(processedPositionQualityStandardDeviationCalculator.getResult());

         processedPositionDelayMeanCalculator.increment(positionVelocityConsistency.getEstimatedDelayForProcessedPosition());
         processedPositionDelayMean.set(processedPositionDelayMeanCalculator.getResult());
         processedPositionDelayStandardDeviationCalculator.increment(positionVelocityConsistency.getEstimatedDelayForProcessedPosition());
         processedPositionDelayStandardDeviation.set(processedPositionDelayStandardDeviationCalculator.getResult());

         rawVelocityQualityMeanCalculator.increment(positionVelocityConsistency.getConsistencyQualityForRawVelocity());
         rawVelocityQualityMean.set(rawVelocityQualityMeanCalculator.getResult());
         rawVelocityQualityStandardDeviationCalculator.increment(positionVelocityConsistency.getConsistencyQualityForRawVelocity());
         rawVelocityQualityStandardDeviation.set(rawVelocityQualityStandardDeviationCalculator.getResult());

         rawVelocityDelayMeanCalculator.increment(positionVelocityConsistency.getEstimatedDelayForRawVelocity());
         rawVelocityDelayMean.set(rawVelocityDelayMeanCalculator.getResult());
         rawVelocityDelayStandardDeviationCalculator.increment(positionVelocityConsistency.getEstimatedDelayForRawVelocity());
         rawVelocityDelayStandardDeviation.set(rawVelocityDelayStandardDeviationCalculator.getResult());

         processedVelocityQualityMeanCalculator.increment(positionVelocityConsistency.getConsistencyQualityForProcessedVelocity());
         processedVelocityQualityMean.set(processedVelocityQualityMeanCalculator.getResult());
         processedVelocityQualityStandardDeviationCalculator.increment(positionVelocityConsistency.getConsistencyQualityForProcessedVelocity());
         processedVelocityQualityStandardDeviation.set(processedVelocityQualityStandardDeviationCalculator.getResult());

         processedVelocityDelayMeanCalculator.increment(positionVelocityConsistency.getEstimatedDelayForProcessedVelocity());
         processedVelocityDelayMean.set(processedVelocityDelayMeanCalculator.getResult());
         processedVelocityDelayStandardDeviationCalculator.increment(positionVelocityConsistency.getEstimatedDelayForProcessedVelocity());
         processedVelocityDelayStandardDeviation.set(processedVelocityDelayStandardDeviationCalculator.getResult());
      }
      
      if (forceTrackingDelay.isEstimatingDelay())
      {
         forceTrackingQualityMeanCalculator.increment(forceTrackingDelay.getCorrelation());
         forceTrackingQualityMean.set(forceTrackingQualityMeanCalculator.getResult());
         forceTrackingQualityStandardDeviationCalculator.increment(forceTrackingDelay.getCorrelation());
         forceTrackingQualityStandardDeviation.set(forceTrackingQualityStandardDeviationCalculator.getResult());

         forceTrackingDelayMeanCalculator.increment(forceTrackingDelay.getEstimatedDelay());
         forceTrackingDelayMean.set(forceTrackingDelayMeanCalculator.getResult());
         forceTrackingDelayStandardDeviationCalculator.increment(forceTrackingDelay.getEstimatedDelay());
         forceTrackingDelayStandardDeviation.set(forceTrackingDelayStandardDeviationCalculator.getResult());
      }
   }

   private void enableEstimators()
   {
      validityChecker.enable();
      positionVelocityConsistency.enable();
      forceTrackingDelay.enable();
      fourierAnalysis.enable();
   }

   private void disableEstimators()
   {
      validityChecker.disable();
      positionVelocityConsistency.disable();
      forceTrackingDelay.disable();
      fourierAnalysis.disable();
   }

   @Override
   public void doTransitionOutOfAction()
   {
      if (logger != null)
         logger.info("Done with check up for the joint: " + joint.getName());

      ramp.set(0.0);
      sendDataReporter = false;
   }

   private boolean sendDataReporter = false;

   private void reportCheckUpResults()
   {
      if (logger == null)
         return;

      String loggerName = logger.getName() + "DataReporter";
      dataReporter = new OneDoFJointCheckUpDiagnosticDataReporter(loggerName, joint, diagnosticParameters,
            processedPositionQualityMean, processedPositionQualityStandardDeviation, processedPositionDelayMean, processedPositionDelayStandardDeviation,
            rawVelocityQualityMean, rawVelocityQualityStandardDeviation, rawVelocityDelayMean, rawVelocityDelayStandardDeviation,
            processedVelocityQualityMean, processedVelocityQualityStandardDeviation, processedVelocityDelayMean, processedVelocityDelayStandardDeviation,
            forceTrackingQualityMean, forceTrackingQualityStandardDeviation, forceTrackingDelayMean,
            forceTrackingDelayStandardDeviation, fourierAnalysis, functionGenerator);
      sendDataReporter = true;
   }

   @Override
   public boolean isDone()
   {
      if (validityChecker.sensorsCannotBeTrusted())
      {
         disableEstimators();

         if (logger != null)
            logger.severe(joint.getName() + " Joint sensors can't be trusted, skipping to the next diagnostic task.");
         return true;
      }

      if (dataReporter != null && !dataReporter.isDoneExportingData())
         return false;

      return getTimeInCurrentTask() >= checkUpDuration.getDoubleValue() + 2.0 * rampDuration.getDoubleValue();
   }

   @Override
   public boolean abortRequested()
   {
      return false;
   }

   @Override
   public double getDesiredJointPositionOffset(OneDoFJoint joint)
   {
      if (joint == this.joint)
         return desiredJointPositionOffset.getDoubleValue();
      else
         return 0.0;
   }

   @Override
   public double getDesiredJointVelocityOffset(OneDoFJoint joint)
   {
      if (joint == this.joint)
         return desiredJointVelocityOffset.getDoubleValue();
      else
         return 0.0;
   }

   @Override
   public double getDesiredJointTauOffset(OneDoFJoint joint)
   {
      if (joint == this.joint)
         return desiredJointTauOffset.getDoubleValue();
      else
         return 0.0;
   }

   @Override
   public void getDataReporterToRun(ArrayDeque<DiagnosticDataReporter> dataReportersToPack)
   {
      if (!sendDataReporter)
         return;
      dataReportersToPack.add(dataReporter);
      sendDataReporter = false;
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
