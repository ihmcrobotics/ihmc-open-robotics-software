package us.ihmc.wholeBodyController.diagnostics;

import java.util.ArrayDeque;
import java.util.logging.Logger;

import org.apache.commons.math3.stat.descriptive.moment.Mean;
import org.apache.commons.math3.stat.descriptive.moment.StandardDeviation;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.diagnostic.DiagnosticParameters;
import us.ihmc.sensorProcessing.diagnostic.OneDoFJointForceTrackingDelayEstimator;
import us.ihmc.sensorProcessing.diagnostic.OneDoFJointFourierAnalysis;
import us.ihmc.sensorProcessing.diagnostic.OneDoFJointSensorValidityChecker;
import us.ihmc.sensorProcessing.diagnostic.PositionVelocity1DConsistencyChecker;
import us.ihmc.simulationconstructionset.util.math.functionGenerator.YoFunctionGenerator;
import us.ihmc.simulationconstructionset.util.math.functionGenerator.YoFunctionGeneratorMode;
import us.ihmc.wholeBodyController.diagnostics.utils.DiagnosticTask;

public class OneDoFJointCheckUpDiagnosticTask extends DiagnosticTask
{
   private Logger logger;

   private final YoVariableRegistry registry;

   private final OneDoFJoint joint;
   private final DoubleYoVariable desiredJointPositionOffset;
   private final DoubleYoVariable desiredJointVelocityOffset;
   private final DoubleYoVariable desiredJointTauOffset;

   private final OneDoFJointSensorValidityChecker validityChecker;
   private final PositionVelocity1DConsistencyChecker positionVelocityConsistency;
   private final OneDoFJointForceTrackingDelayEstimator forceTrackingDelay;
   private final OneDoFJointFourierAnalysis fourierAnalysis;

   private final YoFunctionGenerator functionGenerator;
   private final DoubleYoVariable checkUpDuration;

   private final DoubleYoVariable rampDuration;
   private final DoubleYoVariable ramp;

   private final DiagnosticParameters diagnosticParameters;

   private final Mean velocityQualityMeanCalculator = new Mean();
   private final DoubleYoVariable velocityQualityMean;
   private final StandardDeviation velocityQualityStandardDeviationCalculator = new StandardDeviation();
   private final DoubleYoVariable velocityQualityStandardDeviation;

   private final Mean velocityDelayMeanCalculator = new Mean();
   private final DoubleYoVariable velocityDelayMean;
   private final StandardDeviation velocityDelayStandardDeviationCalculator = new StandardDeviation();
   private final DoubleYoVariable velocityDelayStandardDeviation;

   private final Mean forceTrackingQualityMeanCalculator = new Mean();
   private final DoubleYoVariable forceTrackingQualityMean;
   private final StandardDeviation forceTrackingQualityStandardDeviationCalculator = new StandardDeviation();
   private final DoubleYoVariable forceTrackingQualityStandardDeviation;

   private final Mean forceTrackingDelayMeanCalculator = new Mean();
   private final DoubleYoVariable forceTrackingDelayMean;
   private final StandardDeviation forceTrackingDelayStandardDeviationCalculator = new StandardDeviation();
   private final DoubleYoVariable forceTrackingDelayStandardDeviation;

   private OneDoFJointCheckUpDiagnosticDataReporter dataReporter;

   public OneDoFJointCheckUpDiagnosticTask(OneDoFJoint jointToCheck, DiagnosticControllerToolbox toolbox)
   {
      this.joint = jointToCheck;
      String jointName = joint.getName();
      String nameSuffix = "CheckUp";
      registry = new YoVariableRegistry(jointName + nameSuffix);
      diagnosticParameters = toolbox.getDiagnosticParameters();

      desiredJointPositionOffset = new DoubleYoVariable("q_off_d_" + jointName + nameSuffix, registry);
      desiredJointVelocityOffset = new DoubleYoVariable("qd_off_d_" + jointName + nameSuffix, registry);
      desiredJointTauOffset = new DoubleYoVariable("tau_off_d_" + jointName + nameSuffix, registry);
      checkUpDuration = new DoubleYoVariable(jointName + nameSuffix + "Duration", registry);
      checkUpDuration.set(diagnosticParameters.getJointCheckUpDuration());

      rampDuration = new DoubleYoVariable(jointName + nameSuffix + "SignalRampDuration", registry);
      rampDuration.set(0.2 * checkUpDuration.getDoubleValue());
      ramp = new DoubleYoVariable(jointName + nameSuffix + "SignalRamp", registry);

      validityChecker = toolbox.getJointSensorValidityChecker(joint);
      positionVelocityConsistency = toolbox.getJointPositionVelocityConsistencyChecker(joint);
      forceTrackingDelay = toolbox.getJointForceTrackingDelayEstimator(joint);
      fourierAnalysis = toolbox.getJointFourierAnalysis(joint);

      velocityQualityMean = new DoubleYoVariable(jointName + nameSuffix + "VelocityQualityMean", registry);
      velocityQualityStandardDeviation = new DoubleYoVariable(jointName + nameSuffix + "VelocityQualityStandardDeviation", registry);

      velocityDelayMean = new DoubleYoVariable(jointName + nameSuffix + "VelocityDelayMean", registry);
      velocityDelayStandardDeviation = new DoubleYoVariable(jointName + nameSuffix + "VelocityDelayStandardDeviation", registry);

      forceTrackingQualityMean = new DoubleYoVariable(jointName + nameSuffix + "ForceTrackingQualityMean", registry);
      forceTrackingQualityStandardDeviation = new DoubleYoVariable(jointName + nameSuffix + "ForceTrackingQualityStandardDeviation", registry);

      forceTrackingDelayMean = new DoubleYoVariable(jointName + nameSuffix + "ForceTrackingDelayMean", registry);
      forceTrackingDelayStandardDeviation = new DoubleYoVariable(jointName + nameSuffix + "ForceTrackingDelayStandardDeviation", registry);

      DoubleYoVariable yoTime = toolbox.getYoTime();
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
      velocityQualityMean.set(Double.NaN);
      velocityQualityStandardDeviation.set(Double.NaN);
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
            positionVelocityConsistency.disable();
            forceTrackingDelay.disable();
            fourierAnalysis.disable();
            disableEstimators = false;
         }
         ramp.set(1.0 - (getTimeInCurrentTask() - checkUpDuration.getDoubleValue() - rampDuration.getDoubleValue()) / rampDuration.getDoubleValue());
      }
      else
      {
         if (enableEstimators)
         {
            positionVelocityConsistency.enable();
            forceTrackingDelay.enable();
            fourierAnalysis.enable();
            enableEstimators = false;
         }
         ramp.set(1.0);
      }

      ramp.set(MathTools.clipToMinMax(ramp.getDoubleValue(), 0.0, 1.0));

      double positionOffset = ramp.getDoubleValue() * functionGenerator.getValue(getTimeInCurrentTask());
      double velocityOffset = ramp.getDoubleValue() * functionGenerator.getValueDot();

      desiredJointPositionOffset.set(positionOffset);
      desiredJointVelocityOffset.set(velocityOffset);
      desiredJointTauOffset.set(0.0);

      if (positionVelocityConsistency.isEstimatingDelay())
      {
         velocityQualityMeanCalculator.increment(positionVelocityConsistency.getConsistencyQuality());
         velocityQualityMean.set(velocityQualityMeanCalculator.getResult());
         velocityQualityStandardDeviationCalculator.increment(positionVelocityConsistency.getConsistencyQuality());
         velocityQualityStandardDeviation.set(velocityQualityStandardDeviationCalculator.getResult());

         velocityDelayMeanCalculator.increment(positionVelocityConsistency.getEstimatedDelay());
         velocityDelayMean.set(velocityDelayMeanCalculator.getResult());
         velocityDelayStandardDeviationCalculator.increment(positionVelocityConsistency.getEstimatedDelay());
         velocityDelayStandardDeviation.set(velocityDelayStandardDeviationCalculator.getResult());
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
      dataReporter = new OneDoFJointCheckUpDiagnosticDataReporter(loggerName, joint, diagnosticParameters, velocityQualityMean, velocityQualityStandardDeviation,
            velocityDelayMean, velocityDelayStandardDeviation, forceTrackingQualityMean, forceTrackingQualityStandardDeviation, forceTrackingDelayMean,
            forceTrackingDelayStandardDeviation, fourierAnalysis, functionGenerator);
      sendDataReporter = true;
   }

   @Override
   public boolean isDone()
   {
      if (validityChecker.sensorsCannotBeTrusted())
      {
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
