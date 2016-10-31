package us.ihmc.wholeBodyController.diagnostics;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.logging.Level;
import java.util.logging.LogRecord;
import java.util.logging.Logger;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGenerator;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGeneratorMode;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.diagnostic.DiagnosticParameters;
import us.ihmc.sensorProcessing.diagnostic.OneDoFJointFourierAnalysis;
import us.ihmc.wholeBodyController.diagnostics.logging.JointForceTrackingDelayLogRecord;
import us.ihmc.wholeBodyController.diagnostics.logging.ProcessedJointPositionDelayLogRecord;
import us.ihmc.wholeBodyController.diagnostics.logging.ProcessedJointVelocityDelayLogRecord;
import us.ihmc.wholeBodyController.diagnostics.logging.RawJointVelocityDelayLogRecord;

public class OneDoFJointCheckUpDiagnosticDataReporter implements DiagnosticDataReporter
{
   private final Logger logger;
   private final NumberFormat doubleFormat = new DecimalFormat("0.000;-0.000");
   private final NumberFormat doubleFormat2 = new DecimalFormat("000.000;-000.000");

   private final String jointName;

   private final double badCorrelation;
   private final double goodCorrelation;
   private final double badDelay;
   private final double goodDelay;

   private final double processedPositionQualityMean;
   private final double processedPositionQualityStandardDeviation;
   private final double processedPositionDelayMean;
   private final double processedPositionDelayStandardDeviation;

   private final double rawVelocityQualityMean;
   private final double rawVelocityQualityStandardDeviation;
   private final double rawVelocityDelayMean;
   private final double rawVelocityDelayStandardDeviation;

   private final double processedVelocityQualityMean;
   private final double processedVelocityQualityStandardDeviation;
   private final double processedVelocityDelayMean;
   private final double processedVelocityDelayStandardDeviation;

   private final double forceTrackingQualityMean;
   private final double forceTrackingQualityStandardDeviation;
   private final double forceTrackingDelayMean;
   private final double forceTrackingDelayStandardDeviation;

   private final int fourierAnalysisOutputSize;
   private final double[] fourierAnalysisFrequency;
   private final double[] fourierAnalysisVelocityMagnitude;
   private final double[] fourierAnalysisTauMagnitude;
   private final double[] fourierAnalysisTauDesiredMagnitude;

   private final YoFunctionGeneratorMode functionGeneratorMode;
   private final double functionGeneratorFrequency;
   private final double functionGeneratorAmplitude;

   private final AtomicBoolean isDoneExportingData = new AtomicBoolean(false);

   public OneDoFJointCheckUpDiagnosticDataReporter(String loggerName, OneDoFJoint joint, DiagnosticParameters diagnosticParameters,
         DoubleYoVariable processedPositionQualityMean, DoubleYoVariable processedPositionQualityStandardDeviation, DoubleYoVariable processedPositionDelayMean, DoubleYoVariable processedPositionDelayStandardDeviation,
         DoubleYoVariable rawVelocityQualityMean, DoubleYoVariable rawVelocityQualityStandardDeviation, DoubleYoVariable rawVelocityDelayMean, DoubleYoVariable rawVelocityDelayStandardDeviation,
         DoubleYoVariable processedVelocityQualityMean, DoubleYoVariable processedVelocityQualityStandardDeviation, DoubleYoVariable processedVelocityDelayMean, DoubleYoVariable processedVelocityDelayStandardDeviation,
         DoubleYoVariable forceTrackingQualityMean, DoubleYoVariable forceTrackingQualityStandardDeviation, DoubleYoVariable forceTrackingDelayMean, DoubleYoVariable forceTrackingDelayStandardDeviation,
         OneDoFJointFourierAnalysis fourierAnalysis, YoFunctionGenerator functionGenerator)
   {
      logger = Logger.getLogger(loggerName);

      jointName = joint.getName();

      badCorrelation = diagnosticParameters.getBadCorrelation();
      goodCorrelation = diagnosticParameters.getGoodCorrelation();
      badDelay = diagnosticParameters.getBadDelay();
      goodDelay = diagnosticParameters.getGoodDelay();

      this.processedPositionQualityMean = processedPositionQualityMean.getDoubleValue();
      this.processedPositionQualityStandardDeviation = processedPositionQualityStandardDeviation.getDoubleValue();
      this.processedPositionDelayMean = processedPositionDelayMean.getDoubleValue();
      this.processedPositionDelayStandardDeviation = processedPositionDelayStandardDeviation.getDoubleValue();

      this.rawVelocityQualityMean = rawVelocityQualityMean.getDoubleValue();
      this.rawVelocityQualityStandardDeviation = rawVelocityQualityStandardDeviation.getDoubleValue();
      this.rawVelocityDelayMean = rawVelocityDelayMean.getDoubleValue();
      this.rawVelocityDelayStandardDeviation = rawVelocityDelayStandardDeviation.getDoubleValue();

      this.processedVelocityQualityMean = processedVelocityQualityMean.getDoubleValue();
      this.processedVelocityQualityStandardDeviation = processedVelocityQualityStandardDeviation.getDoubleValue();
      this.processedVelocityDelayMean = processedVelocityDelayMean.getDoubleValue();
      this.processedVelocityDelayStandardDeviation = processedVelocityDelayStandardDeviation.getDoubleValue();

      this.forceTrackingQualityMean = forceTrackingQualityMean.getDoubleValue();
      this.forceTrackingQualityStandardDeviation = forceTrackingQualityStandardDeviation.getDoubleValue();
      this.forceTrackingDelayMean = forceTrackingDelayMean.getDoubleValue();
      this.forceTrackingDelayStandardDeviation = forceTrackingDelayStandardDeviation.getDoubleValue();

      fourierAnalysisOutputSize = fourierAnalysis.getOutputSize();
      fourierAnalysisFrequency = new double[fourierAnalysisOutputSize];
      fourierAnalysisVelocityMagnitude = new double[fourierAnalysisOutputSize];
      fourierAnalysisTauMagnitude = new double[fourierAnalysisOutputSize];
      fourierAnalysisTauDesiredMagnitude = new double[fourierAnalysisOutputSize];
      fourierAnalysis.getFrequencies(fourierAnalysisFrequency);
      fourierAnalysis.getVelocityMagnitudes(fourierAnalysisVelocityMagnitude);
      fourierAnalysis.getTauMagnitudes(fourierAnalysisTauMagnitude);
      fourierAnalysis.getTauDesiredMagnitudes(fourierAnalysisTauDesiredMagnitude);

      functionGeneratorMode = functionGenerator.getMode();
      functionGeneratorFrequency = functionGenerator.getFrequency();
      functionGeneratorAmplitude = functionGenerator.getAmplitude();
   }

   @Override
   public void run()
   {
      isDoneExportingData.set(false);
      reportCheckUpResults();
      isDoneExportingData.set(true);
   }

   private enum DelaySignalType
   {
      PROCESSED_POSITION, RAW_VELOCITY, PROCESSED_VELOCITY;

      public LogRecord createLogRecord(Level level, String msg)
      {
         switch (this)
         {
         case PROCESSED_POSITION:
            return new ProcessedJointPositionDelayLogRecord(level, msg);
         case PROCESSED_VELOCITY:
            return new ProcessedJointVelocityDelayLogRecord(level, msg);
         case RAW_VELOCITY:
            return new RawJointVelocityDelayLogRecord(level, msg);
         default:
            throw new RuntimeException("Should not get there.");
         }
      }

      public String getSignalName()
      {
         switch (this)
         {
         case PROCESSED_POSITION:
            return "processed position";
         case RAW_VELOCITY:
            return "raw velocity";
         case PROCESSED_VELOCITY:
            return "processed velocity";
         default:
            throw new RuntimeException("Should not get there.");
         }
      }
   }

   private void reportCheckUpResults()
   {
      if (logger == null)
         return;

      Level logLevel;

      reportDelay(DelaySignalType.PROCESSED_POSITION, processedPositionQualityMean, processedPositionQualityStandardDeviation, processedPositionDelayMean, processedPositionDelayStandardDeviation);
      reportDelay(DelaySignalType.RAW_VELOCITY, rawVelocityQualityMean, rawVelocityQualityStandardDeviation, rawVelocityDelayMean, rawVelocityDelayStandardDeviation);
      reportDelay(DelaySignalType.PROCESSED_VELOCITY, processedVelocityQualityMean, processedVelocityQualityStandardDeviation, processedVelocityDelayMean, processedVelocityDelayStandardDeviation);

      if (forceTrackingQualityMean < badCorrelation)
         logLevel = Level.SEVERE;
      else if (forceTrackingQualityMean < goodCorrelation)
         logLevel = Level.WARNING;
      else
         logLevel = Level.INFO;

      String forceTrackingQualityMeanFormatted = doubleFormat.format(forceTrackingQualityMean);
      String forceTrackingQualityStandardDeviationFormatted = doubleFormat.format(forceTrackingQualityStandardDeviation);
      logger.log(logLevel,
            "Force tracking quality for the joint: " + jointName + " equals " + forceTrackingQualityMeanFormatted + " second (+/-"
                  + forceTrackingQualityStandardDeviationFormatted
                  + "). Note: 0 means force control is probably not doing anything, and 1 force control tends to achieve the desired input.");

      if (forceTrackingDelayMean > badDelay)
         logLevel = Level.SEVERE;
      else if (forceTrackingDelayMean > goodDelay)
         logLevel = Level.WARNING;
      else
         logLevel = Level.INFO;

      String forceTrackingDelayMeanFormatted = doubleFormat.format(forceTrackingDelayMean);
      String forceTrackingDelayStandardDeviationFormatted = doubleFormat.format(forceTrackingDelayStandardDeviation);
      JointForceTrackingDelayLogRecord logRecord = new JointForceTrackingDelayLogRecord(logLevel, "Estimated force tracking delay for the joint: " + jointName
            + " equals " + forceTrackingDelayMeanFormatted + " second (+/-" + forceTrackingDelayStandardDeviationFormatted + ").");
      logger.log(logRecord);

      logLevel = Level.INFO;

      String frequencies = "";
      String velMags = "";
      String tauMags = "";
      String tauDMags = "";

      for (int i = 0; i < fourierAnalysisOutputSize; i++)
      {
         frequencies += doubleFormat2.format(fourierAnalysisFrequency[i]) + ", ";
         velMags += doubleFormat2.format(fourierAnalysisVelocityMagnitude[i]) + ", ";
         tauMags += doubleFormat2.format(fourierAnalysisTauMagnitude[i]) + ", ";
         tauDMags += doubleFormat2.format(fourierAnalysisTauDesiredMagnitude[i]) + ", ";
      }

      logger.log(logLevel, "Fourier analysis results. The desired joint position is a " + functionGeneratorMode + " with frequency of "
            + doubleFormat.format(functionGeneratorFrequency) + " Hz and amplitude of " + doubleFormat.format(functionGeneratorAmplitude) + " rad.");
      logger.log(logLevel, "Frequency spectrum      (Hz): " + frequencies);
      logger.log(logLevel, "Velocity magnitudes  (rad/s): " + velMags);
      logger.log(logLevel, "Tau magnitudes         (N.m): " + tauMags);
      logger.log(logLevel, "Tau desired magnitudes (N.m): " + tauDMags);
   }

   private void reportDelay(DelaySignalType signalType, double qualityMean, double qualityStandardDeviation, double delayMean, double delayStandardDeviation)
   {
      Level logLevel;
      if (qualityMean < badCorrelation)
         logLevel = Level.SEVERE;
      else if (qualityMean < goodCorrelation)
         logLevel = Level.WARNING;
      else
         logLevel = Level.INFO;

      String velocityQualityMeanFormatted = doubleFormat.format(qualityMean);
      String velocityQualityStandardDeviationFormatted = doubleFormat.format(qualityStandardDeviation);
      String signalName = signalType.getSignalName();
      logger.log(logLevel,
            StringUtils.capitalize(signalName) + " signal quality for the joint: " + jointName + " equals " + velocityQualityMeanFormatted + " second (+/-"
                  + velocityQualityStandardDeviationFormatted
                  + "). Note: 0 means raw position and " + signalName + " are completely inconsistent, and 1 they're perfectly matching.");

      if (delayMean > badDelay)
         logLevel = Level.SEVERE;
      else if (delayMean > goodDelay)
         logLevel = Level.WARNING;
      else
         logLevel = Level.INFO;

      String velocityDelayMeanFormatted = doubleFormat.format(delayMean);
      String velocityDelayStandardDeviationFormatted = doubleFormat.format(delayStandardDeviation);
      logger.log(signalType.createLogRecord(logLevel, StringUtils.capitalize(signalName) + " estimated delay for the joint: " + jointName + " equals " + velocityDelayMeanFormatted + " second (+/-"
            + velocityDelayStandardDeviationFormatted + ")."));
   }

   public boolean isDoneExportingData()
   {
      return isDoneExportingData.get();
   }
}
