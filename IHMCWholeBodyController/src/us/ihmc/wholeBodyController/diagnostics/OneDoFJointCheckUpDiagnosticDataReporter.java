package us.ihmc.wholeBodyController.diagnostics;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.logging.Level;
import java.util.logging.Logger;

import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.diagnostic.DiagnosticParameters;
import us.ihmc.sensorProcessing.diagnostic.OneDoFJointFourierAnalysis;
import us.ihmc.simulationconstructionset.util.math.functionGenerator.YoFunctionGenerator;
import us.ihmc.simulationconstructionset.util.math.functionGenerator.YoFunctionGeneratorMode;

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

   private final double velocityQualityMean;
   private final double velocityQualityStandardDeviation;
   private final double velocityDelayMean;
   private final double velocityDelayStandardDeviation;

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
         DoubleYoVariable velocityQualityMean, DoubleYoVariable velocityQualityStandardDeviation, DoubleYoVariable velocityDelayMean, DoubleYoVariable velocityDelayStandardDeviation,
         DoubleYoVariable forceTrackingQualityMean, DoubleYoVariable forceTrackingQualityStandardDeviation, DoubleYoVariable forceTrackingDelayMean, DoubleYoVariable forceTrackingDelayStandardDeviation,
         OneDoFJointFourierAnalysis fourierAnalysis, YoFunctionGenerator functionGenerator)
   {
      logger = Logger.getLogger(loggerName);

      jointName = joint.getName();

      badCorrelation = diagnosticParameters.getBadCorrelation();
      goodCorrelation = diagnosticParameters.getGoodCorrelation();
      badDelay = diagnosticParameters.getBadDelay();
      goodDelay = diagnosticParameters.getGoodDelay();

      this.velocityQualityMean = velocityQualityMean.getDoubleValue();
      this.velocityQualityStandardDeviation = velocityQualityStandardDeviation.getDoubleValue();
      this.velocityDelayMean = velocityDelayMean.getDoubleValue();
      this.velocityDelayStandardDeviation = velocityDelayStandardDeviation.getDoubleValue();

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

   private void reportCheckUpResults()
   {
      if (logger == null)
         return;

      Level logLevel;

      if (velocityQualityMean < badCorrelation)
         logLevel = Level.SEVERE;
      else if (velocityQualityMean < goodCorrelation)
         logLevel = Level.WARNING;
      else
         logLevel = Level.INFO;

      String velocityQualityMeanFormatted = doubleFormat.format(velocityQualityMean);
      String velocityQualityStandardDeviationFormatted = doubleFormat.format(velocityQualityStandardDeviation);
      logger.log(logLevel,
            "Velocity signal quality for the joint: " + jointName + " equals " + velocityQualityMeanFormatted + "(+/-"
                  + velocityQualityStandardDeviationFormatted
                  + "). Note: 0 means position and velocity are completely inconsistent, and 1 they're perfectly matching.");

      if (velocityDelayMean > badDelay)
         logLevel = Level.SEVERE;
      else if (velocityDelayMean > goodDelay)
         logLevel = Level.WARNING;
      else
         logLevel = Level.INFO;

      String velocityDelayMeanFormatted = doubleFormat.format(velocityDelayMean);
      String velocityDelayStandardDeviationFormatted = doubleFormat.format(velocityDelayStandardDeviation);
      logger.log(logLevel, "Estimated velocity delay for the joint: " + jointName + " equals " + velocityDelayMeanFormatted + "(+/-"
            + velocityDelayStandardDeviationFormatted + ").");

      if (forceTrackingQualityMean < badCorrelation)
         logLevel = Level.SEVERE;
      else if (forceTrackingQualityMean < goodCorrelation)
         logLevel = Level.WARNING;
      else
         logLevel = Level.INFO;

      String forceTrackingQualityMeanFormatted = doubleFormat.format(forceTrackingQualityMean);
      String forceTrackingQualityStandardDeviationFormatted = doubleFormat.format(forceTrackingQualityStandardDeviation);
      logger.log(logLevel,
            "Force tracking quality for the joint: " + jointName + " equals " + forceTrackingQualityMeanFormatted + "(+/-"
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
      logger.log(logLevel, "Estimated force tracking delay for the joint: " + jointName + " equals " + forceTrackingDelayMeanFormatted + "(+/-"
            + forceTrackingDelayStandardDeviationFormatted + ").");

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
      logger.log(logLevel, "Frequency spectrum     (Hz): " + frequencies);
      logger.log(logLevel, "Velocity magnitudes (rad/s): " + velMags);
      logger.log(logLevel, "Tau magnitudes        (N.m): " + tauMags);
      logger.log(logLevel, "Tau desired spectrum  (N.m): " + tauDMags);
   }

   public boolean isDoneExportingData()
   {
      return isDoneExportingData.get();
   }
}
