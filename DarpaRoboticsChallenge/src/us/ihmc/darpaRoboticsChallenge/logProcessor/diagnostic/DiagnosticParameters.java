package us.ihmc.darpaRoboticsChallenge.logProcessor.diagnostic;

public class DiagnosticParameters
{
   public enum DiagnosticEnvironment {RUNTIME_CONTROLLER, RUNTIME_EXTERNAL_MODULE, OFFLINE_LOG};

   public DiagnosticParameters()
   {
   }

   public double getDelayEstimatorIntputSignalsSMAWindow()
   {
      return 0.01;
   }

   public double getDelayEstimatorFilterBreakFrequency()
   {
      return 0.16;
   }

   public double getDelayEstimatorMaximumLag()
   {
      return 0.025;
   }

   public double getDelayEstimatorMaximumLead()
   {
      return 0.00;
   }

   public double getDelayEstimatorObservationWindow()
   {
      return 0.10;
   }

   public double getFFTMagnitudeFilterBreakFrequency()
   {
      return 0.16;
   }

   public double getFFTFrequencyGlitchFilterWindow()
   {
      return 0.01;
   }

   public double getFFTMinimumMagnitude()
   {
      return 1.0e-7;
   }

   public double getFFTObservationWindow()
   {
      return 1.0;
   }
}
