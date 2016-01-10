package us.ihmc.sensorProcessing.diagnostic;

import java.util.ArrayList;
import java.util.List;

public class DiagnosticParameters
{
   public enum DiagnosticEnvironment {RUNTIME_CONTROLLER, RUNTIME_EXTERNAL_MODULE, OFFLINE_LOG};
   protected final DiagnosticEnvironment diagnosticEnvironment;

   public DiagnosticParameters(DiagnosticEnvironment diagnosticEnvironment)
   {
      this.diagnosticEnvironment = diagnosticEnvironment;
   }

   /**
    * Override this method to limit the scope of the automated diagnostic.
    * If not overridden, the diagnostic will be attempted on all the joints.
    * @return
    */
   public List<String> getJointsToIgnoreDuringDiagnostic()
   {
      return new ArrayList<>();
   }

   public double getDelayEstimatorIntputSignalsSMAWindow()
   {
      return 0.01;
   }

   public double getDelayEstimatorFilterBreakFrequency()
   {
      switch (diagnosticEnvironment)
      {
      case RUNTIME_CONTROLLER:
         return 10.0;
      case OFFLINE_LOG:
      case RUNTIME_EXTERNAL_MODULE:
      default:
         return 0.16;
      }
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

   public double getCheckUpOscillationPositionAmplitude()
   {
      return 0.10;
   }

   public double getCheckUpOscillationPositionFrequency()
   {
      return 5.0;
   }

   public double getJointCheckUpDuration()
   {
      return 3.0;
   }

   public double getBadCorrelation()
   {
      return 0.80;
   }

   public double getGoodCorrelation()
   {
      return 0.90;
   }

   public double getBadDelay()
   {
      return 0.02;
   }

   public double getGoodDelay()
   {
      return 0.01;
   }
}
