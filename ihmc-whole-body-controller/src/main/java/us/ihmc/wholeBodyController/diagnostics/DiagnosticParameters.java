package us.ihmc.wholeBodyController.diagnostics;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.GroupParameter;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;
import us.ihmc.sensorProcessing.outputData.JointDesiredBehaviorReadOnly;

public abstract class DiagnosticParameters
{
   public DiagnosticParameters()
   {
   }

   public abstract List<GroupParameter<JointDesiredBehaviorReadOnly>> getDesiredJointBehaviors();

   public abstract WholeBodySetpointParameters getDiagnosticSetpoints();

   public boolean enableLogging()
   {
      return true;
   }

   public abstract double getInitialJointSplineDuration();

   /**
    * Override this method to limit the scope of the automated diagnostic. If not overridden, the
    * diagnostic will be attempted on all the joints.
    * 
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
      return 10.0;
   }

   public double getDelayEstimatorMaximumLag()
   {
      return 0.025;
   }

   public double getDelayEstimatorMaximumLead()
   {
      return 0.025;
   }

   public double getDelayEstimatorObservationWindow()
   {
      return 0.16;
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
      return 0.05;
   }

   public double getCheckUpOscillationPositionFrequency()
   {
      return 5.0; // 10.0Hz seems to give better delay estimation
   }

   public double getJointCheckUpDuration()
   {
      return 2.0;
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

   public String getPelvisIMUName()
   {
      return null;
   }

   public boolean doIdleControlUntilRobotIsAlive()
   {
      return false;
   }

   public double getIdleQdMax()
   {
      return 0.3;
   }

   public double getIdleQddMax()
   {
      return 3.0;
   }

   public double getIdleTauMax()
   {
      return 10.0;
   }
}
