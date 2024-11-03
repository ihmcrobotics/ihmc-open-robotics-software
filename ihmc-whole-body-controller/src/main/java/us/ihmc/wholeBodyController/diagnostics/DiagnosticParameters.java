package us.ihmc.wholeBodyController.diagnostics;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.GroupParameter;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.outputData.JointDesiredBehaviorReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
import us.ihmc.sensorProcessing.stateEstimation.SensorProcessingConfiguration;
import us.ihmc.wholeBodyController.diagnostics.PelvisIMUCheckUpDiagnosticTask.PelvisIMUCheckUpParameters;

public abstract class DiagnosticParameters
{
   public static PelvisIMUCheckUpParameters defaultPelvisIMUCheckUp(HumanoidRobotSensorInformation sensorInformation, HumanoidJointNameMap jointMap)
   {
      PelvisIMUCheckUpParameters parameters = new PelvisIMUCheckUpParameters();
      parameters.setPelvisIMUName(sensorInformation.getPrimaryBodyImu());
      parameters.setSpineJointNames(jointMap.getSpineJointName(SpineJointName.SPINE_YAW),
                                    jointMap.getSpineJointName(SpineJointName.SPINE_PITCH),
                                    jointMap.getSpineJointName(SpineJointName.SPINE_ROLL));
      parameters.setLeftHipJointNames(jointMap.getLegJointName(RobotSide.LEFT, LegJointName.HIP_YAW),
                                      jointMap.getLegJointName(RobotSide.LEFT, LegJointName.HIP_PITCH),
                                      jointMap.getLegJointName(RobotSide.LEFT, LegJointName.HIP_ROLL));
      parameters.setRightHipJointNames(jointMap.getLegJointName(RobotSide.RIGHT, LegJointName.HIP_YAW),
                                       jointMap.getLegJointName(RobotSide.RIGHT, LegJointName.HIP_PITCH),
                                       jointMap.getLegJointName(RobotSide.RIGHT, LegJointName.HIP_ROLL));
      return parameters;
   }

   private DiagnosticSensorProcessingConfiguration sensorProcessingConfiguration = null;

   public DiagnosticParameters()
   {
   }

   public DiagnosticSensorProcessingConfiguration getOrCreateSensorProcessingConfiguration(SensorProcessingConfiguration sensorProcessingConfiguration,
                                                                                           JointDesiredOutputListReadOnly jointDesiredOutput)
   {
      if (this.sensorProcessingConfiguration == null)
      {
         if (sensorProcessingConfiguration == null || jointDesiredOutput == null)
            throw new IllegalStateException("The configuration has not been created yet. It needs to be created and used to configure the SensorProcessing first.");
         this.sensorProcessingConfiguration = new DiagnosticSensorProcessingConfiguration(this, sensorProcessingConfiguration, jointDesiredOutput);
      }
      return this.sensorProcessingConfiguration;
   }

   public abstract void scheduleCheckUps(AutomatedDiagnosticConfiguration configuration);

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
