package us.ihmc.darpaRoboticsChallenge.logProcessor.diagnostic;

import java.util.ArrayList;
import java.util.LinkedHashMap;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.logProcessor.LogDataProcessorFunction;
import us.ihmc.darpaRoboticsChallenge.logProcessor.LogDataProcessorHelper;
import us.ihmc.robotics.dataStructures.YoVariableHolder;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.sensorProcessing.diagnostic.DelayEstimatorBetweenTwoSignals;
import us.ihmc.sensorProcessing.diagnostic.DiagnosticParameters;
import us.ihmc.sensorProcessing.diagnostic.Online1DSignalFourierAnalysis;
import us.ihmc.sensorProcessing.diagnostic.PositionVelocity1DConsistencyChecker;
import us.ihmc.sensorProcessing.diagnostic.PositionVelocity3DConsistencyChecker;
import us.ihmc.sensorProcessing.diagnostic.DiagnosticParameters.DiagnosticEnvironment;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class DiagnosticAnalysisProcessor implements LogDataProcessorFunction
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final LogDataProcessorHelper logDataProcessorHelper;
   private final ArrayList<OneDoFJoint> analyzedJoints = new ArrayList<>();
   private final LinkedHashMap<OneDoFJoint, Online1DSignalFourierAnalysis> jointVelocityFFTs = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJoint, PositionVelocity1DConsistencyChecker> jointVelocityCheck = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJoint, DelayEstimatorBetweenTwoSignals> jointForceControlDelay = new LinkedHashMap<>();

   private final PositionVelocity3DConsistencyChecker centerOfMassCheck;

   public DiagnosticAnalysisProcessor(LogDataProcessorHelper logDataProcessorHelper, DRCRobotModel drcRobotModel)
   {
      this.logDataProcessorHelper = logDataProcessorHelper;
      DiagnosticParameters diagnosticParameters = new DiagnosticParameters(DiagnosticEnvironment.OFFLINE_LOG);
      FullHumanoidRobotModel fullRobotModel = logDataProcessorHelper.getFullRobotModel();
      double dt = drcRobotModel.getEstimatorDT();
      YoVariableHolder logYoVariableHolder = logDataProcessorHelper.getLogYoVariableHolder();

      double delayEstimatorMaxAbsoluteLead = diagnosticParameters.getDelayEstimatorMaximumLead();
      double delayEstimatorMaxAbsoluteLag = diagnosticParameters.getDelayEstimatorMaximumLag();
      double delayEstimatorObservationWindow = diagnosticParameters.getDelayEstimatorObservationWindow();
      double delayEstimatorIntputSignalsSMAWindow = diagnosticParameters.getDelayEstimatorIntputSignalsSMAWindow();
      double delayEstimatorFilterBreakFrequency = diagnosticParameters.getDelayEstimatorFilterBreakFrequency();

      for (RobotSide robotSide : RobotSide.values)
      {
         for (LegJointName legJointName : fullRobotModel.getRobotSpecificJointNames().getLegJointNames())
         {
            OneDoFJoint joint = fullRobotModel.getLegJoint(robotSide, legJointName);
            String jointName = joint.getName();

            analyzedJoints.add(joint);
            double fftObservationWindow = diagnosticParameters.getFFTObservationWindow();
            Online1DSignalFourierAnalysis online1dSignalFrequencyAnalysis = new Online1DSignalFourierAnalysis(jointName, fftObservationWindow, dt, registry);
            online1dSignalFrequencyAnalysis.setMagnitudeFilterBreakFrequency(diagnosticParameters.getFFTMagnitudeFilterBreakFrequency());
            online1dSignalFrequencyAnalysis.setFrequencyGlitchFilterWindow(diagnosticParameters.getFFTFrequencyGlitchFilterWindow());
            online1dSignalFrequencyAnalysis.setMinimumMagnitude(diagnosticParameters.getFFTMinimumMagnitude());
            
            jointVelocityFFTs.put(joint, online1dSignalFrequencyAnalysis);
            DoubleYoVariable q = (DoubleYoVariable) logYoVariableHolder.getVariable("raw_q_" + jointName);
            DoubleYoVariable qd = (DoubleYoVariable) logYoVariableHolder.getVariable("qd_" + jointName);
            PositionVelocity1DConsistencyChecker consistencyChecker = new PositionVelocity1DConsistencyChecker(jointName + "Check", q, qd, dt, registry);
            consistencyChecker.enable();

            consistencyChecker.setDelayEstimationParameters(delayEstimatorMaxAbsoluteLead, delayEstimatorMaxAbsoluteLag, delayEstimatorObservationWindow);
            consistencyChecker.setInputSignalsSMAWindow(delayEstimatorIntputSignalsSMAWindow);
            consistencyChecker.setDelayEstimatorAlphaFilterBreakFrequency(delayEstimatorFilterBreakFrequency);
            
            jointVelocityCheck.put(joint, consistencyChecker);

            DoubleYoVariable tauDesired = (DoubleYoVariable) logYoVariableHolder.getVariable("ll_out_" + jointName + "_f_d");
            DoubleYoVariable tau = (DoubleYoVariable) logYoVariableHolder.getVariable("ll_in_" + jointName + "_f");
            DelayEstimatorBetweenTwoSignals forceTrackingDelay = new DelayEstimatorBetweenTwoSignals(jointName + "ForceTrackingDelay", tauDesired, tau, dt, registry);
            forceTrackingDelay.enable();
            forceTrackingDelay.setAlphaFilterBreakFrequency(delayEstimatorFilterBreakFrequency);
            forceTrackingDelay.setEstimationParameters(delayEstimatorMaxAbsoluteLead, delayEstimatorMaxAbsoluteLag, delayEstimatorObservationWindow);
            
            jointForceControlDelay.put(joint, forceTrackingDelay);
         }
      }

      YoFramePoint com = logDataProcessorHelper.findYoFramePoint("estimatedCenterOfMassPosition", ReferenceFrame.getWorldFrame());
      YoFrameVector comVelocity = logDataProcessorHelper.findYoFrameVector("estimatedCenterOfMassVelocity", ReferenceFrame.getWorldFrame());
      centerOfMassCheck = new PositionVelocity3DConsistencyChecker("centerOfMass", com, comVelocity, dt, registry);
      centerOfMassCheck.enable();
   }

   @Override
   public void processDataAtControllerRate()
   {
   }

   private final AlphaFilteredYoVariable averageDiagnosticDT = new AlphaFilteredYoVariable("averageDiagnosticDT", registry, 0.995);

   @Override
   public void processDataAtStateEstimatorRate()
   {
      logDataProcessorHelper.update();
      long startTime = System.nanoTime();
      for (int i = 0; i < analyzedJoints.size(); i++)
      {
         jointVelocityFFTs.get(analyzedJoints.get(i)).update(analyzedJoints.get(i).getQd());
         jointVelocityCheck.get(analyzedJoints.get(i)).update();
         jointForceControlDelay.get(analyzedJoints.get(i)).update();
      }
      centerOfMassCheck.update();
      long endTime = System.nanoTime();
      averageDiagnosticDT.update(TimeTools.nanoSecondstoSeconds(endTime - startTime));
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return null;
   }
}
