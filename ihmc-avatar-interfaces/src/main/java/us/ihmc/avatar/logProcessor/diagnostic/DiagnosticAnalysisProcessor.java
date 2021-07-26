package us.ihmc.avatar.logProcessor.diagnostic;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.logProcessor.LogDataProcessorFunction;
import us.ihmc.avatar.logProcessor.LogDataProcessorHelper;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.diagnostic.DelayEstimatorBetweenTwoSignals;
import us.ihmc.sensorProcessing.diagnostic.DiagnosticUpdatable;
import us.ihmc.sensorProcessing.diagnostic.OneDoFJointFourierAnalysis;
import us.ihmc.sensorProcessing.diagnostic.OrientationAngularVelocityConsistencyChecker;
import us.ihmc.sensorProcessing.diagnostic.PositionVelocity1DConsistencyChecker;
import us.ihmc.sensorProcessing.diagnostic.PositionVelocity3DConsistencyChecker;
import us.ihmc.wholeBodyController.diagnostics.DiagnosticParameters;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.registry.YoVariableHolder;
import us.ihmc.yoVariables.variable.YoDouble;

public class DiagnosticAnalysisProcessor implements LogDataProcessorFunction
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final LogDataProcessorHelper logDataProcessorHelper;
   private final List<DiagnosticUpdatable> diagnosticUpdatables = new ArrayList<>();

   private final FullHumanoidRobotModel fullRobotModel;
   private final YoVariableHolder logYoVariableHolder;
   private final double dt;

   private DiagnosticParameters diagnosticParameters;

   public DiagnosticAnalysisProcessor(LogDataProcessorHelper logDataProcessorHelper, DRCRobotModel drcRobotModel)
   {
      this.logDataProcessorHelper = logDataProcessorHelper;
      diagnosticParameters = drcRobotModel.getDiagnoticParameters();
      fullRobotModel = logDataProcessorHelper.getFullRobotModel();
      dt = drcRobotModel.getEstimatorDT();
      logYoVariableHolder = logDataProcessorHelper.getLogYoVariableHolder();
   }

   public void addJointFourierAnalyses(String qd_prefix, String qd_suffix, String tau_prefix, String tau_suffix, String tau_d_prefix, String tau_d_suffix)
   {
      OneDoFJointBasics[] oneDoFJoints = fullRobotModel.getOneDoFJoints();
      double fftObservationWindow = diagnosticParameters.getFFTObservationWindow();

      for (OneDoFJointBasics joint : oneDoFJoints)
      {
         String jointName = joint.getName();
         YoDouble qd = (YoDouble) logYoVariableHolder.findVariable(qd_prefix + jointName + qd_suffix);
         YoDouble tau = (YoDouble) logYoVariableHolder.findVariable(tau_prefix + jointName + tau_suffix);
         YoDouble tau_d = (YoDouble) logYoVariableHolder.findVariable(tau_d_prefix + jointName + tau_d_suffix);

         if (qd == null)
         {
            System.err.println("Could not find the find the velocity variable for the joint: " + jointName);
            continue;
         }

         if (tau == null)
         {
            System.err.println("Could not find the find the tau variable for the joint: " + jointName);
            continue;
         }

         if (tau_d == null)
         {
            System.err.println("Could not find the find the tau desired variable for the joint: " + jointName);
            continue;
         }

         OneDoFJointFourierAnalysis online1dSignalFrequencyAnalysis = new OneDoFJointFourierAnalysis(joint, fftObservationWindow, dt, qd, tau, tau_d, registry);
         diagnosticUpdatables.add(online1dSignalFrequencyAnalysis);
      }
   }

   public void addJointConsistencyCheckers()
   {
      OneDoFJointBasics[] oneDoFJoints = fullRobotModel.getOneDoFJoints();

      for (OneDoFJointBasics joint : oneDoFJoints)
      {
         String jointName = joint.getName();
         YoDouble rawJointPosition = (YoDouble) logYoVariableHolder.findVariable("raw_q_" + jointName);
         YoDouble rawJointVelocity = (YoDouble) logYoVariableHolder.findVariable("raw_qd_" + jointName);
         YoDouble processedJointPosition = (YoDouble) logYoVariableHolder.findVariable("q_" + jointName);
         YoDouble processedJointVelocity = (YoDouble) logYoVariableHolder.findVariable("qd_" + jointName);

         if (rawJointPosition == null)
         {
            System.err.println("Could not find the find the raw position variable for the joint: " + jointName);
            continue;
         }

         if (rawJointVelocity == null)
         {
            System.err.println("Could not find the find the raw velocity variable for the joint: " + jointName);
            continue;
         }

         if (processedJointPosition == null)
         {
            System.err.println("Could not find the find the processed position variable for the joint: " + jointName);
            continue;
         }

         if (processedJointVelocity == null)
         {
            System.err.println("Could not find the find the processed velocity variable for the joint: " + jointName);
            continue;
         }

         diagnosticUpdatables.add(new PositionVelocity1DConsistencyChecker(jointName, rawJointPosition, rawJointVelocity, processedJointPosition, processedJointVelocity, dt, registry));
      }
   }

   public void addIMUConsistencyCheckers(String q_prefix, String q_suffix, String qd_prefix, String qd_suffix, boolean performAnalysisInBodyFrame)
   {
      IMUDefinition[] imuDefinitions = fullRobotModel.getIMUDefinitions();

      for (IMUDefinition imuDefinition : imuDefinitions)
      {
         String imuName = imuDefinition.getName();
         ReferenceFrame bodyFrame = imuDefinition.getRigidBody().getBodyFixedFrame();
         ReferenceFrame imuFrame = imuDefinition.getIMUFrame();

         YoFrameQuaternion q = logDataProcessorHelper.findYoFrameQuaternion(q_prefix, imuName + q_suffix, ReferenceFrame.getWorldFrame());
         YoFrameVector3D qd = logDataProcessorHelper.findYoFrameVector(qd_prefix, imuName + qd_suffix, imuFrame);

         if (q == null)
         {
            System.err.println("Could not find the find the position variable for the IMU: " + imuName);
            continue;
         }

         if (qd == null)
         {
            System.err.println("Could not find the find the velocity variable for the IMU: " + imuName);
            continue;
         }

         ReferenceFrame referenceFrameUsedForComparison = performAnalysisInBodyFrame ? bodyFrame : imuFrame;
         diagnosticUpdatables.add(new OrientationAngularVelocityConsistencyChecker(imuName, q, qd, referenceFrameUsedForComparison, dt, registry));
      }
   }

   public void addForceTrackingDelayEstimators(String tau_prefix, String tau_suffix, String tau_d_prefix, String tau_d_suffix)
   {
      OneDoFJointBasics[] oneDoFJoints = fullRobotModel.getOneDoFJoints();

      for (OneDoFJointBasics joint : oneDoFJoints)
      {
         String jointName = joint.getName();

         YoDouble tau = (YoDouble) logYoVariableHolder.findVariable(tau_prefix + jointName + tau_suffix);
         YoDouble tau_d = (YoDouble) logYoVariableHolder.findVariable(tau_d_prefix + jointName + tau_d_suffix);

         if (tau == null)
         {
            System.err.println("Could not find the find the tau variable for the joint: " + jointName);
            continue;
         }

         if (tau_d == null)
         {
            System.err.println("Could not find the find the tau desired variable for the joint: " + jointName);
            continue;
         }

         diagnosticUpdatables.add(new DelayEstimatorBetweenTwoSignals(jointName + "ForceTracking", tau_d, tau, dt, registry));
      }
   }

   public void addCenterOfMassConsistencyChecker()
   {
      YoFramePoint3D com = logDataProcessorHelper.findYoFramePoint("estimatedCenterOfMassPosition", ReferenceFrame.getWorldFrame());
      YoFrameVector3D comVelocity = logDataProcessorHelper.findYoFrameVector("estimatedCenterOfMassVelocity", ReferenceFrame.getWorldFrame());

      if (com == null)
      {
         System.err.println("Could not find the YoFramePoint for the center of mass position");
         return;
      }

      if (comVelocity == null)
      {
         System.err.println("Could not find the YoFrameVector for the center of mass velocity");
         return;
      }

      diagnosticUpdatables.add(new PositionVelocity3DConsistencyChecker("centerOfMass", com, comVelocity, dt, registry));
   }

   @Override
   public void processDataAtControllerRate()
   {
   }

   private boolean firstTick = true;

   @Override
   public void processDataAtStateEstimatorRate()
   {
      if (firstTick)
      {
         for (DiagnosticUpdatable diagnosticUpdatable : diagnosticUpdatables)
            diagnosticUpdatable.enable();
         firstTick = false;
      }

      logDataProcessorHelper.update();
      for (DiagnosticUpdatable diagnosticUpdatable : diagnosticUpdatables)
         diagnosticUpdatable.update();
   }

   @Override
   public YoRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return null;
   }
}
