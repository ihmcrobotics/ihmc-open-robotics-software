package us.ihmc.wholeBodyController.diagnostics;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.logging.Logger;

import us.ihmc.commonWalkingControlModules.configurations.ParameterTools;
import us.ihmc.commons.MathTools;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.ParameterizedPDController;
import us.ihmc.robotics.math.trajectories.OneDoFJointQuinticTrajectoryGenerator;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.sensorProcessing.outputData.JointDesiredBehaviorReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.wholeBodyController.diagnostics.utils.DiagnosticTask;
import us.ihmc.wholeBodyController.diagnostics.utils.DiagnosticTaskExecutor;
import us.ihmc.wholeBodyController.diagnostics.utils.WaitDiagnosticTask;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class AutomatedDiagnosticAnalysisController implements RobotController
{
   private Logger logger;

   private final YoRegistry registry = new YoRegistry(getName());

   private final YoDouble yoTime;

   private final DiagnosticTaskExecutor diagnosticTaskExecutor;

   private final List<JointController> jointControllers = new ArrayList<>();
   private final DoubleProvider trajectoryTimeProvider;

   private final ArrayDeque<DiagnosticDataReporter> dataReportersToExecute = new ArrayDeque<>();
   private DiagnosticDataReporter diagnosticDataReporterRunning = null;

   private final YoBoolean doIdleControl = new YoBoolean("doIdleControl", registry);
   private final YoDouble qdMaxIdle = new YoDouble("qdMaxIdle", registry);
   private final YoDouble qddMaxIdle = new YoDouble("qddMaxIdle", registry);
   private final YoDouble tauMaxIdle = new YoDouble("tauMaxIdle", registry);

   private final YoBoolean isDiagnosticComplete = new YoBoolean("isDiagnosticComplete", registry);
   private final YoBoolean robotIsAlive = new YoBoolean("robotIsAlive", registry);
   private final YoDouble startTime = new YoDouble("diagnosticControllerStartTime", registry);

   private final double controlDT;

   private final ExecutionTimer timer = new ExecutionTimer(getClass().getSimpleName() + "Timer", registry);

   public AutomatedDiagnosticAnalysisController(DiagnosticControllerToolbox toolbox, YoRegistry parentRegistry)
   {
      this.yoTime = toolbox.getYoTime();
      this.controlDT = toolbox.getDT();

      diagnosticTaskExecutor = new DiagnosticTaskExecutor("highLevelTaskExecutor", yoTime, registry);

      FullHumanoidRobotModel fullRobotModel = toolbox.getFullRobotModel();
      JointDesiredOutputList lowLevelOutput = toolbox.getLowLevelOutput();
      DiagnosticParameters diagnosticParameters = toolbox.getDiagnosticParameters();

      doIdleControl.set(diagnosticParameters.doIdleControlUntilRobotIsAlive());
      qdMaxIdle.set(diagnosticParameters.getIdleQdMax());
      qddMaxIdle.set(diagnosticParameters.getIdleQddMax());
      tauMaxIdle.set(diagnosticParameters.getIdleTauMax());

      trajectoryTimeProvider = () -> diagnosticParameters.getInitialJointSplineDuration();
      submitDiagnostic(new WaitDiagnosticTask(trajectoryTimeProvider.getValue()));

      Map<String, JointDesiredBehaviorReadOnly> jointBehaviorMap = new HashMap<>();
      ParameterTools.extractJointBehaviorMap("NoLoad", toolbox.getDiagnosticParameters().getDesiredJointBehaviors(), jointBehaviorMap, registry);

      if (diagnosticParameters.enableLogging())
      {
         diagnosticTaskExecutor.setupForLogging();
         setupForLogging();
      }

      for (OneDoFJointBasics joint : fullRobotModel.getOneDoFJoints())
      {
         String jointName = joint.getName();

         JointDesiredOutput jointDesiredOutput = lowLevelOutput.getJointDesiredOutput(joint);
         JointDesiredBehaviorReadOnly jointDesiredBehavior = jointBehaviorMap.get(jointName);

         if (jointDesiredBehavior == null)
         {
            String msg = "No desired behavior for the joint: " + jointName + ", joint will not be controlled.";

            if (logger != null)
               logger.info(msg);
            else
               LogTools.info(msg);
            continue;
         }

         double setpoint = diagnosticParameters.getDiagnosticSetpoints().getSetpoint(jointName);
         jointControllers.add(new JointController(joint, jointDesiredOutput, jointDesiredBehavior, setpoint));

      }

      parentRegistry.addChild(registry);
   }

   public void setRobotIsAlive(boolean isAlive)
   {
      robotIsAlive.set(isAlive);
   }

   public void setupForLogging()
   {
      logger = Logger.getLogger(getName());
   }

   public void submitDiagnostic(DiagnosticTask diagnosticTask)
   {
      if (logger != null)
         logger.info("Diagnostic task: " + diagnosticTask.getName() + " has been submitted to the controller.");
      diagnosticTaskExecutor.submit(diagnosticTask);
   }

   @Override
   public void initialize()
   {
      for (int i = 0; i < jointControllers.size(); i++)
      {
         jointControllers.get(i).initialize();
      }

      startTime.set(yoTime.getDoubleValue());

      hasBeenInitialized = true;
   }

   private boolean hasBeenInitialized = false;

   @Override
   public void doControl()
   {
      timer.startMeasurement();

      if (!hasBeenInitialized)
      {
         initialize();
         return;
      }

      if (!robotIsAlive.getBooleanValue())
      {
         initialize();
         if (doIdleControl.getBooleanValue())
            doIdleControl();
         return;
      }

      diagnosticTaskExecutor.doControl();
      DiagnosticTask currentTask = diagnosticTaskExecutor.getCurrentTask();
      double timeInTrajectory = yoTime.getDoubleValue() - startTime.getDoubleValue();

      for (int i = 0; i < jointControllers.size(); i++)
      {
         jointControllers.get(i).doControl(currentTask, timeInTrajectory);
      }

      if (currentTask != null)
         currentTask.getDataReporterToRun(dataReportersToExecute);

      handleDataReporters();

      boolean isDone = diagnosticTaskExecutor.isDone();
      if (isDone && !isDiagnosticComplete.getBooleanValue())
      {
         if (logger != null)
         {
            logger.info("---------------------------------------------------");
            logger.info("               Diagnostic complete.                ");
            logger.info("---------------------------------------------------");
         }
      }
      isDiagnosticComplete.set(isDone);

      timer.stopMeasurement();
   }

   private void doIdleControl()
   {
      for (int i = 0; i < jointControllers.size(); i++)
      {
         jointControllers.get(i).doIdleControl();
      }
   }

   private void handleDataReporters()
   {
      if (dataReportersToExecute.isEmpty() && diagnosticDataReporterRunning == null)
         return;

      if (diagnosticDataReporterRunning == null || diagnosticDataReporterRunning.isDoneExportingData())
      {
         diagnosticDataReporterRunning = null;

         if (dataReportersToExecute.isEmpty())
            return;

         diagnosticDataReporterRunning = dataReportersToExecute.poll();
         Thread thread = new Thread(diagnosticDataReporterRunning, "IHMC-AutomatedDiagnosticReporter");
         thread.setPriority(Thread.MIN_PRIORITY);
         thread.start();
      }
   }

   public boolean isDiagnosticDone()
   {
      return isDiagnosticComplete.getBooleanValue();
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return getClass().getSimpleName();
   }

   @Override
   public String getDescription()
   {
      return getName();
   }

   private class JointController
   {
      private final OneDoFJointBasics joint;
      private final JointDesiredOutputBasics desiredOutput;

      private final ParameterizedPDController pdController;
      private final YoDouble desiredPosition;
      private final YoDouble desiredVelocity;
      private final YoDouble desiredTau;

      private final OneDoFJointQuinticTrajectoryGenerator trajectory;

      public JointController(OneDoFJointBasics joint, JointDesiredOutputBasics desiredOutput, JointDesiredBehaviorReadOnly desiredBehavior, double setpoint)
      {
         this.joint = joint;
         this.desiredOutput = desiredOutput;
         String jointName = joint.getName();

         DoubleProvider kp = () -> desiredBehavior.getStiffness();
         DoubleProvider kd = () -> desiredBehavior.getDamping();
         pdController = new ParameterizedPDController(kp, kd, jointName, registry);
         desiredPosition = new YoDouble("q_d_" + jointName, registry);
         desiredVelocity = new YoDouble("qd_d_" + jointName, registry);
         desiredTau = new YoDouble("tau_d_" + jointName, registry);

         trajectory = new OneDoFJointQuinticTrajectoryGenerator(jointName, joint, trajectoryTimeProvider, registry);
         trajectory.setFinalPosition(setpoint);
      }

      public void initialize()
      {
         trajectory.initialize(joint.getQ(), 0.0);
      }

      public void doControl(DiagnosticTask currentTask, double timeInTrajectory)
      {
         double desiredJointPositionOffset = 0.0;
         double desiredJointVelocityOffset = 0.0;
         double desiredJointTauOffset = 0.0;

         if (currentTask != null)
         {
            desiredJointPositionOffset = currentTask.getDesiredJointPositionOffset(joint);
            desiredJointVelocityOffset = currentTask.getDesiredJointVelocityOffset(joint);
            desiredJointTauOffset = currentTask.getDesiredJointTauOffset(joint);
         }

         trajectory.compute(timeInTrajectory);

         desiredPosition.set(trajectory.getValue() + desiredJointPositionOffset);
         desiredVelocity.set(trajectory.getVelocity() + desiredJointVelocityOffset);

         double q = joint.getQ();
         double qDesired = desiredPosition.getDoubleValue();
         double qd = joint.getQd();
         double qdDesired = desiredVelocity.getDoubleValue();
         double tauDesired = pdController.compute(q, qDesired, qd, qdDesired) + desiredJointTauOffset;
         desiredOutput.setDesiredTorque(tauDesired);
         desiredOutput.setDesiredPosition(qDesired);
         desiredOutput.setDesiredVelocity(qdDesired);
         desiredTau.set(tauDesired);
      }

      public void doIdleControl()
      {
         trajectory.compute(trajectoryTimeProvider.getValue());

         desiredPosition.set(trajectory.getValue());
         desiredVelocity.set(0.0);

         double q = joint.getQ();
         double qDesired = desiredPosition.getDoubleValue();
         double qd = joint.getQd();
         double qdDesired = desiredVelocity.getDoubleValue();
         double tauDesired = pdController.compute(q, qDesired, qd, qdDesired);

         double qdMax = qdMaxIdle.getDoubleValue();
         double qddMax = qddMaxIdle.getDoubleValue();
         double tauMax = tauMaxIdle.getDoubleValue();

         qDesired = q + MathTools.clamp(qDesired - q, qdMax * controlDT);
         qdDesired = qd + MathTools.clamp(qdDesired - qd, qddMax * controlDT);
         tauDesired = MathTools.clamp(tauDesired, tauMax);
         desiredOutput.setDesiredTorque(tauDesired);
         desiredOutput.setDesiredPosition(qDesired);
         desiredOutput.setDesiredVelocity(qdDesired);
         desiredTau.set(tauDesired);
      }
   }
}
