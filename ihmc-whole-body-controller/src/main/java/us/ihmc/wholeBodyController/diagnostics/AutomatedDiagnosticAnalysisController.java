package us.ihmc.wholeBodyController.diagnostics;

import java.util.ArrayDeque;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;
import java.util.logging.Logger;

import us.ihmc.commonWalkingControlModules.configurations.ParameterTools;
import us.ihmc.commons.MathTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.ParameterizedPDController;
import us.ihmc.robotics.math.trajectories.OneDoFJointQuinticTrajectoryGenerator;
import us.ihmc.sensorProcessing.outputData.JointDesiredBehaviorReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.tools.lists.PairList;
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

   private final PairList<OneDoFJointBasics, JointDesiredOutputBasics> controlledJoints = new PairList<>();
   private final JointDesiredBehaviorReadOnly[] jointDesiredBehaviors;
   private final Map<OneDoFJointBasics, ParameterizedPDController> jointPDControllerMap = new LinkedHashMap<>();
   private final Map<OneDoFJointBasics, YoDouble> jointDesiredPositionMap = new LinkedHashMap<>();
   private final Map<OneDoFJointBasics, YoDouble> jointDesiredVelocityMap = new LinkedHashMap<>();
   private final Map<OneDoFJointBasics, YoDouble> jointDesiredTauMap = new LinkedHashMap<>();

   private final DoubleProvider trajectoryTimeProvider;
   private final Map<OneDoFJointBasics, OneDoFJointQuinticTrajectoryGenerator> jointTrajectories = new LinkedHashMap<>();

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

      for (OneDoFJointBasics joint : fullRobotModel.getOneDoFJoints())
      {
         controlledJoints.add(joint, lowLevelOutput.getJointDesiredOutput(joint));
      }

      for (String jointToIgnore : toolbox.getWalkingControllerParameters().getJointsToIgnoreInController())
      {
         for (int i = controlledJoints.size() - 1; i >= 0; i--)
         {
            if (controlledJoints.first(i).getName().equals(jointToIgnore))
               controlledJoints.remove(i);
         }
      }

      Map<String, JointDesiredBehaviorReadOnly> jointBehaviorMap = new HashMap<>();
      ParameterTools.extractJointBehaviorMap("NoLoad", toolbox.getDiagnosticParameters().getDesiredJointBehaviors(), jointBehaviorMap, registry);
      jointDesiredBehaviors = new JointDesiredBehaviorReadOnly[controlledJoints.size()];

      for (int i = 0; i < controlledJoints.size(); i++)
      {
         String jointName = controlledJoints.get(i).getKey().getName();
         JointDesiredBehaviorReadOnly jointDesiredBehavior = jointBehaviorMap.get(jointName);

         Objects.requireNonNull(jointBehaviorMap, "No desired behavior for the joint: " + jointName);
         JointDesiredOutputBasics jointDesiredOutput = controlledJoints.get(i).getRight();

         if (!jointDesiredOutput.hasStiffness())
            throw new IllegalArgumentException("Behavior for the joint: " + jointName + " is missing stiffness.");
         if (!jointDesiredOutput.hasDamping())
            throw new IllegalArgumentException("Behavior for the joint: " + jointName + " is missing damping.");

         jointDesiredBehaviors[i] = jointDesiredBehavior;
      }

      trajectoryTimeProvider = () -> diagnosticParameters.getInitialJointSplineDuration();
      submitDiagnostic(new WaitDiagnosticTask(trajectoryTimeProvider.getValue()));

      for (int i = 0; i < controlledJoints.size(); i++)
      {
         OneDoFJointBasics joint = controlledJoints.first(i);
         String jointName = joint.getName();
         OneDoFJointQuinticTrajectoryGenerator jointTrajectory = new OneDoFJointQuinticTrajectoryGenerator(jointName, joint, trajectoryTimeProvider, registry);
         jointTrajectories.put(joint, jointTrajectory);
         jointTrajectories.get(joint).setFinalPosition(diagnosticParameters.getDiagnosticSetpoints().getSetpoint(jointName));
      }

      for (int i = 0; i < controlledJoints.size(); i++)
      {
         OneDoFJointBasics joint = controlledJoints.first(i);
         String jointName = joint.getName();
         JointDesiredBehaviorReadOnly jointDesiredBehavior = jointDesiredBehaviors[i];

         DoubleProvider kp = () -> jointDesiredBehavior.getStiffness();
         DoubleProvider kd = () -> jointDesiredBehavior.getDamping();
         ParameterizedPDController jointPDController = new ParameterizedPDController(kp, kd, jointName, registry);
         YoDouble jointDesiredPosition = new YoDouble("q_d_" + jointName, registry);
         YoDouble jointDesiredVelocity = new YoDouble("qd_d_" + jointName, registry);
         YoDouble jointDesiredTau = new YoDouble("tau_d_" + jointName, registry);

         jointPDControllerMap.put(joint, jointPDController);
         jointDesiredPositionMap.put(joint, jointDesiredPosition);
         jointDesiredVelocityMap.put(joint, jointDesiredVelocity);
         jointDesiredTauMap.put(joint, jointDesiredTau);
      }
   
      if (diagnosticParameters.enableLogging())
      {
         diagnosticTaskExecutor.setupForLogging();
         setupForLogging();
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
      for (int i = 0; i < controlledJoints.size(); i++)
      {
         OneDoFJointBasics state = controlledJoints.first(i);
         jointTrajectories.get(state).initialize(state.getQ(), 0.0);
      }

      startTime.set(yoTime.getDoubleValue());

      hasBeenInitialized = true;
   }

   private boolean hasBeenInitialized = false;

   @Override
   public void doControl()
   {
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

      for (int i = 0; i < controlledJoints.size(); i++)
      {
         OneDoFJointBasics state = controlledJoints.first(i);
         JointDesiredOutputBasics output = controlledJoints.second(i);
         ParameterizedPDController jointPDController = jointPDControllerMap.get(state);

         double desiredJointPositionOffset = 0.0;
         double desiredJointVelocityOffset = 0.0;
         double desiredJointTauOffset = 0.0;

         if (currentTask != null)
         {
            desiredJointPositionOffset = currentTask.getDesiredJointPositionOffset(state);
            desiredJointVelocityOffset = currentTask.getDesiredJointVelocityOffset(state);
            desiredJointTauOffset = currentTask.getDesiredJointTauOffset(state);
         }

         OneDoFJointQuinticTrajectoryGenerator jointTrajectory = jointTrajectories.get(state);
         jointTrajectory.compute(yoTime.getDoubleValue() - startTime.getDoubleValue());

         YoDouble jointDesiredPosition = jointDesiredPositionMap.get(state);
         YoDouble jointDesiredVelocity = jointDesiredVelocityMap.get(state);

         jointDesiredPosition.set(jointTrajectory.getValue() + desiredJointPositionOffset);
         jointDesiredVelocity.set(jointTrajectory.getVelocity() + desiredJointVelocityOffset);

         double q = state.getQ();
         double qDesired = jointDesiredPosition.getDoubleValue();
         double qd = state.getQd();
         double qdDesired = jointDesiredVelocity.getDoubleValue();
         double tauDesired = jointPDController.compute(q, qDesired, qd, qdDesired) + desiredJointTauOffset;
         output.setDesiredTorque(tauDesired);
         output.setDesiredPosition(qDesired);
         output.setDesiredVelocity(qdDesired);

         jointDesiredTauMap.get(state).set(tauDesired);
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
   }

   private void doIdleControl()
   {
      for (int i = 0; i < controlledJoints.size(); i++)
      {
         OneDoFJointBasics state = controlledJoints.first(i);
         JointDesiredOutputBasics output = controlledJoints.second(i);
         ParameterizedPDController jointPDController = jointPDControllerMap.get(state);

         OneDoFJointQuinticTrajectoryGenerator jointTrajectory = jointTrajectories.get(state);
         jointTrajectory.compute(trajectoryTimeProvider.getValue());

         YoDouble jointDesiredPosition = jointDesiredPositionMap.get(state);
         YoDouble jointDesiredVelocity = jointDesiredVelocityMap.get(state);
         jointDesiredPosition.set(jointTrajectory.getValue());
         jointDesiredVelocity.set(0.0);

         double q = state.getQ();
         double qDesired = jointDesiredPosition.getDoubleValue();
         double qd = state.getQd();
         double qdDesired = jointDesiredVelocity.getDoubleValue();
         double tauDesired = jointPDController.compute(q, qDesired, qd, qdDesired);

         double qdMax = qdMaxIdle.getDoubleValue();
         double qddMax = qddMaxIdle.getDoubleValue();
         double tauMax = tauMaxIdle.getDoubleValue();

         qDesired = q + MathTools.clamp(qDesired - q, qdMax * controlDT);
         qdDesired = qd + MathTools.clamp(qdDesired - qd, qddMax * controlDT);
         tauDesired = MathTools.clamp(tauDesired, tauMax);
         output.setDesiredTorque(tauDesired);
         output.setDesiredPosition(qDesired);
         output.setDesiredVelocity(qdDesired);

         jointDesiredTauMap.get(state).set(tauDesired);
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
}
