package us.ihmc.wholeBodyController.diagnostics;

import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayDeque;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.logging.Logger;

import org.yaml.snakeyaml.Yaml;

import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.commons.MathTools;
import us.ihmc.robotics.controllers.PDController;
import us.ihmc.robotics.math.trajectories.OneDoFJointQuinticTrajectoryGenerator;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.trajectories.providers.ConstantDoubleProvider;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.sensorProcessing.diagnostic.DiagnosticParameters;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.tools.lists.PairList;
import us.ihmc.wholeBodyController.diagnostics.utils.DiagnosticTask;
import us.ihmc.wholeBodyController.diagnostics.utils.DiagnosticTaskExecutor;
import us.ihmc.wholeBodyController.diagnostics.utils.WaitDiagnosticTask;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class AutomatedDiagnosticAnalysisController implements RobotController
{
   private Logger logger;

   private final YoVariableRegistry registry = new YoVariableRegistry(getName());

   private final YoDouble yoTime;

   private final DiagnosticTaskExecutor diagnosticTaskExecutor;

   private final PairList<OneDoFJoint, JointDesiredOutput> controlledJoints = new PairList<>();
   private final Map<OneDoFJoint, PDController> jointPDControllerMap = new LinkedHashMap<>();
   private final Map<OneDoFJoint, YoDouble> jointDesiredPositionMap = new LinkedHashMap<>();
   private final Map<OneDoFJoint, YoDouble> jointDesiredVelocityMap = new LinkedHashMap<>();
   private final Map<OneDoFJoint, YoDouble> jointDesiredTauMap = new LinkedHashMap<>();

   private final DoubleProvider trajectoryTimeProvider;
   private final Map<OneDoFJoint, OneDoFJointQuinticTrajectoryGenerator> jointTrajectories = new LinkedHashMap<>();

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

   public AutomatedDiagnosticAnalysisController(DiagnosticControllerToolbox toolbox, InputStream gainStream, InputStream setpointStream,
         YoVariableRegistry parentRegistry)
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

      for(OneDoFJoint joint : fullRobotModel.getOneDoFJoints())
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

      trajectoryTimeProvider = new ConstantDoubleProvider(diagnosticParameters.getInitialJointSplineDuration());
      submitDiagnostic(new WaitDiagnosticTask(trajectoryTimeProvider.getValue()));

      for (int i = 0; i < controlledJoints.size(); i++)
      {
         OneDoFJoint joint = controlledJoints.first(i);
         String name = joint.getName();
         OneDoFJointQuinticTrajectoryGenerator jointTrajectory = new OneDoFJointQuinticTrajectoryGenerator(name, joint, trajectoryTimeProvider, registry);
         jointTrajectories.put(joint, jointTrajectory);
      }

      setupJointControllers(gainStream, setpointStream);

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

   @SuppressWarnings("unchecked")
   private void setupJointControllers(InputStream gainStream, InputStream setpointStream)
   {
      Yaml yaml = new Yaml();

      Map<String, Map<String, Double>> gainMap = null;

      if (gainStream != null)
      {
         gainMap = (Map<String, Map<String, Double>>) yaml.load(gainStream);
         try
         {
            gainStream.close();
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }
         
      Map<String, Double> setpointMap = null;

      if (setpointStream != null)
      {
         setpointMap = (Map<String, Double>) yaml.load(setpointStream);
         try
         {
            setpointStream.close();
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }

      for (int i = 0; i < controlledJoints.size(); i++)
      {
         OneDoFJoint joint = controlledJoints.first(i);
         String jointName = joint.getName();
         
         PDController jointPDController = new PDController(jointName, registry);
         YoDouble jointDesiredPosition = new YoDouble("q_d_" + jointName, registry);
         YoDouble jointDesiredVelocity = new YoDouble("qd_d_" + jointName, registry);
         YoDouble jointDesiredTau = new YoDouble("tau_d_" + jointName, registry);
         
         if (gainMap != null)
         {
            Map<String, Double> jointGains = gainMap.get(jointName);
            if (jointGains != null)
            {
               Double kp = jointGains.get("kp");
               if (kp != null)
                  jointPDController.setProportionalGain(kp);
               Double kd = jointGains.get("kd");
               if (kd != null)
                  jointPDController.setDerivativeGain(kd);
            }
            else
            {
               jointPDController.setProportionalGain(50.0);
               jointPDController.setDerivativeGain(5.0);
            }
         }
         else
         {
            jointPDController.setProportionalGain(50.0);
            jointPDController.setDerivativeGain(5.0);
         }

         if (setpointMap != null)
         {
            Double jointSetpoint = setpointMap.get(jointName);
            if (jointSetpoint != null)
               jointTrajectories.get(joint).setFinalPosition(jointSetpoint);
         }

         jointPDControllerMap.put(joint, jointPDController);
         jointDesiredPositionMap.put(joint, jointDesiredPosition);
         jointDesiredVelocityMap.put(joint, jointDesiredVelocity);
         jointDesiredTauMap.put(joint, jointDesiredTau);
      }
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
         OneDoFJoint state = controlledJoints.first(i);
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
         OneDoFJoint state = controlledJoints.first(i);
         JointDesiredOutput output = controlledJoints.second(i);
         PDController jointPDController = jointPDControllerMap.get(state);

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
         OneDoFJoint state = controlledJoints.first(i);
         JointDesiredOutput output = controlledJoints.second(i);
         PDController jointPDController = jointPDControllerMap.get(state);

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
         Thread thread = new Thread(diagnosticDataReporterRunning);
         thread.setPriority(Thread.MIN_PRIORITY);
         thread.start();
      }
   }

   public boolean isDiagnosticDone()
   {
      return isDiagnosticComplete.getBooleanValue();
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
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
