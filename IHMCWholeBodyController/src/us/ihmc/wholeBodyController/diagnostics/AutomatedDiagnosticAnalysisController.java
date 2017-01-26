package us.ihmc.wholeBodyController.diagnostics;

import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.logging.Logger;

import org.yaml.snakeyaml.Yaml;

import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.controllers.PDController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.trajectories.OneDoFJointQuinticTrajectoryGenerator;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.trajectories.providers.ConstantDoubleProvider;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.sensorProcessing.diagnostic.DiagnosticParameters;
import us.ihmc.wholeBodyController.diagnostics.utils.DiagnosticTask;
import us.ihmc.wholeBodyController.diagnostics.utils.DiagnosticTaskExecutor;
import us.ihmc.wholeBodyController.diagnostics.utils.WaitDiagnosticTask;

public class AutomatedDiagnosticAnalysisController implements RobotController
{
   private Logger logger;

   private final YoVariableRegistry registry = new YoVariableRegistry(getName());

   private final DoubleYoVariable yoTime;

   private final DiagnosticTaskExecutor diagnosticTaskExecutor;

   private final ArrayList<OneDoFJoint> controlledJoints = new ArrayList<>();
   private final Map<OneDoFJoint, PDController> jointPDControllerMap = new LinkedHashMap<>();
   private final Map<OneDoFJoint, DoubleYoVariable> jointDesiredPositionMap = new LinkedHashMap<>();
   private final Map<OneDoFJoint, DoubleYoVariable> jointDesiredVelocityMap = new LinkedHashMap<>();
   private final Map<OneDoFJoint, DoubleYoVariable> jointDesiredTauMap = new LinkedHashMap<>();

   private final DoubleProvider trajectoryTimeProvider;
   private final Map<OneDoFJoint, OneDoFJointQuinticTrajectoryGenerator> jointTrajectories = new LinkedHashMap<>();

   private final ArrayDeque<DiagnosticDataReporter> dataReportersToExecute = new ArrayDeque<>();
   private DiagnosticDataReporter diagnosticDataReporterRunning = null;

   private final BooleanYoVariable doIdleControl = new BooleanYoVariable("doIdleControl", registry);
   private final DoubleYoVariable qdMaxIdle = new DoubleYoVariable("qdMaxIdle", registry);
   private final DoubleYoVariable qddMaxIdle = new DoubleYoVariable("qddMaxIdle", registry);
   private final DoubleYoVariable tauMaxIdle = new DoubleYoVariable("tauMaxIdle", registry);

   private final BooleanYoVariable isDiagnosticComplete = new BooleanYoVariable("isDiagnosticComplete", registry);
   private final BooleanYoVariable robotIsAlive = new BooleanYoVariable("robotIsAlive", registry);
   private final DoubleYoVariable startTime = new DoubleYoVariable("diagnosticControllerStartTime", registry);

   private final double controlDT;

   public AutomatedDiagnosticAnalysisController(DiagnosticControllerToolbox toolbox, InputStream gainStream, InputStream setpointStream,
         YoVariableRegistry parentRegistry)
   {
      this.yoTime = toolbox.getYoTime();
      this.controlDT = toolbox.getDT();

      diagnosticTaskExecutor = new DiagnosticTaskExecutor("highLevelTaskExecutor", yoTime, registry);

      FullHumanoidRobotModel fullRobotModel = toolbox.getFullRobotModel();
      DiagnosticParameters diagnosticParameters = toolbox.getDiagnosticParameters();

      doIdleControl.set(diagnosticParameters.doIdleControlUntilRobotIsAlive());
      qdMaxIdle.set(diagnosticParameters.getIdleQdMax());
      qddMaxIdle.set(diagnosticParameters.getIdleQddMax());
      tauMaxIdle.set(diagnosticParameters.getIdleTauMax());

      fullRobotModel.getOneDoFJoints(controlledJoints);

      for (String jointToIgnore : toolbox.getWalkingControllerParameters().getJointsToIgnoreInController())
      {
         for (int i = controlledJoints.size() - 1; i >= 0; i--)
         {
            if (controlledJoints.get(i).getName().equals(jointToIgnore))
               controlledJoints.remove(i);
         }
      }

      trajectoryTimeProvider = new ConstantDoubleProvider(diagnosticParameters.getInitialJointSplineDuration());
      submitDiagnostic(new WaitDiagnosticTask(trajectoryTimeProvider.getValue()));

      for (int i = 0; i < controlledJoints.size(); i++)
      {
         OneDoFJoint joint = controlledJoints.get(i);
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
         OneDoFJoint joint = controlledJoints.get(i);
         String jointName = joint.getName();
         
         PDController jointPDController = new PDController(jointName, registry);
         DoubleYoVariable jointDesiredPosition = new DoubleYoVariable("q_d_" + jointName, registry);
         DoubleYoVariable jointDesiredVelocity = new DoubleYoVariable("qd_d_" + jointName, registry);
         DoubleYoVariable jointDesiredTau = new DoubleYoVariable("tau_d_" + jointName, registry);
         
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
         OneDoFJoint joint = controlledJoints.get(i);
         jointTrajectories.get(joint).initialize(joint.getQ(), 0.0);
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
         OneDoFJoint joint = controlledJoints.get(i);
         PDController jointPDController = jointPDControllerMap.get(joint);

         double desiredJointPositionOffset = 0.0;
         double desiredJointVelocityOffset = 0.0;
         double desiredJointTauOffset = 0.0;

         if (currentTask != null)
         {
            desiredJointPositionOffset = currentTask.getDesiredJointPositionOffset(joint);  
            desiredJointVelocityOffset = currentTask.getDesiredJointVelocityOffset(joint);  
            desiredJointTauOffset = currentTask.getDesiredJointTauOffset(joint);            
         }

         OneDoFJointQuinticTrajectoryGenerator jointTrajectory = jointTrajectories.get(joint);
         jointTrajectory.compute(yoTime.getDoubleValue() - startTime.getDoubleValue());

         DoubleYoVariable jointDesiredPosition = jointDesiredPositionMap.get(joint);
         DoubleYoVariable jointDesiredVelocity = jointDesiredVelocityMap.get(joint);

         jointDesiredPosition.set(jointTrajectory.getValue() + desiredJointPositionOffset);
         jointDesiredVelocity.set(jointTrajectory.getVelocity() + desiredJointVelocityOffset);

         double q = joint.getQ();
         double qDesired = jointDesiredPosition.getDoubleValue();
         double qd = joint.getQd();
         double qdDesired = jointDesiredVelocity.getDoubleValue();
         double tauDesired = jointPDController.compute(q, qDesired, qd, qdDesired) + desiredJointTauOffset;
         joint.setTau(tauDesired);
         joint.setqDesired(qDesired);
         joint.setQdDesired(qdDesired);

         jointDesiredTauMap.get(joint).set(tauDesired);
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
         OneDoFJoint joint = controlledJoints.get(i);
         PDController jointPDController = jointPDControllerMap.get(joint);

         OneDoFJointQuinticTrajectoryGenerator jointTrajectory = jointTrajectories.get(joint);
         jointTrajectory.compute(trajectoryTimeProvider.getValue());

         DoubleYoVariable jointDesiredPosition = jointDesiredPositionMap.get(joint);
         DoubleYoVariable jointDesiredVelocity = jointDesiredVelocityMap.get(joint);
         jointDesiredPosition.set(jointTrajectory.getValue());
         jointDesiredVelocity.set(0.0);

         double q = joint.getQ();
         double qDesired = jointDesiredPosition.getDoubleValue();
         double qd = joint.getQd();
         double qdDesired = jointDesiredVelocity.getDoubleValue();
         double tauDesired = jointPDController.compute(q, qDesired, qd, qdDesired);

         double qdMax = qdMaxIdle.getDoubleValue();
         double qddMax = qddMaxIdle.getDoubleValue();
         double tauMax = tauMaxIdle.getDoubleValue();

         qDesired = q + MathTools.clipToMinMax(qDesired - q, qdMax * controlDT);
         qdDesired = qd + MathTools.clipToMinMax(qdDesired - qd, qddMax * controlDT);
         tauDesired = MathTools.clipToMinMax(tauDesired, tauMax);
         joint.setTau(tauDesired);
         joint.setqDesired(qDesired);
         joint.setQdDesired(qdDesired);

         jointDesiredTauMap.get(joint).set(tauDesired);
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
