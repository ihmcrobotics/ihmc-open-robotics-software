package us.ihmc.wholeBodyController.diagnostics;

import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.logging.Logger;

import org.yaml.snakeyaml.Yaml;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.PDController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.wholeBodyController.diagnostics.utils.DiagnosticTask;
import us.ihmc.wholeBodyController.diagnostics.utils.DiagnosticTaskExecutor;

public class AutomatedDiagnosticAnalysisController implements RobotController
{
   private Logger logger;

   private final YoVariableRegistry registry = new YoVariableRegistry(getName());

   private final DoubleYoVariable yoTime;

   private final DiagnosticTaskExecutor diagnosticTaskExecutor;

   private final ArrayList<OneDoFJoint> controlledJoints = new ArrayList<>();
   private final Map<OneDoFJoint, PDController> jointPDControllerMap = new LinkedHashMap<>();
   private final Map<OneDoFJoint, DoubleYoVariable> jointDesiredPositionMap = new LinkedHashMap<>();
   private final Map<OneDoFJoint, DoubleYoVariable> jointDesiredTauMap = new LinkedHashMap<>();

   private final BooleanYoVariable isDiagnosticComplete = new BooleanYoVariable("isDiagnosticComplete", registry);

   public AutomatedDiagnosticAnalysisController(DiagnosticControllerToolbox toolbox, InputStream gainStream, InputStream setpointStream,
         YoVariableRegistry parentRegistry)
   {
      this.yoTime = toolbox.getYoTime();

      diagnosticTaskExecutor = new DiagnosticTaskExecutor("highLevelTaskExecutor", yoTime, registry);

      FullHumanoidRobotModel fullRobotModel = toolbox.getFullRobotModel();
      fullRobotModel.getOneDoFJoints(controlledJoints);

      for (String jointToIgnore : toolbox.getWalkingControllerParameters().getJointsToIgnoreInController())
      {
         for (int i = controlledJoints.size() - 1; i >= 0; i--)
         {
            if (controlledJoints.get(i).getName().equals(jointToIgnore))
               controlledJoints.remove(i);
         }
      }

      setupJointControllers(gainStream, setpointStream);

      if (toolbox.getDiagnosticParameters().enableLogging())
      {
         diagnosticTaskExecutor.setupForLogging();
         setupForLogging();
      }

      parentRegistry.addChild(registry);
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
               jointDesiredPosition.set(jointSetpoint);
         }

         jointPDControllerMap.put(joint, jointPDController);
         jointDesiredPositionMap.put(joint, jointDesiredPosition);
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

   }

   @Override
   public void doControl()
   {
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

         double q = joint.getQ();
         double qDesired = jointDesiredPositionMap.get(joint).getDoubleValue() + desiredJointPositionOffset;
         double qd = joint.getQd();
         double qdDesired = 0.0 + desiredJointVelocityOffset;
         double tauDesired = jointPDController.compute(q, qDesired, qd, qdDesired) + desiredJointTauOffset;
         joint.setTau(tauDesired);

         jointDesiredTauMap.get(joint).set(tauDesired);
      }

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
