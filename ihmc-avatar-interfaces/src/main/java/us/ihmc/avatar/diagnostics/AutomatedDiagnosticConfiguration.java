package us.ihmc.avatar.diagnostics;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.wholeBodyController.diagnostics.AutomatedDiagnosticAnalysisController;
import us.ihmc.wholeBodyController.diagnostics.DiagnosticControllerToolbox;
import us.ihmc.wholeBodyController.diagnostics.DiagnosticParameters;
import us.ihmc.wholeBodyController.diagnostics.OneDoFJointCheckUpDiagnosticTask;
import us.ihmc.wholeBodyController.diagnostics.PelvisIMUCheckUpDiagnosticTask;
import us.ihmc.wholeBodyController.diagnostics.utils.DiagnosticParallelTask;
import us.ihmc.wholeBodyController.diagnostics.utils.WaitDiagnosticTask;

public class AutomatedDiagnosticConfiguration
{
   private final DiagnosticControllerToolbox toolbox;
   private final AutomatedDiagnosticAnalysisController diagnosticController;
   private final boolean enableLogging;

   public AutomatedDiagnosticConfiguration(DiagnosticControllerToolbox toolbox, AutomatedDiagnosticAnalysisController diagnosticController)
   {
      this.toolbox = toolbox;
      this.diagnosticController = diagnosticController;
      enableLogging = toolbox.getDiagnosticParameters().enableLogging();
   }

   public void addWait(double timeToWait)
   {
      diagnosticController.submitDiagnostic(new WaitDiagnosticTask(timeToWait));
   }

   /**
    * Queues a check-up for running diagnostic on a single joint.
    * 
    * @param jointName the name of the joint to run diagnostics on.
    */
   public void addSingleJointCheckUp(String jointName)
   {
      OneDoFJointBasics joint = toolbox.getFullRobotModel().getOneDoFJointByName(jointName);
      OneDoFJointCheckUpDiagnosticTask checkUp = new OneDoFJointCheckUpDiagnosticTask(joint, toolbox);

      if (enableLogging)
         checkUp.setupForLogging();

      diagnosticController.submitDiagnostic(checkUp);
   }

   /**
    * Queues a check-up for running diagnostic on several joints in parallel.
    * 
    * @param jointNames the name of the joints to run diagnostics on.
    */
   public void addParallelJointCheckUp(String... jointNames)
   {
      addParallelJointCheckUp(Arrays.asList(jointNames));
   }

   /**
    * Queues a check-up for running diagnostic on several joints in parallel.
    * 
    * @param jointNames the name of the joints to run diagnostics on.
    */
   public void addParallelJointCheckUp(Collection<String> jointNames)
   {
      List<OneDoFJointCheckUpDiagnosticTask> checkUps = new ArrayList<>();

      for (String jointName : jointNames)
      {
         OneDoFJointBasics joint = toolbox.getFullRobotModel().getOneDoFJointByName(jointName);

         OneDoFJointCheckUpDiagnosticTask checkUp = new OneDoFJointCheckUpDiagnosticTask(joint, toolbox);
         if (enableLogging)
            checkUp.setupForLogging();
         checkUps.add(checkUp);
      }

      if (!checkUps.isEmpty())
      {
         if (checkUps.size() > 1)
            diagnosticController.submitDiagnostic(new DiagnosticParallelTask(checkUps));
         else
            diagnosticController.submitDiagnostic(checkUps.get(0));
      }
   }

   /**
    * Queues a list of check-ups, each check-up can be for either a single or multiple joint(s).
    * 
    * @param jointNames a 2D list: the first dimension represents the series of check-ups to run, the
    *                   second dimension represents the name of the joints to run diagnostic for at
    *                   each check-up.
    */
   public void addJointCheckUps(List<? extends List<String>> jointNames)
   {
      for (int i = 0; i < jointNames.size(); i++)
      {
         addParallelJointCheckUp(jointNames.get(i));
      }
   }

   public void addPelvisIMUCheckUpDiagnostic()
   {
      DiagnosticParameters diagnosticParameters = toolbox.getDiagnosticParameters();
      String pelvisIMUName = diagnosticParameters.getPelvisIMUName();
      if (pelvisIMUName == null || pelvisIMUName.isEmpty())
      {
         System.err.println(getClass().getSimpleName() + ": Cannot create the pelvis IMU check up diagnostic without a pelvisIMUName to look for.");
         return;
      }

      PelvisIMUCheckUpDiagnosticTask checkUp = new PelvisIMUCheckUpDiagnosticTask(pelvisIMUName, toolbox);
      if (enableLogging)
         checkUp.setupForLogging();
      diagnosticController.submitDiagnostic(checkUp);
   }
}
