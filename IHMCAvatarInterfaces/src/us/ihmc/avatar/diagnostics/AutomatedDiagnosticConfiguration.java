package us.ihmc.avatar.diagnostics;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.RobotSpecificJointNames;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.diagnostic.DiagnosticParameters;
import us.ihmc.wholeBodyController.diagnostics.AutomatedDiagnosticAnalysisController;
import us.ihmc.wholeBodyController.diagnostics.DiagnosticControllerToolbox;
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

   public void addJointCheckUpDiagnostic()
   {
      FullHumanoidRobotModel fullRobotModel = toolbox.getFullRobotModel();
      RobotSpecificJointNames robotSpecificJointNames = fullRobotModel.getRobotSpecificJointNames();
      List<String> jointsToIgnore = toolbox.getDiagnosticParameters().getJointsToIgnoreDuringDiagnostic();

      for (LegJointName legJointName : robotSpecificJointNames.getLegJointNames())
      {
         List<OneDoFJointCheckUpDiagnosticTask> checkUps = new ArrayList<>();

         for (RobotSide robotSide : RobotSide.values)
         {
            OneDoFJoint legJoint = fullRobotModel.getLegJoint(robotSide, legJointName);
            if (!jointsToIgnore.contains(legJoint.getName()))
            {
               OneDoFJointCheckUpDiagnosticTask checkUp = new OneDoFJointCheckUpDiagnosticTask(legJoint, toolbox);
               if (enableLogging)
                  checkUp.setupForLogging();
               checkUps.add(checkUp);
            }
         }

         if (!checkUps.isEmpty())
         {
            if (checkUps.size() > 1)
               diagnosticController.submitDiagnostic(new DiagnosticParallelTask(checkUps));
            else
               diagnosticController.submitDiagnostic(checkUps.get(0));
         }
      }

      for (ArmJointName armJointName : robotSpecificJointNames.getArmJointNames())
      {
         List<OneDoFJointCheckUpDiagnosticTask> checkUps = new ArrayList<>();

         for (RobotSide robotSide : RobotSide.values)
         {
            OneDoFJoint armJoint = fullRobotModel.getArmJoint(robotSide, armJointName);
            if (!jointsToIgnore.contains(armJoint.getName()))
            {
               OneDoFJointCheckUpDiagnosticTask checkUp = new OneDoFJointCheckUpDiagnosticTask(armJoint, toolbox);
               if (enableLogging)
                  checkUp.setupForLogging();
               checkUps.add(checkUp);
            }
         }

         if (!checkUps.isEmpty())
         {
            if (checkUps.size() > 1)
               diagnosticController.submitDiagnostic(new DiagnosticParallelTask(checkUps));
            else
               diagnosticController.submitDiagnostic(checkUps.get(0));
         }
      }

      for (SpineJointName spineJointName : robotSpecificJointNames.getSpineJointNames())
      {
         OneDoFJoint spineJoint = fullRobotModel.getSpineJoint(spineJointName);
         if (!jointsToIgnore.contains(spineJoint.getName()))
         {
            OneDoFJointCheckUpDiagnosticTask checkUp = new OneDoFJointCheckUpDiagnosticTask(spineJoint, toolbox);
            if (enableLogging)
               checkUp.setupForLogging();
            diagnosticController.submitDiagnostic(checkUp);
         }
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
