package us.ihmc.darpaRoboticsChallenge.diagnostics;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.SdfLoader.partNames.ArmJointName;
import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.SdfLoader.partNames.RobotSpecificJointNames;
import us.ihmc.SdfLoader.partNames.SpineJointName;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.wholeBodyController.diagnostics.AutomatedDiagnosticAnalysisController;
import us.ihmc.wholeBodyController.diagnostics.DiagnosticControllerToolbox;
import us.ihmc.wholeBodyController.diagnostics.OneDoFJointCheckUpDiagnosticTask;
import us.ihmc.wholeBodyController.diagnostics.utils.DiagnosticParallelTask;
import us.ihmc.wholeBodyController.diagnostics.utils.WaitDiagnosticTask;

public class AutomatedDiagnosticConfiguration
{
   private final DiagnosticControllerToolbox toolbox;
   private final AutomatedDiagnosticAnalysisController diagnosticController;
   private final YoVariableRegistry controllerRegistry;

   public AutomatedDiagnosticConfiguration(DiagnosticControllerToolbox toolbox, AutomatedDiagnosticAnalysisController diagnosticController)
   {
      this.toolbox = toolbox;
      this.diagnosticController = diagnosticController;
      controllerRegistry = diagnosticController.getYoVariableRegistry();
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
               OneDoFJointCheckUpDiagnosticTask checkUp = new OneDoFJointCheckUpDiagnosticTask(legJoint, toolbox, controllerRegistry);
               checkUp.setupForLogging();
               checkUps.add(checkUp);
            }
         }

         diagnosticController.submitDiagnostic(new DiagnosticParallelTask(checkUps));
      }

      for (ArmJointName armJointName : robotSpecificJointNames.getArmJointNames())
      {
         List<OneDoFJointCheckUpDiagnosticTask> checkUps = new ArrayList<>();

         for (RobotSide robotSide : RobotSide.values)
         {
            OneDoFJoint armJoint = fullRobotModel.getArmJoint(robotSide, armJointName);
            if (!jointsToIgnore.contains(armJoint.getName()))
            {
               OneDoFJointCheckUpDiagnosticTask checkUp = new OneDoFJointCheckUpDiagnosticTask(armJoint, toolbox, controllerRegistry);
               checkUp.setupForLogging();
               checkUps.add(checkUp);
            }
         }

         diagnosticController.submitDiagnostic(new DiagnosticParallelTask(checkUps));
      }

      for (SpineJointName spineJointName : robotSpecificJointNames.getSpineJointNames())
      {
         OneDoFJoint spineJoint = fullRobotModel.getSpineJoint(spineJointName);
         if (!jointsToIgnore.contains(spineJoint.getName()))
         {
            OneDoFJointCheckUpDiagnosticTask checkUp = new OneDoFJointCheckUpDiagnosticTask(spineJoint, toolbox, controllerRegistry);
            checkUp.setupForLogging();
            diagnosticController.submitDiagnostic(checkUp);
         }
      }
   }
}
