package us.ihmc.commonWalkingControlModules.controlModules.naturalPosture;

import us.ihmc.commonWalkingControlModules.configurations.NaturalPostureParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointLimitEnforcementMethodCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitEnforcement;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitParameters;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class NaturalPostureManager
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private NaturalPostureController controller;
   private NaturalPosturePrivilegedConfigurationController privilegedConfigurationController;
   private final JointLimitEnforcementMethodCommand jointLimitEnforcementMethodCommand = new JointLimitEnforcementMethodCommand();
   private ExecutionTimer timer;

   private final YoBoolean enableNaturalPosture = new YoBoolean("enableNaturalPosture", registry);
   private final YoBoolean usePelvisPrivilegedPoseCommand = new YoBoolean("usePelvisPrivilegedPoseCommandForNaturalPosture", registry);
   private final YoBoolean useBodyManagerCommands = new YoBoolean("useBodyManagerCommandsForNaturalPosture", registry);
   private final YoBoolean usePelvisOrientationCommand = new YoBoolean("usePelvisOrientationCommandForNaturalPosture", registry);

   public NaturalPostureManager(NaturalPostureParameters parameters, HighLevelHumanoidControllerToolbox controllerToolbox, YoRegistry parentRegistry)
   {
      // This must be true to use Natural Posture
      enableNaturalPosture.set(false);

      usePelvisPrivilegedPoseCommand.set(false);
      useBodyManagerCommands.set(false);
      usePelvisOrientationCommand.set(true);

      parentRegistry.addChild(registry);

      controller = new NaturalPostureController(parameters, controllerToolbox, registry);
      privilegedConfigurationController = new NaturalPosturePrivilegedConfigurationController(parameters, controllerToolbox.getFullRobotModel(), registry);

      timer = new ExecutionTimer("naturalPostureTimer", registry);

      OneDoFJointBasics[] allOneDoFJoints = MultiBodySystemTools.filterJoints(controllerToolbox.getControlledJoints(), OneDoFJointBasics.class);
      String[] jointNamesRestrictiveLimits = parameters.getJointsWithRestrictiveLimits();
      OneDoFJointBasics[] jointsWithRestrictiveLimits = MultiBodySystemTools.filterJoints(ScrewTools.findJointsWithNames(allOneDoFJoints,
                                                                                                                         jointNamesRestrictiveLimits),
                                                                                          OneDoFJointBasics.class);

      for (OneDoFJointBasics joint : jointsWithRestrictiveLimits)
      {
         JointLimitParameters limitParameters = parameters.getJointLimitParametersForJointsWithRestrictiveLimits(joint.getName());
         if (limitParameters == null)
            throw new RuntimeException("Must define joint limit parameters for joint " + joint.getName() + " if using joints with restrictive limits.");
         jointLimitEnforcementMethodCommand.addLimitEnforcementMethod(joint, JointLimitEnforcement.RESTRICTIVE, limitParameters);
      }
   }

   public boolean isEnabled()
   {
      return enableNaturalPosture.getBooleanValue();
   }

   public InverseDynamicsCommand<?> getQPObjectiveCommand()
   {
      return controller.getInverseDynamicsCommand();
   }

   public InverseDynamicsCommand<?> getPelvisPrivilegedPoseCommand()
   {
      return privilegedConfigurationController.getPelvisPrivilegedPoseCommand();
   }

   public JointLimitEnforcementMethodCommand getJointLimitEnforcementCommand()
   {
      return jointLimitEnforcementMethodCommand;
   }

   public void compute()
   {
      timer.startMeasurement();
      privilegedConfigurationController.compute();
      controller.compute();
      timer.stopMeasurement();
   }

   public NaturalPostureController getController()
   {
      return controller;
   }

   public NaturalPosturePrivilegedConfigurationController getPrivilegedConfigurationController()
   {
      return privilegedConfigurationController;
   }

   /**
    * This should always evaluate to TRUE if natural posture is disabled.
    */
   public boolean getUseBodyManagerCommands()
   {
      if (enableNaturalPosture.getBooleanValue())
         return useBodyManagerCommands.getBooleanValue();
      else
         return true;
   }

   /**
    * This should always evaluate to FALSE if natural posture is disabled.
    */
   public boolean getUsePelvisPrivilegedPoseCommand()
   {
      if (enableNaturalPosture.getBooleanValue())
         return usePelvisPrivilegedPoseCommand.getBooleanValue();
      else
         return false;
   }

   /**
    * This should always evaluate to TRUE if natural posture is disabled.
    */
   public boolean getUsePelvisOrientationCommand()
   {
      if (enableNaturalPosture.getBooleanValue())
         return usePelvisOrientationCommand.getBooleanValue();
      else
         return true;
   }
}