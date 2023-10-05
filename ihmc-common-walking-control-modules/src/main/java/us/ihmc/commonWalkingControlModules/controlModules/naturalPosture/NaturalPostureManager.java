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

   private final NaturalPostureController controller;
   private final NaturalPosturePrivilegedConfigurationController privilegedConfigurationController;
   private final JointLimitEnforcementMethodCommand jointLimitEnforcementMethodCommand = new JointLimitEnforcementMethodCommand();
   private final ExecutionTimer timer;

   private final YoBoolean useNaturalPostureCommand = new YoBoolean("useNaturalPostureCommand", registry);
   private final YoBoolean usePelvisPrivilegedPoseCommand = new YoBoolean("usePelvisPrivilegedPoseCommandForNaturalPosture", registry);
   private final YoBoolean useBodyManagerCommands = new YoBoolean("useBodyManagerCommandsForNaturalPosture", registry);
   private final YoBoolean usePelvisOrientationCommand = new YoBoolean("usePelvisOrientationCommandForNaturalPosture", registry);

   public NaturalPostureManager(NaturalPostureParameters parameters, HighLevelHumanoidControllerToolbox controllerToolbox, YoRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);

      useNaturalPostureCommand.set(true); //this must be true to use Natural Posture
      usePelvisPrivilegedPoseCommand.set(false);
      useBodyManagerCommands.set(false);
      usePelvisOrientationCommand.set(true);

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

   public YoBoolean getUseNaturalPostureCommand()
   {
      return useNaturalPostureCommand;
   }

   public YoBoolean getUseBodyManagerCommands()
   {
      return useBodyManagerCommands;
   }

   public YoBoolean getUsePelvisPrivilegedPoseCommand()
   {
      return usePelvisPrivilegedPoseCommand;
   }

   public YoBoolean getUsePelvisOrientationCommand()
   {
      return usePelvisOrientationCommand;
   }
}