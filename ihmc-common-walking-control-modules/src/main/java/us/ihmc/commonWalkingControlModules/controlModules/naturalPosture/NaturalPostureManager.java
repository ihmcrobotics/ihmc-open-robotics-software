package us.ihmc.commonWalkingControlModules.controlModules.naturalPosture;

import us.ihmc.commonWalkingControlModules.configurations.HumanoidRobotNaturalPosture;
import us.ihmc.commonWalkingControlModules.configurations.NaturalPostureParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
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

   private final NaturalPostureController naturalPostureController;
   private final NaturalPosturePrivilegedConfigurationController naturalPosturePrivilegedConfigurationController;
   private final JointLimitEnforcementMethodCommand jointLimitEnforcementMethodCommand = new JointLimitEnforcementMethodCommand();
   private final ExecutionTimer naturalPostureTimer;

   private final YoBoolean useNaturalPostureCommand = new YoBoolean("useNaturalPostureCommand", registry);
   private final YoBoolean usePelvisPrivilegedPoseCommand = new YoBoolean("usePelvisPrivilegedPoseCommand", registry);
   private final YoBoolean useBodyManagerCommands = new YoBoolean("useBodyManagerCommands", registry);
   private final YoBoolean usePelvisOrientationCommand = new YoBoolean("usePelvisOrientationCommand", registry);

   public NaturalPostureManager(WalkingControllerParameters walkingControllerParameters,
                                HighLevelHumanoidControllerToolbox controllerToolbox,
                                YoRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);

      useNaturalPostureCommand.set(true); //this must be true to use Natural Posture
      usePelvisPrivilegedPoseCommand.set(false);
      useBodyManagerCommands.set(false);
      usePelvisOrientationCommand.set(true);

      NaturalPostureParameters naturalPostureParameters = walkingControllerParameters.getNaturalPostureParameters();
      HumanoidRobotNaturalPosture robotNaturalPosture = walkingControllerParameters.getNaturalPosture(controllerToolbox.getFullRobotModel());
      naturalPostureController = new NaturalPostureController(robotNaturalPosture, controllerToolbox, registry);
      naturalPosturePrivilegedConfigurationController = new NaturalPosturePrivilegedConfigurationController(controllerToolbox.getFullRobotModel(), registry);

      naturalPostureTimer = new ExecutionTimer("naturalPostureTimer", registry);

      OneDoFJointBasics[] allOneDoFjoints = MultiBodySystemTools.filterJoints(controllerToolbox.getControlledJoints(), OneDoFJointBasics.class);
      String[] jointNamesRestrictiveLimits = naturalPostureParameters.getJointsWithRestrictiveLimits();
      OneDoFJointBasics[] jointsWithRestrictiveLimits = MultiBodySystemTools.filterJoints(ScrewTools.findJointsWithNames(allOneDoFjoints,
                                                                                                                         jointNamesRestrictiveLimits),
                                                                                          OneDoFJointBasics.class);
      for (OneDoFJointBasics joint : jointsWithRestrictiveLimits)
      {
         JointLimitParameters limitParameters = naturalPostureParameters.getJointLimitParametersForJointsWithRestrictiveLimits(joint.getName());
         if (limitParameters == null)
            throw new RuntimeException("Must define joint limit parameters for joint " + joint.getName() + " if using joints with restrictive limits.");
         jointLimitEnforcementMethodCommand.addLimitEnforcementMethod(joint, JointLimitEnforcement.RESTRICTIVE, limitParameters);
      }
   }

   public InverseDynamicsCommand<?> getQPObjectiveCommand()
   {
      return naturalPostureController.getInverseDynamicsCommand();
   }

   public InverseDynamicsCommand<?> getPelvisPrivilegedPoseCommand()
   {
      return naturalPostureController.getPelvisPrivilegedPoseCommand();
   }

   public JointLimitEnforcementMethodCommand getJointLimitEnforcementCommand()
   {
      return jointLimitEnforcementMethodCommand;
   }

   public void initialize()
   {
      naturalPostureController.getHumanoidRobotNaturalPosture().initialize();
      naturalPosturePrivilegedConfigurationController.initialize();
   }

   public void compute()
   {
      naturalPosturePrivilegedConfigurationController.compute(); //this is intentionally outside the time measurement

      naturalPostureTimer.startMeasurement();
      naturalPostureController.compute();
      naturalPostureTimer.stopMeasurement();
   }

   public NaturalPostureController getNaturalPostureController()
   {
      return naturalPostureController;
   }

   public NaturalPosturePrivilegedConfigurationController getNaturalPosturePrivilegedConfigurationController()
   {
      return naturalPosturePrivilegedConfigurationController;
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