package us.ihmc.commonWalkingControlModules.controlModules.naturalPosture;

import us.ihmc.commonWalkingControlModules.configurations.HumanoidRobotNaturalPosture;
import us.ihmc.commonWalkingControlModules.configurations.NaturalPostureParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointLimitEnforcementMethodCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitEnforcement;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitParameters;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.registry.YoRegistry;

public class NaturalPostureManager
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final ControllerNaturalPostureManager walkingManager;

   private final HumanoidRobotNaturalPosture robotNaturalPosture;
   private final JointLimitEnforcementMethodCommand jointLimitEnforcementMethodCommand = new JointLimitEnforcementMethodCommand();

   private final ExecutionTimer naturalPostureTimer;

   public NaturalPostureManager(HumanoidRobotNaturalPosture naturalPostureMeasurement,
                                NaturalPostureParameters naturalPostureParameters,
                                HighLevelHumanoidControllerToolbox controllerToolbox,
                                YoRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);

      robotNaturalPosture = naturalPostureMeasurement;

      walkingManager = new ControllerNaturalPostureManager(robotNaturalPosture, controllerToolbox, registry);
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
      return walkingManager.getInverseDynamicsCommand();
   }

   public InverseDynamicsCommand<?> getPelvisPrivilegedPoseCommand()
   {
      return walkingManager.getPelvisPrivilegedPoseCommand();
   }

   public JointLimitEnforcementMethodCommand getJointLimitEnforcementCommand()
   {
      return jointLimitEnforcementMethodCommand;
   }

   public void compute()
   {
      naturalPostureTimer.startMeasurement();
      walkingManager.compute();
      naturalPostureTimer.stopMeasurement();
   }

   public void initialize()
   {
      robotNaturalPosture.initialize();
   }
}