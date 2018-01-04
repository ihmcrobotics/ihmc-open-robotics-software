package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ParameterTools;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.parameters.JointAccelerationIntegrationParametersReadOnly;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class AccelerationIntegrationParameterHelper
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final String[] jointNames;
   private final YoBoolean[] jointsLoaded;
   private final JointAccelerationIntegrationParametersReadOnly[] accelerationIntegrationSettingsNoLoad;
   private final JointAccelerationIntegrationParametersReadOnly[] accelerationIntegrationSettingsLoaded;

   private final JointAccelerationIntegrationCommand jointAccelerationIntegrationCommand = new JointAccelerationIntegrationCommand();

   private final WalkingHighLevelHumanoidController walkingController;

   public AccelerationIntegrationParameterHelper(HighLevelControllerParameters parameters, OneDoFJoint[] joints,
                                                 WalkingHighLevelHumanoidController walkingController, YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
      this.walkingController = walkingController;

      Map<String, JointAccelerationIntegrationParametersReadOnly> parameterJointNameMapNoLoad = new HashMap<>();
      ParameterTools.extractAccelerationIntegrationParameterMap("NoLoad", parameters.getJointAccelerationIntegrationParametersNoLoad(),
                                                                parameterJointNameMapNoLoad, registry);

      Map<String, JointAccelerationIntegrationParametersReadOnly> parameterJointNameMapLoaded = new HashMap<>();
      ParameterTools.extractAccelerationIntegrationParameterMap("Loaded", parameters.getJointAccelerationIntegrationParametersLoaded(),
                                                                parameterJointNameMapLoaded, registry);

      jointNames = new String[joints.length];
      jointsLoaded = new YoBoolean[joints.length];
      accelerationIntegrationSettingsNoLoad = new JointAccelerationIntegrationParametersReadOnly[joints.length];
      accelerationIntegrationSettingsLoaded = new JointAccelerationIntegrationParametersReadOnly[joints.length];

      List<String> jointsWithoutParametersNoLoad = new ArrayList<>();
      for (int jointIdx = 0; jointIdx < joints.length; jointIdx++)
      {
         OneDoFJoint joint = joints[jointIdx];
         String jointName = joint.getName();
         jointNames[jointIdx] = jointName;
         jointsLoaded[jointIdx] = new YoBoolean(jointName + "_isUnderLoad", registry);
         jointAccelerationIntegrationCommand.addJointToComputeDesiredPositionFor(joint);

         JointAccelerationIntegrationParametersReadOnly integrationParametersNoLoad = parameterJointNameMapNoLoad.get(jointName);
         if (integrationParametersNoLoad == null)
         {
            jointsWithoutParametersNoLoad.add(jointName);
         }
         accelerationIntegrationSettingsNoLoad[jointIdx] = integrationParametersNoLoad;

         JointAccelerationIntegrationParametersReadOnly integrationParametersLoaded = parameterJointNameMapLoaded.get(jointName);
         accelerationIntegrationSettingsLoaded[jointIdx] = integrationParametersLoaded;
      }

      if (!jointsWithoutParametersNoLoad.isEmpty())
      {
         PrintTools.warn("Got joints without acceleration integration parameters. Will use default values for:\n"
               + jointsWithoutParametersNoLoad);
      }
   }

   public void update()
   {
      for (int jointIdx = 0; jointIdx < jointNames.length; jointIdx++)
      {
         boolean isLoaded = walkingController.isJointLoaded(jointNames[jointIdx]);
         jointsLoaded[jointIdx].set(isLoaded);

         JointAccelerationIntegrationParametersReadOnly integrationParametersNoLoad = accelerationIntegrationSettingsNoLoad[jointIdx];
         JointAccelerationIntegrationParametersReadOnly integrationParametersLoaded = accelerationIntegrationSettingsLoaded[jointIdx];

         if (isLoaded && integrationParametersLoaded != null)
         {
            jointAccelerationIntegrationCommand.setJointParameters(jointIdx, integrationParametersLoaded);
         }
         else if (integrationParametersNoLoad != null)
         {
            jointAccelerationIntegrationCommand.setJointParameters(jointIdx, integrationParametersNoLoad);
         }
      }
   }

   public JointAccelerationIntegrationCommand getJointAccelerationIntegrationCommand()
   {
      return jointAccelerationIntegrationCommand;
   }
}
