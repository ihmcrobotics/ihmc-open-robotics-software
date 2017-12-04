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

   public AccelerationIntegrationParameterHelper(HighLevelControllerParameters parameters, List<String> positionControlledJoints, OneDoFJoint[] joints,
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

      Map<String, OneDoFJoint> jointNameMap = new HashMap<>();
      for (OneDoFJoint joint : joints)
      {
         jointNameMap.put(joint.getName(), joint);
      }

      boolean integrateAllJoints = parameters.enableJointAccelerationIntegrationForAllJoints();
      List<String> integratingJoints = getJointsWithAccelerationIntegration(positionControlledJoints, integrateAllJoints, joints);
      jointNames = new String[integratingJoints.size()];
      jointsLoaded = new YoBoolean[integratingJoints.size()];
      accelerationIntegrationSettingsNoLoad = new JointAccelerationIntegrationParametersReadOnly[integratingJoints.size()];
      accelerationIntegrationSettingsLoaded = new JointAccelerationIntegrationParametersReadOnly[integratingJoints.size()];

      for (int jointIdx = 0; jointIdx < integratingJoints.size(); jointIdx++)
      {
         String jointName = integratingJoints.get(jointIdx);
         jointNames[jointIdx] = jointName;
         jointsLoaded[jointIdx] = new YoBoolean(jointName + "_isUnderLoad", registry);

         OneDoFJoint joint = jointNameMap.get(jointName);
         if (joint == null)
         {
            throw new RuntimeException("Joint " + jointName + " is not a " + OneDoFJoint.class.getSimpleName() + " on the robot.");
         }
         jointAccelerationIntegrationCommand.addJointToComputeDesiredPositionFor(joint);

         JointAccelerationIntegrationParametersReadOnly integrationParametersNoLoad = parameterJointNameMapNoLoad.get(jointName);
         if (integrationParametersNoLoad == null)
         {
            PrintTools.warn("(NoLoad) No integration parameters for joint " + jointName + " defined. Using default values.");
         }
         accelerationIntegrationSettingsNoLoad[jointIdx] = integrationParametersNoLoad;

         JointAccelerationIntegrationParametersReadOnly integrationParametersLoaded = parameterJointNameMapLoaded.get(jointName);
         if (integrationParametersLoaded == null)
         {
            PrintTools.warn("(Loaded) No integration parameters for joint " + jointName + " defined. Using No-Load parameters.");
         }
         accelerationIntegrationSettingsLoaded[jointIdx] = integrationParametersLoaded;
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

   public static List<String> getJointsWithAccelerationIntegration(List<String> positionControlledJoints, boolean enableForAllJoints, OneDoFJoint[] joints)
   {
      List<String> ret = new ArrayList<>();
      if (enableForAllJoints)
      {
         for (OneDoFJoint joint : joints)
         {
            ret.add(joint.getName());
         }
      }
      else
      {
         ret.addAll(positionControlledJoints);
      }
      return ret;
   }
}
