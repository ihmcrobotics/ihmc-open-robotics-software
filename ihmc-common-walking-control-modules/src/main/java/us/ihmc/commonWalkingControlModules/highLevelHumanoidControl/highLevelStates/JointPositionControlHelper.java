package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.configurations.ParameterTools;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.parameters.JointAccelerationIntegrationParametersReadOnly;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class JointPositionControlHelper
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final JointAccelerationIntegrationParametersReadOnly[] accelerationIntegrationSettings;

   private final JointAccelerationIntegrationCommand jointAccelerationIntegrationCommand = new JointAccelerationIntegrationCommand();
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

   public JointPositionControlHelper(WalkingControllerParameters walkingControllerParameters, OneDoFJoint[] joints, YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);

      Map<String, JointAccelerationIntegrationParametersReadOnly> parameterJointNameMap = new HashMap<>();
      ParameterTools.extractAccelerationIntegrationParameterMap(walkingControllerParameters.getJointAccelerationIntegrationParameters(),
                                                                parameterJointNameMap, registry);

      Map<String, OneDoFJoint> jointNameMap = new HashMap<>();
      for (OneDoFJoint joint : joints)
      {
         jointNameMap.put(joint.getName(), joint);
      }

      List<String> positionControlledJoints = walkingControllerParameters.getOrCreatePositionControlledJoints();
      for (int jointIdx = 0; jointIdx < positionControlledJoints.size(); jointIdx++)
      {
         String jointName = positionControlledJoints.get(jointIdx);
         OneDoFJoint joint = jointNameMap.get(jointName);
         if (joint == null)
         {
            throw new RuntimeException("Joint " + jointName + " is not a " + OneDoFJoint.class.getSimpleName() + " on the robot.");
         }
         lowLevelOneDoFJointDesiredDataHolder.registerJointWithEmptyData(joint);
         lowLevelOneDoFJointDesiredDataHolder.setJointControlMode(joint, JointDesiredControlMode.POSITION);
      }

      boolean integrateAllJoints = walkingControllerParameters.enableJointAccelerationIntegrationForAllJoints();
      List<String> integratingJoints = getJointsWithAccelerationIntegration(positionControlledJoints, integrateAllJoints, joints);
      accelerationIntegrationSettings = new JointAccelerationIntegrationParametersReadOnly[integratingJoints.size()];
      for (int jointIdx = 0; jointIdx < integratingJoints.size(); jointIdx++)
      {
         String jointName = integratingJoints.get(jointIdx);

         OneDoFJoint joint = jointNameMap.get(jointName);
         if (joint == null)
         {
            throw new RuntimeException("Joint " + jointName + " is not a " + OneDoFJoint.class.getSimpleName() + " on the robot.");
         }
         jointAccelerationIntegrationCommand.addJointToComputeDesiredPositionFor(joint);

         JointAccelerationIntegrationParametersReadOnly integrationParameters = parameterJointNameMap.get(jointName);
         if (integrationParameters == null)
         {
            PrintTools.warn("Integrating the desired acceleration for " + jointName + " but no integration parameters are defined - using defaults.");
         }
         accelerationIntegrationSettings[jointIdx] = integrationParameters;
      }
   }

   public void update()
   {
      for (int jointIdx = 0; jointIdx < accelerationIntegrationSettings.length; jointIdx++)
      {
         JointAccelerationIntegrationParametersReadOnly integrationParameters = accelerationIntegrationSettings[jointIdx];
         if (integrationParameters != null)
         {
            jointAccelerationIntegrationCommand.setJointParameters(jointIdx, integrationParameters);
         }
      }
   }

   public JointAccelerationIntegrationCommand getJointAccelerationIntegrationCommand()
   {
      return jointAccelerationIntegrationCommand;
   }

   public LowLevelOneDoFJointDesiredDataHolder getLowLevelOneDoFJointDesiredDataHolder()
   {
      return lowLevelOneDoFJointDesiredDataHolder;
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
