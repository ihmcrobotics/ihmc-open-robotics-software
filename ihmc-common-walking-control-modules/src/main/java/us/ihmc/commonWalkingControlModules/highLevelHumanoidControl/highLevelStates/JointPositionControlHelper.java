package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commonWalkingControlModules.configurations.ParameterTools;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.parameters.JointAccelerationIntegrationParametersReadOnly;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class JointPositionControlHelper
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final JointAccelerationIntegrationParametersReadOnly[] accelerationIntegrationSettings;
   private final JointAccelerationIntegrationParametersReadOnly[] accelerationIntegrationSettingsNoLoad;
   private final SideDependentList<TIntArrayList> legJointIndices = new SideDependentList<>();

   private final JointAccelerationIntegrationCommand jointAccelerationIntegrationCommand = new JointAccelerationIntegrationCommand();
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

   public JointPositionControlHelper(WalkingControllerParameters walkingControllerParameters, OneDoFJoint[] joints, FullHumanoidRobotModel fullRobotModel,
                                     YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);

      Map<String, JointAccelerationIntegrationParametersReadOnly> parameterJointNameMap = new HashMap<>();
      ParameterTools.extractAccelerationIntegrationParameterMap("", walkingControllerParameters.getJointAccelerationIntegrationParameters(),
                                                                parameterJointNameMap, registry);

      Map<String, JointAccelerationIntegrationParametersReadOnly> parameterJointNameMapNoLoad = new HashMap<>();
      ParameterTools.extractAccelerationIntegrationParameterMap("NoLoad", walkingControllerParameters.getJointAccelerationIntegrationParametersNoLoad(),
                                                                parameterJointNameMapNoLoad, registry);

      Map<String, OneDoFJoint> jointNameMap = new HashMap<>();
      for (OneDoFJoint joint : joints)
      {
         jointNameMap.put(joint.getName(), joint);
      }

      RigidBody pelvis = fullRobotModel.getPelvis();
      SideDependentList<OneDoFJoint[]> legJoints = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         legJointIndices.put(robotSide, new TIntArrayList());
         RigidBody foot = fullRobotModel.getFoot(robotSide);
         legJoints.put(robotSide, ScrewTools.filterJoints(ScrewTools.createJointPath(pelvis, foot), OneDoFJoint.class));
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
      accelerationIntegrationSettingsNoLoad = new JointAccelerationIntegrationParametersReadOnly[integratingJoints.size()];
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
         accelerationIntegrationSettingsNoLoad[jointIdx] = parameterJointNameMapNoLoad.get(jointName);

         // find the indices of the leg joints
         for (RobotSide robotSide : RobotSide.values)
         {
            for (OneDoFJoint legJoint : legJoints.get(robotSide))
            {
               if (legJoint.getName().equals(jointName))
               {
                  legJointIndices.get(robotSide).add(jointIdx);
               }
            }
         }
      }
   }

   public void update(WalkingStateEnum walkingStateEnum)
   {
      for (int jointIdx = 0; jointIdx < accelerationIntegrationSettings.length; jointIdx++)
      {
         JointAccelerationIntegrationParametersReadOnly integrationParameters = accelerationIntegrationSettings[jointIdx];
         if (walkingStateEnum.isSingleSupport())
         {
            boolean isLegJoint = legJointIndices.get(walkingStateEnum.getSupportSide().getOppositeSide()).contains(jointIdx);
            if (isLegJoint && accelerationIntegrationSettingsNoLoad[jointIdx] != null)
            {
               // we have a leg joint that is not loaded!
               integrationParameters = accelerationIntegrationSettingsNoLoad[jointIdx];
            }
         }

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
