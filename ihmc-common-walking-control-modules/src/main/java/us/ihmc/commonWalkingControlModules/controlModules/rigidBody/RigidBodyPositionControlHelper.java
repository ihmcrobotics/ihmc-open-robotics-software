package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.parameters.JointAccelerationIntegrationParametersReadOnly;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

public class RigidBodyPositionControlHelper
{
   private final YoInteger numberOfPositionControlledJoints;

   private final JointAccelerationIntegrationParametersReadOnly[] accelerationIntegrationSettings;

   private final JointAccelerationIntegrationCommand jointAccelerationIntegrationCommand = new JointAccelerationIntegrationCommand();
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

   public RigidBodyPositionControlHelper(String namePrefix, OneDoFJoint[] jointsToControl, List<String> positionControlledJointNames,
         Map<String, JointAccelerationIntegrationParametersReadOnly> integrationSettings, YoVariableRegistry parentRegistry)
   {
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix + "PositionControlHelper");

      ArrayList<OneDoFJoint> positionControlledJoints = new ArrayList<>();
      for (int jointIdx = 0; jointIdx < jointsToControl.length; jointIdx++)
      {
         OneDoFJoint joint = jointsToControl[jointIdx];
         if (positionControlledJointNames.contains(joint.getName()))
            positionControlledJoints.add(joint);
      }

      int numberOfJoints = positionControlledJoints.size();
      numberOfPositionControlledJoints = new YoInteger(namePrefix + "NumberOfPositionControlledJoints", registry);
      numberOfPositionControlledJoints.set(numberOfJoints);
      accelerationIntegrationSettings = new JointAccelerationIntegrationParametersReadOnly[numberOfJoints];

      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         OneDoFJoint positionControlledJoint = positionControlledJoints.get(jointIdx);
         String jointName = positionControlledJoint.getName();
         if (!integrationSettings.containsKey(jointName))
            throw new RuntimeException("Attempting to position control a joint that has no acceleration integration settings.");

         accelerationIntegrationSettings[jointIdx] = integrationSettings.get(jointName);

         lowLevelOneDoFJointDesiredDataHolder.registerJointWithEmptyData(positionControlledJoint);
         lowLevelOneDoFJointDesiredDataHolder.setJointControlMode(positionControlledJoint, JointDesiredControlMode.POSITION);
         jointAccelerationIntegrationCommand.addJointToComputeDesiredPositionFor(positionControlledJoint);
      }

      parentRegistry.addChild(registry);
   }

   public boolean hasPositionControlledJoints()
   {
      return numberOfPositionControlledJoints.getIntegerValue() != 0;
   }

   public void update()
   {
      for (int jointIdx = 0; jointIdx < numberOfPositionControlledJoints.getIntegerValue(); jointIdx++)
      {
         jointAccelerationIntegrationCommand.setJointParameters(jointIdx, accelerationIntegrationSettings[jointIdx]);
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

}
