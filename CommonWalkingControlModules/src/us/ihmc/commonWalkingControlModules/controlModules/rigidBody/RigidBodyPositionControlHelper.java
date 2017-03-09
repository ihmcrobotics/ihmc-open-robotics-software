package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationSettings;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelJointControlMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class RigidBodyPositionControlHelper
{
   private final IntegerYoVariable numberOfPositionControlledJoints;

   private final DoubleYoVariable[] accelerationIntegrationAlphaPosition;
   private final DoubleYoVariable[] accelerationIntegrationAlphaVelocity;
   private final DoubleYoVariable[] accelerationIntegrationMaxPositionError;
   private final DoubleYoVariable[] accelerationIntegrationMaxVelocity;

   private final JointAccelerationIntegrationCommand jointAccelerationIntegrationCommand = new JointAccelerationIntegrationCommand();
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

   public RigidBodyPositionControlHelper(String namePrefix, OneDoFJoint[] jointsToControl, List<String> positionControlledJointNames,
         Map<String, JointAccelerationIntegrationSettings> integrationSettings, YoVariableRegistry parentRegistry)
   {
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix + "PositionControlHelper");

      ArrayList<OneDoFJoint> positionControlledJoints = new ArrayList<>();
      for (int jointIdx = 0; jointIdx < jointsToControl.length; jointIdx++)
      {
         OneDoFJoint joint = jointsToControl[jointIdx];
         if (positionControlledJointNames.contains(joint.getName()))
            positionControlledJoints.add(joint);
      }

      numberOfPositionControlledJoints = new IntegerYoVariable(namePrefix + "NumberOfPositionControlledJoints", registry);
      numberOfPositionControlledJoints.set(positionControlledJoints.size());

      accelerationIntegrationAlphaPosition = new DoubleYoVariable[numberOfPositionControlledJoints.getIntegerValue()];
      accelerationIntegrationAlphaVelocity = new DoubleYoVariable[numberOfPositionControlledJoints.getIntegerValue()];
      accelerationIntegrationMaxPositionError = new DoubleYoVariable[numberOfPositionControlledJoints.getIntegerValue()];
      accelerationIntegrationMaxVelocity = new DoubleYoVariable[numberOfPositionControlledJoints.getIntegerValue()];

      for (int jointIdx = 0; jointIdx < numberOfPositionControlledJoints.getIntegerValue(); jointIdx++)
      {
         OneDoFJoint positionControlledJoint = positionControlledJoints.get(jointIdx);
         String jointName = positionControlledJoint.getName();

         String prefix = namePrefix + "_" + jointName + "_accelerationIntegration";
         accelerationIntegrationAlphaPosition[jointIdx] = new DoubleYoVariable(prefix + "AlphaPosition", registry);
         accelerationIntegrationAlphaVelocity[jointIdx] = new DoubleYoVariable(prefix + "AlphaVelocity", registry);
         accelerationIntegrationMaxPositionError[jointIdx] = new DoubleYoVariable(prefix + "MaxPositionError", registry);
         accelerationIntegrationMaxVelocity[jointIdx] = new DoubleYoVariable(prefix + "MaxVelocity", registry);

         if (!integrationSettings.containsKey(jointName))
            throw new RuntimeException("Attempting to position control a joint that has no acceleration integration settings.");

         JointAccelerationIntegrationSettings jointIntegrationSettings = integrationSettings.get(jointName);
         accelerationIntegrationAlphaPosition[jointIdx].set(jointIntegrationSettings.getAlphaPosition());
         accelerationIntegrationAlphaVelocity[jointIdx].set(jointIntegrationSettings.getAlphaVelocity());
         accelerationIntegrationMaxPositionError[jointIdx].set(jointIntegrationSettings.getMaxPositionError());
         accelerationIntegrationMaxVelocity[jointIdx].set(jointIntegrationSettings.getMaxVelocity());

         lowLevelOneDoFJointDesiredDataHolder.registerJointWithEmptyData(positionControlledJoint);
         lowLevelOneDoFJointDesiredDataHolder.setJointControlMode(positionControlledJoint, LowLevelJointControlMode.POSITION_CONTROL);
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
         double alphaPosition = accelerationIntegrationAlphaPosition[jointIdx].getDoubleValue();
         double alphaVelocity = accelerationIntegrationAlphaVelocity[jointIdx].getDoubleValue();
         jointAccelerationIntegrationCommand.setJointAlphas(jointIdx, alphaPosition, alphaVelocity);

         double maxPositionError = accelerationIntegrationMaxPositionError[jointIdx].getDoubleValue();
         double maxVelocity = accelerationIntegrationMaxVelocity[jointIdx].getDoubleValue();
         jointAccelerationIntegrationCommand.setJointMaxima(jointIdx, maxPositionError, maxVelocity);
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
