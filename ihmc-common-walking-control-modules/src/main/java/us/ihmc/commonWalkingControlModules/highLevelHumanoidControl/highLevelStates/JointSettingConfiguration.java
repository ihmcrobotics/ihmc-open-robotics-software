package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.GroupParameter;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.parameters.JointAccelerationIntegrationParametersReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.sensorProcessing.outputData.JointDesiredBehaviorReadOnly;

public interface JointSettingConfiguration
{
   public List<GroupParameter<JointAccelerationIntegrationParametersReadOnly>> getJointAccelerationIntegrationParameters();

   public List<GroupParameter<JointAccelerationIntegrationParametersReadOnly>> getJointAccelerationIntegrationParametersUnderLoad();

   public List<GroupParameter<JointDesiredBehaviorReadOnly>> getDesiredJointBehaviors();

   public List<GroupParameter<JointDesiredBehaviorReadOnly>> getDesiredJointBehaviorsUnderLoad();

   static JointSettingConfiguration extract(HighLevelControllerParameters parameters, HighLevelControllerName stateEnum)
   {
      return new JointSettingConfiguration()
      {
         @Override
         public List<GroupParameter<JointAccelerationIntegrationParametersReadOnly>> getJointAccelerationIntegrationParametersUnderLoad()
         {
            return parameters.getJointAccelerationIntegrationParametersUnderLoad();
         }

         @Override
         public List<GroupParameter<JointAccelerationIntegrationParametersReadOnly>> getJointAccelerationIntegrationParameters()
         {
            return parameters.getJointAccelerationIntegrationParameters();
         }

         @Override
         public List<GroupParameter<JointDesiredBehaviorReadOnly>> getDesiredJointBehaviorsUnderLoad()
         {
            return parameters.getDesiredJointBehaviorsUnderLoad(stateEnum);
         }

         @Override
         public List<GroupParameter<JointDesiredBehaviorReadOnly>> getDesiredJointBehaviors()
         {
            return parameters.getDesiredJointBehaviors(stateEnum);
         }
      };
   }
}
