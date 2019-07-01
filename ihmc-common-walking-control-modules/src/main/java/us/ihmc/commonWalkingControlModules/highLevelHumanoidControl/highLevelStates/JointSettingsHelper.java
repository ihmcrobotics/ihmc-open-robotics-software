package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.google.common.base.CaseFormat;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ParameterTools;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.parameters.JointAccelerationIntegrationParametersReadOnly;
import us.ihmc.commons.PrintTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredBehaviorReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class JointSettingsHelper
{
   private final YoVariableRegistry registry;

   private final String[] jointNames;
   private final boolean[] jointsLoaded;
   private final YoBoolean[] yoJointsLoaded;

   private final JointAccelerationIntegrationParametersReadOnly[] accelerationIntegrationSettingsNoLoad;
   private final JointAccelerationIntegrationParametersReadOnly[] accelerationIntegrationSettingsLoaded;

   private final JointDesiredBehaviorReadOnly[] jointDesiredBehaviorNoLoad;
   private final JointDesiredBehaviorReadOnly[] jointDesiredBehaviorLoaded;

   private final JointAccelerationIntegrationCommand jointAccelerationIntegrationCommand;
   private final JointDesiredOutputList stateSpecificJointSettings;

   private final JointLoadStatusProvider jointLoadStatusProvider;


   public JointSettingsHelper(HighLevelControllerParameters parameters, OneDoFJointBasics[] joints, JointLoadStatusProvider jointLoadStatusProvider,
                              HighLevelControllerName stateEnum, YoVariableRegistry parentRegistry)
   {
      this(JointSettingConfiguration.extract(parameters, stateEnum), joints, jointLoadStatusProvider,
           CaseFormat.UPPER_UNDERSCORE.to(CaseFormat.UPPER_CAMEL, stateEnum.toString()), parentRegistry);
   }

   public JointSettingsHelper(JointSettingConfiguration configuration, List<OneDoFJointBasics> joints, JointLoadStatusProvider jointLoadStatusProvider,
                              String stateName, YoVariableRegistry parentRegistry)
   {
      this(configuration, joints.toArray(new OneDoFJointBasics[joints.size()]), jointLoadStatusProvider, stateName, parentRegistry);
   }

   public JointSettingsHelper(JointSettingConfiguration configuration, OneDoFJointBasics[] joints, JointLoadStatusProvider jointLoadStatusProvider, String stateName,
                              YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(stateName + "JointSettings");
      parentRegistry.addChild(registry);

      jointAccelerationIntegrationCommand = new JointAccelerationIntegrationCommand();
      stateSpecificJointSettings = new JointDesiredOutputList(joints);
      this.jointLoadStatusProvider = jointLoadStatusProvider;

      // TODO: this is not state dependent
      // For now we can not load different parameters from a parameter class for different states.
      Map<String, JointAccelerationIntegrationParametersReadOnly> parameterJointNameMapNoLoad = new HashMap<>();
      ParameterTools.extractAccelerationIntegrationParameterMap("NoLoad", configuration.getJointAccelerationIntegrationParameters(),
                                                                parameterJointNameMapNoLoad, registry);
      Map<String, JointAccelerationIntegrationParametersReadOnly> parameterJointNameMapLoaded = new HashMap<>();
      ParameterTools.extractAccelerationIntegrationParameterMap("Loaded", configuration.getJointAccelerationIntegrationParametersUnderLoad(),
                                                                parameterJointNameMapLoaded, registry);

      // TODO: these use the same default values
      Map<String, JointDesiredBehaviorReadOnly> jointBehaviorMapNoLoad = new HashMap<>();
      ParameterTools.extractJointBehaviorMap("NoLoad", configuration.getDesiredJointBehaviors(), jointBehaviorMapNoLoad, registry);
      Map<String, JointDesiredBehaviorReadOnly> jointBehaviorMapLoaded = new HashMap<>();
      ParameterTools.extractJointBehaviorMap("Loaded", configuration.getDesiredJointBehaviorsUnderLoad(), jointBehaviorMapLoaded, registry);

      jointNames = new String[joints.length];
      yoJointsLoaded = new YoBoolean[joints.length];
      jointsLoaded = new boolean[joints.length];
      accelerationIntegrationSettingsNoLoad = new JointAccelerationIntegrationParametersReadOnly[joints.length];
      accelerationIntegrationSettingsLoaded = new JointAccelerationIntegrationParametersReadOnly[joints.length];
      jointDesiredBehaviorNoLoad = new JointDesiredBehaviorReadOnly[joints.length];
      jointDesiredBehaviorLoaded = new JointDesiredBehaviorReadOnly[joints.length];

      List<String> jointsWithoutParameters = new ArrayList<>();
      List<String> jointsWithoutBehaviors = new ArrayList<>();
      for (int jointIdx = 0; jointIdx < joints.length; jointIdx++)
      {
         OneDoFJointBasics joint = joints[jointIdx];
         String jointName = joint.getName();
         jointNames[jointIdx] = jointName;
         jointAccelerationIntegrationCommand.addJointToComputeDesiredPositionFor(joint);

         JointAccelerationIntegrationParametersReadOnly integrationParametersNoLoad = parameterJointNameMapNoLoad.get(jointName);
         JointAccelerationIntegrationParametersReadOnly integrationParametersLoaded = parameterJointNameMapLoaded.get(jointName);
         accelerationIntegrationSettingsNoLoad[jointIdx] = integrationParametersNoLoad;
         accelerationIntegrationSettingsLoaded[jointIdx] = integrationParametersLoaded;
         if (integrationParametersNoLoad == null)
         {
            jointsWithoutParameters.add(jointName);
         }

         JointDesiredBehaviorReadOnly desiredBehaviorNoLoad = jointBehaviorMapNoLoad.get(jointName);
         JointDesiredBehaviorReadOnly desiredBehaviorLoaded = jointBehaviorMapLoaded.get(jointName);
         jointDesiredBehaviorNoLoad[jointIdx] = desiredBehaviorNoLoad;
         jointDesiredBehaviorLoaded[jointIdx] = desiredBehaviorLoaded;
         if (desiredBehaviorNoLoad == null)
         {
            jointsWithoutBehaviors.add(jointName);
         }

         if ((integrationParametersNoLoad == null && desiredBehaviorNoLoad == null) || (integrationParametersLoaded == null && desiredBehaviorLoaded == null))
         {
            yoJointsLoaded[jointIdx] = null;
         }
         else
         {
            yoJointsLoaded[jointIdx] = new YoBoolean(jointName + "_isUnderLoad", registry);
         }
      }

      if (!jointsWithoutParameters.isEmpty())
      {
         PrintTools.warn("In State " + stateName + "\n"
               + "Got joints without acceleration integration parameters.\n"
               + "Will use default values for: " + jointsWithoutParameters);
      }
      if (!jointsWithoutBehaviors.isEmpty())
      {
         throw new RuntimeException("In State " + stateName + "\n"
               + "Must define joint behaviors for: " + jointsWithoutBehaviors);
      }
   }

   public void update()
   {
      for (int jointIdx = 0; jointIdx < jointNames.length; jointIdx++)
      {
         JointDesiredOutputBasics jointDesiredOutput = stateSpecificJointSettings.getJointDesiredOutput(jointIdx);
         boolean isLoaded = jointLoadStatusProvider.isJointLoadBearing(jointNames[jointIdx]);
         boolean wasLoaded = jointsLoaded[jointIdx];
         jointDesiredOutput.setResetIntegrators(isLoaded != wasLoaded);
         jointsLoaded[jointIdx] = isLoaded;

         if (yoJointsLoaded[jointIdx] != null)
         {
            yoJointsLoaded[jointIdx].set(isLoaded);
         }

         JointAccelerationIntegrationParametersReadOnly integrationParametersNoLoad = accelerationIntegrationSettingsNoLoad[jointIdx];
         JointAccelerationIntegrationParametersReadOnly integrationParametersLoaded = accelerationIntegrationSettingsLoaded[jointIdx];

         if (isLoaded && integrationParametersLoaded != null)
         { // The joint is loaded and we have parameters for this case.
            jointAccelerationIntegrationCommand.setJointParameters(jointIdx, integrationParametersLoaded);
            jointDesiredOutput.setVelocityIntegrationBreakFrequency(integrationParametersLoaded.getVelocityBreakFrequency());
            jointDesiredOutput.setPositionIntegrationBreakFrequency(integrationParametersLoaded.getPositionBreakFrequency());
            jointDesiredOutput.setMaxPositionError(integrationParametersLoaded.getMaxPositionError());
            jointDesiredOutput.setMaxVelocityError(integrationParametersLoaded.getMaxVelocity());
         }
         else if (integrationParametersNoLoad != null)
         { // The joint is not loaded or we do not have parameters for the loaded joint but we have default no load parameters.
            jointAccelerationIntegrationCommand.setJointParameters(jointIdx, integrationParametersNoLoad);
            jointDesiredOutput.setVelocityIntegrationBreakFrequency(integrationParametersNoLoad.getVelocityBreakFrequency());
            jointDesiredOutput.setPositionIntegrationBreakFrequency(integrationParametersNoLoad.getPositionBreakFrequency());
            jointDesiredOutput.setMaxPositionError(integrationParametersNoLoad.getMaxPositionError());
            jointDesiredOutput.setMaxVelocityError(integrationParametersNoLoad.getMaxVelocity());
         }

         JointDesiredBehaviorReadOnly desiredBehaviorNoLoad = jointDesiredBehaviorNoLoad[jointIdx];
         JointDesiredBehaviorReadOnly desiredBehaviorLoaded = jointDesiredBehaviorLoaded[jointIdx];

         if (isLoaded && desiredBehaviorLoaded != null)
         { // The joint is loaded and we have parameters for this case.
            jointDesiredOutput.setControlMode(desiredBehaviorLoaded.getControlMode());
            jointDesiredOutput.setStiffness(desiredBehaviorLoaded.getStiffness());
            jointDesiredOutput.setDamping(desiredBehaviorLoaded.getDamping());
            jointDesiredOutput.setMasterGain(desiredBehaviorLoaded.getMasterGain());
            jointDesiredOutput.setVelocityScaling(desiredBehaviorLoaded.getVelocityScaling());
         }
         else
         { // The joint is not loaded or we do not have parameters for the loaded joint.
            jointDesiredOutput.setControlMode(desiredBehaviorNoLoad.getControlMode());
            jointDesiredOutput.setStiffness(desiredBehaviorNoLoad.getStiffness());
            jointDesiredOutput.setDamping(desiredBehaviorNoLoad.getDamping());
            jointDesiredOutput.setMasterGain(desiredBehaviorNoLoad.getMasterGain());
            jointDesiredOutput.setVelocityScaling(desiredBehaviorNoLoad.getVelocityScaling());
         }
      }
   }

   public JointAccelerationIntegrationCommand getJointAccelerationIntegrationCommand()
   {
      return jointAccelerationIntegrationCommand;
   }

   public JointDesiredOutputList getStateSpecificJointSettings()
   {
      return stateSpecificJointSettings;
   }
}
