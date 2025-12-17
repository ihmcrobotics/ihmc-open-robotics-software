package us.ihmc.openAlexander.parameters.diagnostic;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import us.ihmc.openAlexander.ZuluJointMap;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.configurations.GroupParameter;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.sensorProcessing.outputData.JointDesiredBehaviorReadOnly;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
import us.ihmc.wholeBodyController.diagnostics.AutomatedDiagnosticConfiguration;
import us.ihmc.wholeBodyController.diagnostics.DiagnosticParameters;

import static us.ihmc.wholeBodyController.parameters.HighLevelParametersTools.getLeftAndRightJointNames;

public class AlexanderDiagnosticParameters extends DiagnosticParameters
{
   private final RobotTarget target;
   private final ZuluJointMap jointMap;
   private final HumanoidRobotSensorInformation sensorInformation;
   private final HighLevelControllerParameters highLevelControllerParameters;

   public AlexanderDiagnosticParameters(RobotTarget target,
                                    ZuluJointMap jointMap,
                                    HumanoidRobotSensorInformation sensorInformation,
                                    HighLevelControllerParameters highLevelControllerParameters)
   {
      this.target = target;
      this.jointMap = jointMap;
      this.sensorInformation = sensorInformation;
      this.highLevelControllerParameters = highLevelControllerParameters;
   }

   @Override
   public void scheduleCheckUps(AutomatedDiagnosticConfiguration configuration)
   {
      configuration.addWait(1.0);
      //      configuration.addJointCheckUps(defaultJointCheckUpConfiguration(jointMap));
      configuration.addPelvisIMUCheckUpDiagnostic(DiagnosticParameters.defaultPelvisIMUCheckUp(sensorInformation, jointMap));
   }

   public static List<List<String>> defaultJointCheckUpConfiguration(HumanoidJointNameMap jointMap)
   {
      List<List<String>> jointNames = new ArrayList<>();
      jointNames.add(getLeftAndRightJointNames(jointMap, LegJointName.ANKLE_ROLL));
      jointNames.add(getLeftAndRightJointNames(jointMap, LegJointName.ANKLE_PITCH));
      jointNames.add(getLeftAndRightJointNames(jointMap, LegJointName.KNEE_PITCH));
      jointNames.add(getLeftAndRightJointNames(jointMap, LegJointName.HIP_PITCH));
      jointNames.add(getLeftAndRightJointNames(jointMap, LegJointName.HIP_ROLL));
      jointNames.add(getLeftAndRightJointNames(jointMap, LegJointName.HIP_YAW));
      jointNames.add(Collections.singletonList(jointMap.getSpineJointName(SpineJointName.SPINE_YAW)));
      jointNames.add(Collections.singletonList(jointMap.getSpineJointName(SpineJointName.SPINE_PITCH)));
      jointNames.add(Collections.singletonList(jointMap.getSpineJointName(SpineJointName.SPINE_ROLL)));
      return jointNames;
   }

   @Override
   public double getInitialJointSplineDuration()
   {
      return target == RobotTarget.REAL_ROBOT ? 1.0 : 1.0;
   }

   @Override
   public List<GroupParameter<JointDesiredBehaviorReadOnly>> getDesiredJointBehaviors()
   {
      return highLevelControllerParameters.getDesiredJointBehaviors(HighLevelControllerName.EXIT_WALKING);
   }

   @Override
   public WholeBodySetpointParameters getDiagnosticSetpoints()
   {
      return highLevelControllerParameters.getStandPrepParameters();
   }

   @Override
   public boolean doIdleControlUntilRobotIsAlive()
   {
      return true;
   }

   @Override
   public double getCheckUpOscillationPositionFrequency()
   {
      return target == RobotTarget.REAL_ROBOT ? 5.0 : 5.0; // 10.0Hz seems to give better delay estimation
   }

   @Override
   public double getCheckUpOscillationPositionAmplitude()
   {
      return target == RobotTarget.REAL_ROBOT ? 0.01 : 0.05;
   }
}
