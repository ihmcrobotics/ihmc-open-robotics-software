package us.ihmc.zulu.parameters.diagnostic;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import us.ihmc.zulu.ZuluJointMap;
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

public class ZuluDiagnosticParameters extends DiagnosticParameters
{
   private final ZuluJointMap jointMap;
   private final HumanoidRobotSensorInformation sensorInformation;
   private final HighLevelControllerParameters highLevelControllerParameters;

   public ZuluDiagnosticParameters(ZuluJointMap jointMap,
                                   HumanoidRobotSensorInformation sensorInformation,
                                   HighLevelControllerParameters highLevelControllerParameters)
   {
      this.jointMap = jointMap;
      this.sensorInformation = sensorInformation;
      this.highLevelControllerParameters = highLevelControllerParameters;
   }

   @Override
   public void scheduleCheckUps(AutomatedDiagnosticConfiguration configuration)
   {
      configuration.addWait(1.0);
      configuration.addPelvisIMUCheckUpDiagnostic(DiagnosticParameters.defaultPelvisIMUCheckUp(sensorInformation, jointMap));
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
}
