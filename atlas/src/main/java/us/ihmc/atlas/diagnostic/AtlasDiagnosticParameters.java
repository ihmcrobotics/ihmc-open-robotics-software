package us.ihmc.atlas.diagnostic;

import static us.ihmc.atlas.parameters.AtlasHighLevelControllerParameters.configureBehavior;
import static us.ihmc.atlas.parameters.AtlasHighLevelControllerParameters.configureSymmetricBehavior;
import static us.ihmc.sensorProcessing.outputData.JointDesiredControlMode.EFFORT;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.atlas.parameters.AtlasHighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.GroupParameter;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.sensorProcessing.outputData.JointDesiredBehaviorReadOnly;
import us.ihmc.wholeBodyController.diagnostics.DiagnosticParameters;

public class AtlasDiagnosticParameters extends DiagnosticParameters
{
   private final AtlasJointMap jointMap;
   private final boolean runningOnRealRobot;

   private final AtlasDiagnosticSetpoints setpoints;

   public AtlasDiagnosticParameters(AtlasJointMap jointMap, boolean runningOnRealRobot)
   {
      this.jointMap = jointMap;
      this.runningOnRealRobot = runningOnRealRobot;
      setpoints = new AtlasDiagnosticSetpoints(jointMap);
   }

   public static List<List<String>> defaultJointCheckUpConfiguration(HumanoidJointNameMap jointMap)
   {
      List<List<String>> jointNames = new ArrayList<>();
      jointNames.add(AtlasHighLevelControllerParameters.getLeftAndRightJointNames(jointMap, LegJointName.ANKLE_ROLL));
      jointNames.add(AtlasHighLevelControllerParameters.getLeftAndRightJointNames(jointMap, LegJointName.ANKLE_PITCH));
      jointNames.add(AtlasHighLevelControllerParameters.getLeftAndRightJointNames(jointMap, LegJointName.KNEE_PITCH));
      jointNames.add(AtlasHighLevelControllerParameters.getLeftAndRightJointNames(jointMap, LegJointName.HIP_PITCH));
      jointNames.add(AtlasHighLevelControllerParameters.getLeftAndRightJointNames(jointMap, LegJointName.HIP_ROLL));
      jointNames.add(AtlasHighLevelControllerParameters.getLeftAndRightJointNames(jointMap, LegJointName.HIP_YAW));
      jointNames.add(Collections.singletonList(jointMap.getSpineJointName(SpineJointName.SPINE_YAW)));
      jointNames.add(Collections.singletonList(jointMap.getSpineJointName(SpineJointName.SPINE_PITCH)));
      jointNames.add(Collections.singletonList(jointMap.getSpineJointName(SpineJointName.SPINE_ROLL)));
      jointNames.add(AtlasHighLevelControllerParameters.getLeftAndRightJointNames(jointMap, ArmJointName.SHOULDER_YAW));
      jointNames.add(AtlasHighLevelControllerParameters.getLeftAndRightJointNames(jointMap, ArmJointName.SHOULDER_ROLL));
      jointNames.add(AtlasHighLevelControllerParameters.getLeftAndRightJointNames(jointMap, ArmJointName.ELBOW_PITCH));
      jointNames.add(AtlasHighLevelControllerParameters.getLeftAndRightJointNames(jointMap, ArmJointName.ELBOW_ROLL));
      jointNames.add(AtlasHighLevelControllerParameters.getLeftAndRightJointNames(jointMap, ArmJointName.FIRST_WRIST_PITCH));
      jointNames.add(AtlasHighLevelControllerParameters.getLeftAndRightJointNames(jointMap, ArmJointName.WRIST_ROLL));
      jointNames.add(AtlasHighLevelControllerParameters.getLeftAndRightJointNames(jointMap, ArmJointName.SECOND_WRIST_PITCH));
      return jointNames;
   }

   @Override
   public double getInitialJointSplineDuration()
   {
      return runningOnRealRobot ? 10.0 : 1.0;
   }

   @Override
   public List<GroupParameter<JointDesiredBehaviorReadOnly>> getDesiredJointBehaviors()
   {
      List<GroupParameter<JointDesiredBehaviorReadOnly>> behaviors = new ArrayList<>();

      if (!runningOnRealRobot)
      {
         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SHOULDER_YAW, EFFORT, 200.0, 20.0);
         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SHOULDER_ROLL, EFFORT, 200.0, 20.0);
         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.ELBOW_PITCH, EFFORT, 100.0, 10.0);
         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.ELBOW_ROLL, EFFORT, 100.0, 10.0);
         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.FIRST_WRIST_PITCH, EFFORT, 50.0, 5.0);
         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.WRIST_ROLL, EFFORT, 10.0, 1.0);
         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SECOND_WRIST_PITCH, EFFORT, 0.5, 0.05);

         configureBehavior(behaviors, jointMap, NeckJointName.PROXIMAL_NECK_PITCH, EFFORT, 50.0, 5.0);
         configureBehavior(behaviors, jointMap, SpineJointName.SPINE_YAW, EFFORT, 400.0, 40.0);
         configureBehavior(behaviors, jointMap, SpineJointName.SPINE_PITCH, EFFORT, 400.0, 40.0);
         configureBehavior(behaviors, jointMap, SpineJointName.SPINE_ROLL, EFFORT, 400.0, 40.0);

         configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_YAW, EFFORT, 200.0, 20.0);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_ROLL, EFFORT, 200.0, 20.0);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_PITCH, EFFORT, 20.0, 20.0);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.KNEE_PITCH, EFFORT, 100.0, 10.0);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.ANKLE_PITCH, EFFORT, 50.0, 5.0);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.ANKLE_ROLL, EFFORT, 50.0, 5.0);
      }
      else
      {
         throw new RuntimeException("Add gains for real robot.");
      }

      return behaviors;
   }

   @Override
   public WholeBodySetpointParameters getDiagnosticSetpoints()
   {
      return setpoints;
   }
}
