package us.ihmc.valkyrie.diagnostic;

import static us.ihmc.sensorProcessing.outputData.JointDesiredControlMode.POSITION;
import static us.ihmc.valkyrie.ValkyrieHighLevelControllerParameters.configureBehavior;
import static us.ihmc.valkyrie.ValkyrieHighLevelControllerParameters.configureSymmetricBehavior;
import static us.ihmc.valkyrie.ValkyrieHighLevelControllerParameters.getLeftAndRightJointNames;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.GroupParameter;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.outputData.JointDesiredBehaviorReadOnly;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;
import us.ihmc.valkyrie.parameters.ValkyrieOrderedJointMap;
import us.ihmc.wholeBodyController.diagnostics.DiagnosticParameters;

public class ValkyrieDiagnosticParameters extends DiagnosticParameters
{
   private final HumanoidJointNameMap jointMap;
   private final boolean runningOnRealRobot;

   private final boolean ignoreAllNeckJoints = true;
   private final boolean ignoreAllArmJoints = false;
   private final boolean ignoreAllLegJoints = false;
   private final boolean ignoreAllSpineJoints = false;

   private final ValkyrieDiagnosticSetpoints setpoints;

   public ValkyrieDiagnosticParameters(ValkyrieJointMap jointMap, boolean runningOnRealRobot)
   {
      this.jointMap = jointMap;
      this.runningOnRealRobot = runningOnRealRobot;
      setpoints = new ValkyrieDiagnosticSetpoints(jointMap);
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
      jointNames.add(getLeftAndRightJointNames(jointMap, ArmJointName.SHOULDER_PITCH));
      jointNames.add(getLeftAndRightJointNames(jointMap, ArmJointName.SHOULDER_ROLL));
      jointNames.add(getLeftAndRightJointNames(jointMap, ArmJointName.SHOULDER_YAW));
      jointNames.add(getLeftAndRightJointNames(jointMap, ArmJointName.ELBOW_PITCH));
      return jointNames;
   }

   @Override
   public double getInitialJointSplineDuration()
   {
      return runningOnRealRobot ? 10.0 : 1.0;
   }

   @Override
   public List<String> getJointsToIgnoreDuringDiagnostic()
   {
      List<String> jointToIgnoreList = new ArrayList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         String[] forcedSideJointNames = ValkyrieOrderedJointMap.forcedSideDependentJointNames.get(robotSide);
         jointToIgnoreList.add(forcedSideJointNames[ValkyrieOrderedJointMap.LeftIndexFingerPitch1]);
         jointToIgnoreList.add(forcedSideJointNames[ValkyrieOrderedJointMap.LeftIndexFingerPitch2]);
         jointToIgnoreList.add(forcedSideJointNames[ValkyrieOrderedJointMap.LeftIndexFingerPitch3]);
         jointToIgnoreList.add(forcedSideJointNames[ValkyrieOrderedJointMap.LeftMiddleFingerPitch1]);
         jointToIgnoreList.add(forcedSideJointNames[ValkyrieOrderedJointMap.LeftMiddleFingerPitch2]);
         jointToIgnoreList.add(forcedSideJointNames[ValkyrieOrderedJointMap.LeftMiddleFingerPitch3]);
         jointToIgnoreList.add(forcedSideJointNames[ValkyrieOrderedJointMap.LeftPinkyPitch1]);
         jointToIgnoreList.add(forcedSideJointNames[ValkyrieOrderedJointMap.LeftPinkyPitch2]);
         jointToIgnoreList.add(forcedSideJointNames[ValkyrieOrderedJointMap.LeftPinkyPitch3]);
         jointToIgnoreList.add(forcedSideJointNames[ValkyrieOrderedJointMap.LeftThumbRoll]);
         jointToIgnoreList.add(forcedSideJointNames[ValkyrieOrderedJointMap.LeftThumbPitch1]);
         jointToIgnoreList.add(forcedSideJointNames[ValkyrieOrderedJointMap.LeftThumbPitch2]);
         jointToIgnoreList.add(forcedSideJointNames[ValkyrieOrderedJointMap.LeftThumbPitch3]);
      }

      if (ignoreAllNeckJoints)
      {
         for (NeckJointName neckJointName : jointMap.getNeckJointNames())
            jointToIgnoreList.add(jointMap.getNeckJointName(neckJointName));
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         if (ignoreAllArmJoints)
         {
            for (ArmJointName armJointName : jointMap.getArmJointNames())
               jointToIgnoreList.add(jointMap.getArmJointName(robotSide, armJointName));
         }
         else
         {
            jointToIgnoreList.add(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_ROLL));
            jointToIgnoreList.add(jointMap.getArmJointName(robotSide, ArmJointName.WRIST_ROLL));
            jointToIgnoreList.add(jointMap.getArmJointName(robotSide, ArmJointName.FIRST_WRIST_PITCH));
         }

         if (ignoreAllLegJoints)
         {
            for (LegJointName legJointName : jointMap.getLegJointNames())
               jointToIgnoreList.add(jointMap.getLegJointName(robotSide, legJointName));
         }
      }

      if (ignoreAllSpineJoints)
      {
         for (SpineJointName spineJointName : jointMap.getSpineJointNames())
            jointToIgnoreList.add(jointMap.getSpineJointName(spineJointName));
      }
      else
      {

      }

      jointToIgnoreList.add("hokuyo_joint");

      return jointToIgnoreList;
   }

   @Override
   public List<GroupParameter<JointDesiredBehaviorReadOnly>> getDesiredJointBehaviors()
   {
      List<GroupParameter<JointDesiredBehaviorReadOnly>> behaviors = new ArrayList<>();

      if (runningOnRealRobot)
      {
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_YAW, POSITION, 50.0, 5.0);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_ROLL, POSITION, 100.0, 10.0);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_PITCH, POSITION, 100.0, 10.0);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.KNEE_PITCH, POSITION, 100.0, 10.0);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.ANKLE_PITCH, POSITION, 30.0, 3.0);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.ANKLE_ROLL, POSITION, 30.0, 3.0);

         configureBehavior(behaviors, jointMap, SpineJointName.SPINE_YAW, POSITION, 100.0, 10.0);
         configureBehavior(behaviors, jointMap, SpineJointName.SPINE_PITCH, POSITION, 100.0, 10.0);
         configureBehavior(behaviors, jointMap, SpineJointName.SPINE_ROLL, POSITION, 100.0, 10.0);

         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SHOULDER_PITCH, POSITION, 50.0, 5.0);
         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SHOULDER_ROLL, POSITION, 50.0, 5.0);
         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SHOULDER_YAW, POSITION, 30.0, 3.0);
         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.ELBOW_PITCH, POSITION, 50.0, 5.0);

         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.ELBOW_ROLL, POSITION, 50.0, 5.0);
         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.WRIST_ROLL, POSITION, 10.0, 1.0);
         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.FIRST_WRIST_PITCH, POSITION, 10.0, 1.0);

         configureBehavior(behaviors, jointMap, NeckJointName.PROXIMAL_NECK_PITCH, POSITION, 100.0, 10.0);
         configureBehavior(behaviors, jointMap, NeckJointName.DISTAL_NECK_YAW, POSITION, 100.0, 10.0);
         configureBehavior(behaviors, jointMap, NeckJointName.DISTAL_NECK_PITCH, POSITION, 100.0, 10.0);
      }
      else
      {
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_YAW, POSITION, 300.0, 30.0);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_ROLL, POSITION, 500.0, 50.0);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_PITCH, POSITION, 550.0, 55.0);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.KNEE_PITCH, POSITION, 260.0, 26.0);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.ANKLE_PITCH, POSITION, 30.0, 3.0);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.ANKLE_ROLL, POSITION, 30.0, 3.0);

         configureBehavior(behaviors, jointMap, SpineJointName.SPINE_YAW, POSITION, 500.0, 50.0);
         configureBehavior(behaviors, jointMap, SpineJointName.SPINE_PITCH, POSITION, 800.0, 80.0);
         configureBehavior(behaviors, jointMap, SpineJointName.SPINE_ROLL, POSITION, 800.0, 80.0);

         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SHOULDER_PITCH, POSITION, 200.0, 50.0);
         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SHOULDER_ROLL, POSITION, 200.0, 50.0);
         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SHOULDER_YAW, POSITION, 100.0, 25.0);
         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.ELBOW_PITCH, POSITION, 100.0, 25.0);

         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.ELBOW_ROLL, POSITION, 30.0, 3.0);
         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.WRIST_ROLL, POSITION, 10.0, 1.0);
         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.FIRST_WRIST_PITCH, POSITION, 10.0, 1.0);

         configureBehavior(behaviors, jointMap, NeckJointName.PROXIMAL_NECK_PITCH, POSITION, 100.0, 10.0);
         configureBehavior(behaviors, jointMap, NeckJointName.DISTAL_NECK_YAW, POSITION, 100.0, 10.0);
         configureBehavior(behaviors, jointMap, NeckJointName.DISTAL_NECK_PITCH, POSITION, 100.0, 10.0);
      }

      return behaviors;
   }

   @Override
   public WholeBodySetpointParameters getDiagnosticSetpoints()
   {
      return setpoints;
   }

   @Override
   public boolean doIdleControlUntilRobotIsAlive()
   {
      return true;
   }
}
