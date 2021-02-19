package us.ihmc.atlas.diagnostic;

import static us.ihmc.atlas.parameters.AtlasHighLevelControllerParameters.configureBehavior;
import static us.ihmc.atlas.parameters.AtlasHighLevelControllerParameters.configureSymmetricBehavior;
import static us.ihmc.sensorProcessing.outputData.JointDesiredControlMode.EFFORT;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.robotics.dataStructures.parameters.GroupParameter;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.sensorProcessing.diagnostic.DiagnosticParameters;
import us.ihmc.sensorProcessing.outputData.JointDesiredBehaviorReadOnly;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;

public class AtlasDiagnosticParameters extends DiagnosticParameters
{
   private final AtlasJointMap jointMap;
   private final String pelvisIMUName;
   private final boolean runningOnRealRobot;

   public AtlasDiagnosticParameters(DiagnosticEnvironment diagnosticEnvironment,
                                    AtlasJointMap jointMap,
                                    HumanoidRobotSensorInformation sensorInformation,
                                    boolean runningOnRealRobot)
   {
      super(diagnosticEnvironment, runningOnRealRobot);

      this.jointMap = jointMap;
      this.runningOnRealRobot = runningOnRealRobot;
      pelvisIMUName = sensorInformation.getPrimaryBodyImu();
   }

   @Override
   public String getPelvisIMUName()
   {
      return pelvisIMUName;
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
}
