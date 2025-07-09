package us.ihmc.openAlexander.parameters.controller;

import us.ihmc.commonWalkingControlModules.configurations.GroupParameter;
import us.ihmc.robotics.partNames.*;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.outputData.JointDesiredBehavior;
import us.ihmc.sensorProcessing.outputData.JointDesiredBehaviorReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class HighLevelParametersTools
{
   public static JointDesiredBehavior configureBehavior(List<GroupParameter<JointDesiredBehaviorReadOnly>> behaviors,
                                                        HumanoidJointNameMap jointMap,
                                                        SpineJointName jointName,
                                                        JointDesiredControlMode controlMode,
                                                        double stiffness,
                                                        double damping,
                                                        double maxPositionError,
                                                        double maxVelocityError,
                                                        double velocityScaling)
   {
      JointDesiredBehavior jointBehavior = new JointDesiredBehavior(controlMode, stiffness, damping);
      jointBehavior.setMaxPositionError(maxPositionError);
      jointBehavior.setMaxVelocityError(maxVelocityError);
      jointBehavior.setVelocityScaling(velocityScaling);
      List<String> names = Collections.singletonList(jointMap.getSpineJointName(jointName));
      behaviors.add(new GroupParameter<>(jointName.toString(), jointBehavior, names));
      return jointBehavior;
   }

   public static JointDesiredBehavior configureSymmetricBehavior(List<GroupParameter<JointDesiredBehaviorReadOnly>> behaviors,
                                                                 HumanoidJointNameMap jointMap,
                                                                 LegJointName jointName,
                                                                 JointDesiredControlMode controlMode,
                                                                 double stiffness,
                                                                 double damping,
                                                                 double maxPositionError,
                                                                 double maxVelocityError,
                                                                 double velocityScaling)
   {
      JointDesiredBehavior jointBehavior = new JointDesiredBehavior(controlMode, stiffness, damping);
      jointBehavior.setMaxPositionError(maxPositionError);
      jointBehavior.setMaxVelocityError(maxVelocityError);
      jointBehavior.setVelocityScaling(velocityScaling);
      behaviors.add(new GroupParameter<>(jointName.toString(), jointBehavior, getLeftAndRightJointNames(jointMap, jointName)));
      return jointBehavior;
   }

   public static JointDesiredBehavior configureSymmetricBehavior(List<GroupParameter<JointDesiredBehaviorReadOnly>> behaviors,
                                                                 HumanoidJointNameMap jointMap,
                                                                 ArmJointName jointName,
                                                                 JointDesiredControlMode controlMode,
                                                                 double stiffness,
                                                                 double damping,
                                                                 double maxPositionError,
                                                                 double maxVelocityError,
                                                                 double velocityScaling)
   {
      JointDesiredBehavior jointBehavior = new JointDesiredBehavior(controlMode, stiffness, damping);
      jointBehavior.setMaxPositionError(maxPositionError);
      jointBehavior.setMaxVelocityError(maxVelocityError);
      jointBehavior.setVelocityScaling(velocityScaling);
      behaviors.add(new GroupParameter<>(jointName.toString(), jointBehavior, getLeftAndRightJointNames(jointMap, jointName)));
      return jointBehavior;
   }

   public static List<String> getLeftAndRightJointNames(HumanoidJointNameMap jointMap, LegJointName legJointName)
   {
      List<String> jointNames = new ArrayList<>();
      for (RobotSide side : RobotSide.values)
      {
         jointNames.add(jointMap.getLegJointName(side, legJointName));
      }
      return jointNames;
   }

   public static List<String> getLeftAndRightJointNames(HumanoidJointNameMap jointMap, ArmJointName armJointName)
   {
      List<String> jointNames = new ArrayList<>();
      for (RobotSide side : RobotSide.values)
      {
         jointNames.add(jointMap.getArmJointName(side, armJointName));
      }
      return jointNames;
   }

   public static JointDesiredBehavior configureNeckBehavior(List<GroupParameter<JointDesiredBehaviorReadOnly>> behaviors,
                                                        HumanoidJointNameMap jointMap,
                                                        NeckJointName jointName,
                                                        JointDesiredControlMode controlMode,
                                                        double stiffness,
                                                        double damping)
   {
      JointDesiredBehavior jointBehavior = new JointDesiredBehavior(controlMode, stiffness, damping);
      List<String> names = Collections.singletonList(jointMap.getNeckJointName(jointName));
      behaviors.add(new GroupParameter<>(jointName.toString(), jointBehavior, names));
      return jointBehavior;
   }
}
