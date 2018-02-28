package us.ihmc.robotModels;

import us.ihmc.robotics.partNames.QuadrupedJointName;
import us.ihmc.robotics.kinematics.JointLimit;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

import java.util.List;

public interface FullQuadrupedRobotModel extends FullLeggedRobotModel<RobotQuadrant>
{
   JointLimit getJointLimit(QuadrupedJointName jointName);

   QuadrupedJointName getNameForOneDoFJoint(OneDoFJoint oneDoFJoint);

   List<OneDoFJoint> getLegJointsList(RobotQuadrant robotQuadrant);
}