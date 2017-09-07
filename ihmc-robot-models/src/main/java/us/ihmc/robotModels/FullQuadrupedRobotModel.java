package us.ihmc.robotModels;

import java.util.List;

import us.ihmc.robotics.partNames.QuadrupedJointName;
import us.ihmc.robotics.kinematics.JointLimit;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;

public interface FullQuadrupedRobotModel extends FullRobotModel
{

   public abstract RigidBody getFoot(RobotQuadrant robotQuadrant);

   public abstract List<OneDoFJoint> getLegOneDoFJoints(RobotQuadrant quadrant);

   public abstract OneDoFJoint getOneDoFJointBeforeFoot(RobotQuadrant quadrant);

   public abstract OneDoFJoint getOneDoFJointByName(QuadrupedJointName name);

   public abstract QuadrupedJointName getNameForOneDoFJoint(OneDoFJoint oneDoFJoint);

   public abstract JointLimit getJointLimit(QuadrupedJointName jointName);

}