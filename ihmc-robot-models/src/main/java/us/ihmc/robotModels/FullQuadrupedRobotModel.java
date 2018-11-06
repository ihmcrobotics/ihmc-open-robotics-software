package us.ihmc.robotModels;

import us.ihmc.robotics.kinematics.JointLimitData;
import us.ihmc.robotics.partNames.QuadrupedJointName;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.kinematics.JointLimit;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.List;

public interface FullQuadrupedRobotModel extends FullLeggedRobotModel<RobotQuadrant>
{
   /** Use {@link #getJointLimitData(OneDoFJoint joint)}*/
   @Deprecated
   JointLimit getJointLimit(QuadrupedJointName jointName);

   JointLimitData getJointLimitData(OneDoFJoint joint);

   QuadrupedJointName getNameForOneDoFJoint(OneDoFJoint oneDoFJoint);

   List<OneDoFJoint> getLegJointsList(RobotQuadrant robotQuadrant);

   /**
    * Returns the {@link RigidBodyBasics} describing the body of this robot.
    * In the current framework (on the day: 3/1/2014), the body is the the first successor of the root joint.
    */
   RigidBodyBasics getBody();

   default RobotQuadrant[] getRobotSegments()
   {
      return RobotQuadrant.values;
   }
}