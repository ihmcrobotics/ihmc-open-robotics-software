package us.ihmc.robotModels;

import java.util.List;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.kinematics.JointLimit;
import us.ihmc.robotics.kinematics.JointLimitData;
import us.ihmc.robotics.partNames.QuadrupedJointName;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public interface FullQuadrupedRobotModel extends FullLeggedRobotModel<RobotQuadrant>
{
   /** Use {@link #getJointLimitData(OneDoFJointBasics joint)} */
   @Deprecated
   JointLimit getJointLimit(QuadrupedJointName jointName);

   JointLimitData getJointLimitData(OneDoFJointBasics joint);

   QuadrupedJointName getNameForOneDoFJoint(OneDoFJointBasics oneDoFJoint);

   List<OneDoFJointBasics> getLegJointsList(RobotQuadrant robotQuadrant);

   /**
    * Returns the {@link RigidBodyBasics} describing the body of this robot. In the current framework
    * (on the day: 3/1/2014), the body is the the first successor of the root joint.
    */
   RigidBodyBasics getBody();

   @Override
   default RobotQuadrant[] getRobotSegments()
   {
      return RobotQuadrant.values;
   }
}