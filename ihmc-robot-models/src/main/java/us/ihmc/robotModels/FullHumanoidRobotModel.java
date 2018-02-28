package us.ihmc.robotModels;

import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;

public interface FullHumanoidRobotModel extends FullLeggedRobotModel<RobotSide>
{
   /**
    * Return the {@link OneDoFJoint} describing the the corresponding arm joint.
    * @param robotSide Refers to which arm the joint belongs to (assuming there is only one left arm and one right arm).
    * @param armJointName Refers to the joint's name.
    */
   OneDoFJoint getArmJoint(RobotSide robotSide, ArmJointName armJointName);

   /**
    * Returns the {@link RigidBody} describing the left or right hand of this robot.
    * A hand is most of the time considered as the end-effector of the leg to which it belongs to.
    * However, hand palm and fingers are located after the hand.
    *
    * @param robotSide Refers to which side the foot belongs to (assuming there is only one left foot and one right foot).
    */
   RigidBody getHand(RobotSide robotSide);

   /**
    * Returns a control frame attached to the right or left hand that the controller uses to control the hand in taskspace.
    * @param robotSide
    * @return
    */
   MovingReferenceFrame getHandControlFrame(RobotSide robotSide);

   void setJointAngles(RobotSide side, LimbName limb, double[] q);
}