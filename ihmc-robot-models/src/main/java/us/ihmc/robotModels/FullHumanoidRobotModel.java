package us.ihmc.robotModels;

import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public interface FullHumanoidRobotModel extends FullLeggedRobotModel<RobotSide>
{
   /**
    * Returns the {@link RigidBodyBasics} describing the chest (or trunk) of this robot. The chest is
    * considered to be located right after the spine joints, and right before the arms and head.
    */
   RigidBodyBasics getChest();

   /**
    * Returns the {@link RigidBodyBasics} describing the pelvis of this robot. In the current framework
    * (on the day: 3/1/2018), the pelvis is the the first successor of the root joint.
    */
   default RigidBodyBasics getPelvis()
   {
      return getRootBody();
   }

   /**
    * Return the {@link OneDoFJointBasics} describing the the corresponding arm joint.
    * 
    * @param robotSide    Refers to which arm the joint belongs to (assuming there is only one left arm
    *                     and one right arm).
    * @param armJointName Refers to the joint's name.
    */
   OneDoFJointBasics getArmJoint(RobotSide robotSide, ArmJointName armJointName);

   /**
    * Returns the {@link RigidBodyBasics} describing the left or right hand of this robot. A hand is
    * most of the time considered as the end-effector of the leg to which it belongs to. However, hand
    * palm and fingers are located after the hand.
    *
    * @param robotSide Refers to which side the foot belongs to (assuming there is only one left foot
    *                  and one right foot).
    */
   RigidBodyBasics getHand(RobotSide robotSide);

   /**
    * Returns the {@link RigidBodyBasics} describing the left or right forearm of this robot.
    *
    * @param robotSide Refers to which side the forearm belongs to (assuming there is only one left forearm
    *                  and one right forearm).
    */
   RigidBodyBasics getForearm(RobotSide robotSide);

   /**
    * Returns a control frame attached to the right or left hand that the controller uses to control
    * the hand in taskspace.
    * 
    * @param robotSide
    * @return
    */
   MovingReferenceFrame getHandControlFrame(RobotSide robotSide);

   @Override
   default RobotSide[] getRobotSegments()
   {
      return RobotSide.values;
   }

   @Override
   SideDependentList<MovingReferenceFrame> getSoleFrames();
}