package us.ihmc.SdfLoader.models;

import us.ihmc.SdfLoader.partNames.ArmJointName;
import us.ihmc.SdfLoader.partNames.FingerName;
import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.SdfLoader.partNames.LimbName;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;

public interface FullHumanoidRobotModel extends FullRobotModel
{
   /**
    *  Returns the {@link ReferenceFrame} attached right after the specified leg joint.
    *  Its origin is at the joint location.
    *  When all the joints are at 0, the {@link ReferenceFrame} is oriented: x = forward, y = left, and z = up.
    * 
    * @param robotSide Refers to which leg the frame's joint belongs to (assuming there is only one left leg and one right leg).
    * @param legJointName Refers to which joint the frame is attached to.
    */
   public abstract ReferenceFrame getFrameAfterLegJoint(RobotSide robotSide, LegJointName legJointName);

   /**
    * Return the {@link OneDoFJoint} describing the the corresponding leg joint.
    * @param robotSide Refers to which leg the joint belongs to (assuming there is only one left leg and one right leg).
    * @param legJointName Refers to the joint's name.
    */
   public abstract OneDoFJoint getLegJoint(RobotSide robotSide, LegJointName legJointName);

   /**
    * Return the {@link OneDoFJoint} describing the the corresponding arm joint.
    * @param robotSide Refers to which arm the joint belongs to (assuming there is only one left arm and one right arm).
    * @param armJointName Refers to the joint's name.
    */
   public abstract OneDoFJoint getArmJoint(RobotSide robotSide, ArmJointName armJointName);

   /**
    * Returns the {@link RigidBody} describing the left or right foot of this robot.
    * A foot is considered as the end-effector (thus the last {@RigidBody}) of the leg to which it belongs to.
    * 
    * @param robotSide Refers to which side the foot belongs to (assuming there is only one left foot and one right foot).
    */
   public abstract RigidBody getFoot(RobotSide robotSide);

   /**
    * Returns the {@link RigidBody} describing the left or right hand of this robot.
    * A hand is most of the time considered as the end-effector of the leg to which it belongs to.
    * However, hand palm and fingers are located after the hand. 
    * 
    * @param robotSide Refers to which side the foot belongs to (assuming there is only one left foot and one right foot).
    */
   public abstract RigidBody getHand(RobotSide robotSide);

   /**
    * This method is equivalent to:
    * <p>{@link FullHumanoidRobotModel#getFoot(RobotSide)} when {@code limbName == LimbName.LEG},</p>
    * <p>{@link FullHumanoidRobotModel#getHand(RobotSide)} when {@code limbName == LimbName.ARM}.</p>
    * @param robotSide refers to the end-effector side.
    * @param limbName refers to the limb the end-effector is attached to (either a leg or an arm).
    */
   public abstract RigidBody getEndEffector(RobotSide robotSide, LimbName limbName);

   /**
    * This methods returns the frame located right after the parent joint of this end-effector (see {@link FullHumanoidRobotModel#getEndEffector(RobotSide, LimbName)}).
    */
   public abstract ReferenceFrame getEndEffectorFrame(RobotSide robotSide, LimbName limbName);

   /**
    * Returns a control frame attached to the right or left hand that the controller uses to control the hand in taskspace.
    * @param robotSide
    * @return
    */
   public abstract ReferenceFrame getHandControlFrame(RobotSide robotSide);
   
   // TODO This methods is currently for the DRCDemo0 in BoxMovingHighLevelHumanoidController. We should get rid of it.
   public abstract FramePoint getStaticWristToFingerOffset(RobotSide robotSide, FingerName fingerName); // FIXME Should be moved to somewhere where current joint data may be used once we start grasping

   /**
    * Returns the left or right sole reference frame.
    * A sole frame is attached to a foot and is generally used to put the foot contact points.
    * Its origin is right in the middle of the bottom of the foot.
    * @param robotSide
    * @return
    */
   public abstract ReferenceFrame getSoleFrame(RobotSide robotSide);
   
   
}