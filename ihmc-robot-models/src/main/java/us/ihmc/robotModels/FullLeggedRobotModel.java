package us.ihmc.robotModels;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.robotSide.RobotSegment;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;

public interface FullLeggedRobotModel<E extends Enum<E> & RobotSegment<E>> extends FullRobotModel
{
   /**
    *  Returns the {@link ReferenceFrame} attached right after the specified leg joint.
    *  Its origin is at the joint location.
    *  When all the joints are at 0, the {@link ReferenceFrame} is oriented: x = forward, y = left, and z = up.
    *
    * @param robotSegment Refers to which leg the frame's joint belongs to (assuming there is only one left leg and one right leg).
    * @param legJointName Refers to which joint the frame is attached to.
    */
   MovingReferenceFrame getFrameAfterLegJoint(E robotSegment, LegJointName legJointName);

   /**
    * Return the {@link OneDoFJoint} describing the the corresponding leg joint.
    * @param robotSegment Refers to which leg the joint belongs to (assuming there is only one left leg and one right leg).
    * @param legJointName Refers to the joint's name.
    */
   OneDoFJoint getLegJoint(E robotSegment, LegJointName legJointName);

   /**
    * Returns the {@link RigidBody} describing the left or right foot of this robot.
    * A foot is considered as the end-effector (thus the last {@RigidBody}) of the leg to which it belongs to.
    *
    * @param robotSegment Refers to which side the foot belongs to (assuming there is only one left foot and one right foot).
    */
   RigidBody getFoot(E robotSegment);

   /**
    * This method is equivalent to:
    * <p>{@link FullLeggedRobotModel#getFoot(E)} when {@code limbName == LimbName.LEG},</p>
    * <p>{@link FullHumanoidRobotModel#getHand(RobotSide)} when {@code limbName == LimbName.ARM}.</p>
    * @param robotSegment refers to the end-effector side.
    * @param limbName refers to the limb the end-effector is attached to (either a leg or an arm).
    */
   RigidBody getEndEffector(E robotSegment, LimbName limbName);

   /**
    * This methods returns the frame located right after the parent joint of this end-effector (see {@link FullLeggedRobotModel#getEndEffector(E, LimbName)}).
    */
   MovingReferenceFrame getEndEffectorFrame(E robotSegment, LimbName limbName);

   /**
    * Returns the left or right sole reference frame.
    * A sole frame is attached to a foot and is generally used to put the foot contact points.
    * Its origin is right in the middle of the bottom of the foot.
    * @param robotSegment
    * @return
    */
   MovingReferenceFrame getSoleFrame(E robotSegment);

   SegmentDependentList<E, MovingReferenceFrame> getSoleFrames();

   E[] getRobotSegments();
}