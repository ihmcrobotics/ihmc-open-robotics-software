package us.ihmc.robotModels;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commons.FormattingTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class FullRobotModelUtils
{
   public static OneDoFJointBasics[] getAllJointsExcludingHands(FullHumanoidRobotModel model)
   {
      List<OneDoFJointBasics> joints = new ArrayList<>();
      getAllJointsExcludingHands(joints, model);
      return joints.toArray(new OneDoFJointBasics[joints.size()]);
   }

   public static void getAllJointsExcludingHands(List<OneDoFJointBasics> jointsToPack, FullHumanoidRobotModel model)
   {
      model.getOneDoFJoints(jointsToPack);
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics hand = model.getHand(robotSide);
         if (hand != null)
         {
            OneDoFJointBasics[] fingerJoints = MultiBodySystemTools.filterJoints(MultiBodySystemTools.collectSubtreeJoints(hand), OneDoFJointBasics.class);
            for (OneDoFJointBasics fingerJoint : fingerJoints)
            {
               jointsToPack.remove(fingerJoint);
            }
         }
      }
   }

   public static OneDoFJointBasics[] getArmJoints(FullHumanoidRobotModel model, RobotSide side, ArmJointName[] armJointNames)
   {
      OneDoFJointBasics[] oneDoFJoints = new OneDoFJointBasics[armJointNames.length];
      for (int i = 0; i < armJointNames.length; i++)
      {
         oneDoFJoints[i] = model.getArmJoint(side, armJointNames[i]);
      }
      return oneDoFJoints;
   }

   /**
    * Gets string of joint angles in a somewhat compact form, like:
    * <pre>
    *    0.605, 0.317, -0.495,
    *    -1.004, -0.600, 0.000,
    *    -0.005
    * </pre>
    */
   public static String getArmJointAnglesString(OneDoFJointBasics[] armOneDoFJoints)
   {
      StringBuilder jointAnglesString = new StringBuilder();
      for (int i = 0; i < armOneDoFJoints.length; i++)
      {
         double q = armOneDoFJoints[i].getQ();
         jointAnglesString.append(FormattingTools.getFormattedDecimal3D(q));
         if (i < armOneDoFJoints.length - 1)
         {
            jointAnglesString.append(",");
         }
         if ((i - 2) % 3 == 0)
         {
            jointAnglesString.append("\n");
         }
         else
         {
            jointAnglesString.append(" ");
         }

         ++i;
      }
      return jointAnglesString.toString();
   }

   public static void checkJointNameHash(int expectedJointNameHash, int actualJointNameHash)
   {
      if (expectedJointNameHash != actualJointNameHash)
      {
         throw new RuntimeException(String.format("Joint names do not match. expected: %s actual: %s", expectedJointNameHash, actualJointNameHash));
      }
   }

   public static void copy(FullHumanoidRobotModel from, FullHumanoidRobotModel to)
   {
      to.getRootJoint().setJointConfiguration(from.getRootJoint());
      for (int i = 0; i < to.getOneDoFJoints().length; i++)
      {
         to.getOneDoFJoints()[i].setJointConfiguration(from.getOneDoFJoints()[i]);
      }
      to.updateFrames();
   }

   /**
    * For IK solvers that solve for the CoM of the hand, this is used to
    * specify the desired as the hand "control" frame, which is usually the
    * center of the palm.
    */
   public static Point3D getHandCenterOfMassInControlFrame(FullHumanoidRobotModel model, RobotSide side, RigidBodyTransform handControlFrameToWristTransform)
   {
      RigidBodyBasics hand = model.getHand(side);
      MovingReferenceFrame lastWristFrameAfterJoint = hand.getParentJoint().getFrameAfterJoint();
      ReferenceFrame controlFrame
            = ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(lastWristFrameAfterJoint, handControlFrameToWristTransform);

      FramePoint3D frameHandCenterOfMass = new FramePoint3D();
      hand.getCenterOfMass(frameHandCenterOfMass);
      frameHandCenterOfMass.changeFrame(controlFrame);

      Point3D handCenterOfMass = new Point3D();
      handCenterOfMass.set(frameHandCenterOfMass);

      // Clean up after we're done. Remove the frame we added to the tree.
      controlFrame.remove();

      return handCenterOfMass;
   }
}
