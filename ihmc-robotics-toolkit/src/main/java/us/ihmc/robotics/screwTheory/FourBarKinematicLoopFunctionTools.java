package us.ihmc.robotics.screwTheory;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RevoluteJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RevoluteJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.kinematics.fourbar.FourBar;
import us.ihmc.robotics.kinematics.fourbar.FourBarAngle;
import us.ihmc.robotics.kinematics.fourbar.FourBarVertex;

public class FourBarKinematicLoopFunctionTools
{
   private static final String A_NAME = "A";
   private static final String B_NAME = "B";
   private static final String C_NAME = "C";
   private static final String D_NAME = "D";

   /**
    * Computes the angle ABC. The computed angle is in the range [-<i>pi</i>; <i>pi</i>].
    * <p>
    * This is equivalent to:
    *
    * <pre>
    * Vector2D BA = new Vector2D();
    * Vector2D BC = new Vector2D();
    * BA.sub(A, B);
    * BC.sub(C, B);
    * double angleABC = BA.angle(BC);
    * </pre>
    * </p>
    *
    * @param A the first point. Not modified.
    * @param B the second point at which the angle is computed. Not modified.
    * @param C the third point. Not modified.
    * @return the angle ABC in radians.
    */
   public static double angleABC(Point2DReadOnly A, Point2DReadOnly B, Point2DReadOnly C)
   {
      double BAx = A.getX() - B.getX();
      double BAy = A.getY() - B.getY();

      double BCx = C.getX() - B.getX();
      double BCy = C.getY() - B.getY();

      return EuclidGeometryTools.angleFromFirstToSecondVector2D(BAx, BAy, BCx, BCy);
   }

   /**
    * Reorders the joints such that the resulting four bar kinematics is either of these two:
    * 
    * <pre>
    *     root            root
    *      |               |
    *   A-----B         A-----B
    *   |     |          \   /
    *   |     |    or     \ /
    *   |     |            X
    *   |     |           / \
    *   |     |          /   \
    *   D-----C         C-----D
    *      |               |
    * end-effector    end-effector
    * </pre>
    * 
    * once reordered, the array <code>joints = {A, B, C, D}</code>.
    * <p>
    * In addition to reordering the joints, this method does the following:
    * <ul>
    * <li>performs sanity checks on the kinematics to ensure it represents a four bar linkage.
    * <li>initializes the converters and the four bar calculator.
    * <li>updates the master joint index so points to the same joint in the array.
    * <li>evaluates and updates if needed the limits of each joint.
    * </ul>
    * </p>
    * 
    * @param joints           the array of joints to use in the four bar linkage, expected to be 4
    *                         joints. Modified.
    * @param converters       use for converting back and forth between joint angle and interior angle
    *                         of the four bar geometry. Modified.
    * @param fourBar          the calculator for evaluating the configuration of the four bar geometry.
    *                         Modified.
    * @param masterJointIndex the index of the joint that is used to control the four bar linkage.
    * @param epsilon          tolerance used during sanity checks.
    * @return the update index to point to the master joint in the update array.
    * @throws IllegalArgumentException if the given joints cannot be used to represent a four bar
    *                                  linkage.
    */
   public static int configureFourBarKinematics(RevoluteJointBasics[] joints, FourBarToJointConverter[] converters, FourBar fourBar, int masterJointIndex,
                                                double epsilon)
   {
      if (joints.length != 4)
         throw new IllegalArgumentException("Expected 4 joints");

      RevoluteJointReadOnly loopClosureJoint = null;
      RevoluteJointReadOnly masterJoint = joints[masterJointIndex];
      RigidBodyReadOnly ancestor = findCommonClosestAncestor(joints[0], joints[1], joints[2], joints[3]);

      for (int i = 0; i < 4; i++)
      {
         joints[i].setQ(0.0);
         joints[i].getFrameAfterJoint().update();
         if (joints[i].isLoopClosure())
            loopClosureJoint = joints[i];
      }

      // Check that at zero-configuration, the four bar is properly closed.
      if (loopClosureJoint == null)
         throw new IllegalArgumentException("Could not find the loop closure joint.");
      RigidBodyTransform error = loopClosureJoint.getLoopClosureFrame()
                                                 .getTransformToDesiredFrame(loopClosureJoint.getSuccessor().getParentJoint().getFrameAfterJoint());
      if (error.getTranslation().length() > epsilon)
         throw new IllegalArgumentException("The four bar is not properly closed at zero-configuration, error:\n" + error);

      ReferenceFrame fourBarLocalFrame = GeometryTools.constructReferenceFrameFromPointAndAxis("LocalFrame",
                                                                                               new FramePoint3D(masterJoint.getFrameBeforeJoint()),
                                                                                               Axis3D.Z,
                                                                                               masterJoint.getJointAxis());

      FramePoint3D tempFramePoint3D = new FramePoint3D();
      FrameVector3D[] axes = {new FrameVector3D(), new FrameVector3D(), new FrameVector3D(), new FrameVector3D()};
      FramePoint2D[] origins = {new FramePoint2D(), new FramePoint2D(), new FramePoint2D(), new FramePoint2D()};

      for (int i = 0; i < 4; i++)
      {
         RevoluteJointBasics joint = joints[i];
         FrameVector3D axis = axes[i];
         axis.setIncludingFrame(joint.getJointAxis());
         axis.changeFrame(fourBarLocalFrame);

         tempFramePoint3D.setToZero(joint.getFrameAfterJoint());
         tempFramePoint3D.changeFrame(fourBarLocalFrame);
         origins[i].setIncludingFrame(tempFramePoint3D);
      }

      /*
       * First we reorder the joints so the layout is as follows:
       * @formatter:off
       *     root            root
       *      |               |
       *   A-----B         A-----B
       *   |     |          \   /
       *   |     |    or     \ /
       *   |     |            X
       *   |     |           / \
       *   |     |          /   \
       *   D-----C         C-----D
       *      |               |
       * end-effector    end-effector
       * @formatter:on
       */
      int jointAIndex = -1;
      int jointBIndex = -1;

      for (int i = 0; i < 4; i++)
      {
         if (joints[i].getPredecessor() == ancestor)
         {
            if (jointAIndex == -1)
               jointAIndex = i;
            else
               jointBIndex = i;
         }
      }

      int jointCIndex = -1;
      int jointDIndex = -1;

      for (int i = 0; i < 4; i++)
      {
         if (i == jointAIndex || i == jointBIndex)
            continue;

         if (joints[i].getPredecessor() == joints[jointAIndex].getSuccessor())
            jointDIndex = i;
         else
            jointCIndex = i;
      }

      // Now we check if DAB is clockwise, if not we mirror the four bar.
      if (angleABC(origins[jointDIndex], origins[jointAIndex], origins[jointBIndex]) < 0.0)
      { // Counter-clockwise, let's mirror the layout
         int temp = jointAIndex;
         jointAIndex = jointBIndex;
         jointBIndex = temp;
         temp = jointDIndex;
         jointDIndex = jointCIndex;
         jointCIndex = temp;
      }

      RevoluteJointBasics jointA = joints[jointAIndex];
      RevoluteJointBasics jointB = joints[jointBIndex];
      RevoluteJointBasics jointC = joints[jointCIndex];
      RevoluteJointBasics jointD = joints[jointDIndex];
      // Asserts that the joints represent a four bar linkage.
      checkKinematicLayout(jointA, jointB, jointC, jointD);
      joints[0] = jointA;
      joints[1] = jointB;
      joints[2] = jointC;
      joints[3] = jointD;

      FramePoint2D originA = origins[jointAIndex];
      FramePoint2D originB = origins[jointBIndex];
      FramePoint2D originC = origins[jointCIndex];
      FramePoint2D originD = origins[jointDIndex];
      FrameVector3D axisA = axes[jointAIndex];
      FrameVector3D axisB = axes[jointBIndex];
      FrameVector3D axisC = axes[jointCIndex];
      FrameVector3D axisD = axes[jointDIndex];

      if (!EuclidFrameTools.areVector3DsParallel(axisA, axisB, epsilon))
         throw new IllegalArgumentException(String.format("The axes of the joint %s and %s are not parallel.", jointA.getName(), jointB.getName()));
      if (!EuclidFrameTools.areVector3DsParallel(axisA, axisC, epsilon))
         throw new IllegalArgumentException(String.format("The axes of the joint %s and %s are not parallel.", jointA.getName(), jointC.getName()));
      if (!EuclidFrameTools.areVector3DsParallel(axisA, axisD, epsilon))
         throw new IllegalArgumentException(String.format("The axes of the joint %s and %s are not parallel.", jointA.getName(), jointD.getName()));

      converters[0].set(FourBarAngle.DAB, -Math.signum(axisA.getZ()), angleABC(originD, originA, originB));
      converters[1].set(FourBarAngle.ABC, +Math.signum(axisB.getZ()), angleABC(originA, originB, originC));
      converters[2].set(FourBarAngle.BCD, +Math.signum(axisC.getZ()), angleABC(originB, originC, originD));
      converters[3].set(FourBarAngle.CDA, -Math.signum(axisD.getZ()), angleABC(originC, originD, originA));

      fourBar.setup(originA, originB, originC, originD);

      for (FourBarAngle fourBarAngle : FourBarAngle.values)
      {
         int index = fourBarAngle.ordinal();
         FourBarVertex fourBarVertex = fourBar.getVertex(fourBarAngle);
         RevoluteJointBasics joint = joints[index];

         FourBarToJointConverter converter = converters[index];

         // Checking joint limits and correcting them if needed.
         double lowerLimit;
         double upperLimit;

         if (converter.getSign() > 0.0)
         {
            lowerLimit = converter.toJointAngle(fourBarVertex.getMinAngle());
            upperLimit = converter.toJointAngle(fourBarVertex.getMaxAngle());
         }
         else
         {
            lowerLimit = converter.toJointAngle(fourBarVertex.getMaxAngle());
            upperLimit = converter.toJointAngle(fourBarVertex.getMinAngle());
         }

         if (joint.getJointLimitLower() < lowerLimit)
         {
            LogTools.warn("Correcting {} lower limit from {} to {}", joint.getName(), joint.getJointLimitLower(), lowerLimit);
            joint.setJointLimitLower(lowerLimit);
         }

         if (joint.getJointLimitUpper() > upperLimit)
         {
            LogTools.warn("Correcting {} upper limit from {} to {}", joint.getName(), joint.getJointLimitUpper(), upperLimit);
            joint.setJointLimitUpper(upperLimit);
         }
      }

      for (int i = 0; i < 4; i++)
      {
         if (joints[i] == masterJoint)
            return i;
      }

      throw new IllegalStateException("Something went wrong: Could not retrieve the master joint.");
   }

   private static void checkKinematicLayout(RevoluteJointReadOnly jointA, RevoluteJointReadOnly jointB, RevoluteJointReadOnly jointC,
                                            RevoluteJointReadOnly jointD)
   {
      RigidBodyReadOnly ancestor = findCommonClosestAncestor(jointA, jointB, jointC, jointD);

      if (jointA.getPredecessor() == ancestor)
      {
         if (jointB.getPredecessor() == ancestor)
         {
            /*
             * @formatter:off
             *     root            root
             *      |               |
             *   A-----B         B-----A
             *   |     |         |     |
             *   |     |    or   |     |
             *   D-----C         C-----D
             *      |               |
             * end-effector    end-effector
             * @formatter:on
             * The relationship between the joints is the same when the four bar is inverted.
             */
            checkParentChildLayout(jointA, jointD, A_NAME, D_NAME);
            checkParentChildLayout(jointB, jointC, B_NAME, C_NAME);
            checkCommonSuccessorLayout(jointC, jointD, C_NAME, D_NAME);
            return;
         }
         else if (jointD.getPredecessor() == ancestor)
         {
            /*
             * @formatter:off
             *     root            root
             *      |               |
             *   D-----A         A-----D
             *   |     |         |     |
             *   |     |    or   |     |
             *   C-----B         B-----C
             *      |               |
             * end-effector    end-effector
             * @formatter:on
             * The relationship between the joints is the same when the four bar is inverted.
             */
            checkParentChildLayout(jointD, jointC, D_NAME, C_NAME);
            checkParentChildLayout(jointA, jointB, A_NAME, B_NAME);
            checkCommonSuccessorLayout(jointB, jointC, B_NAME, C_NAME);
            return;
         }
      }
      else if (jointC.getPredecessor() == ancestor)
      {
         if (jointB.getPredecessor() == ancestor)
         {
            /*
             * @formatter:off
             *     root            root
             *      |               |
             *   B-----C         C-----B
             *   |     |         |     |
             *   |     |    or   |     |
             *   A-----D         D-----A
             *      |               |
             * end-effector    end-effector
             * @formatter:on
             * The relationship between the joints is the same when the four bar is inverted.
             */
            checkParentChildLayout(jointB, jointA, B_NAME, A_NAME);
            checkParentChildLayout(jointC, jointD, C_NAME, D_NAME);
            checkCommonSuccessorLayout(jointA, jointD, A_NAME, D_NAME);
            return;
         }
         else if (jointD.getPredecessor() == ancestor)
         {
            /*
             * @formatter:off
             *     root            root
             *      |               |
             *   C-----D         D-----C
             *   |     |         |     |
             *   |     |    or   |     |
             *   B-----A         A-----B
             *      |               |
             * end-effector    end-effector
             * @formatter:on
             * The relationship between the joints is the same when the four bar is inverted.
             */
            checkParentChildLayout(jointD, jointA, D_NAME, A_NAME);
            checkParentChildLayout(jointC, jointB, C_NAME, B_NAME);
            checkCommonSuccessorLayout(jointA, jointB, A_NAME, B_NAME);
            return;
         }
      }

      throw new IllegalArgumentException(String.format("Improper kinematics for four bar linkage. Could not find any consecutive pair of joints with the same predecessor.\n\tjoints [A=%s, B=%s, C=%s, D=%s]",
                                                       jointA.getName(),
                                                       jointB.getName(),
                                                       jointC.getName(),
                                                       jointD.getName()));
   }

   private static void checkParentChildLayout(JointReadOnly parentJoint, JointReadOnly childJoint, String parentJointFourBarName, String childJointFourBarName)
   {
      if (parentJoint.getSuccessor() != childJoint.getPredecessor())
      {
         String messageFormat = "The joints %1$s & %2$s are expected to be connected:\n\tjoints [%1$s=%3$s, %2$s=%4$s]\n\tsuccessor %1$s==%5$s, predecessor %2$s=%6$s";
         throw new IllegalArgumentException(String.format(messageFormat,
                                                          parentJointFourBarName,
                                                          childJointFourBarName,
                                                          parentJoint.getName(),
                                                          childJoint.getName(),
                                                          parentJoint.getSuccessor().getName(),
                                                          childJoint.getPredecessor().getName()));
      }
   }

   private static void checkCommonSuccessorLayout(JointReadOnly joint1, JointReadOnly joint2, String joint1FourBarName, String joint2FourBarName)
   {
      if (joint1.getSuccessor() != joint1.getSuccessor())
      {
         String messageFormat = "The joints %1$s & %2$s are expected to share the same successor:\n\tjoints [%1$s=%3$s, %2$s=%4$s]\\n\\tsuccessors [%1$s=%5$s, %2$s=%6$s]";
         throw new IllegalArgumentException(String.format(messageFormat,
                                                          joint1FourBarName,
                                                          joint2FourBarName,
                                                          joint1.getName(),
                                                          joint2.getName(),
                                                          joint1.getSuccessor().getName(),
                                                          joint2.getSuccessor().getName()));
      }
   }

   /**
    * Finds the closest common ancestor to all predecessor of the given joints.
    * <p>
    * This is useful to identify which of the joints are starting the kinematics loop.
    * </p>
    * 
    * @param jointA the first joint.
    * @param jointB the second joint.
    * @param jointC the third joint.
    * @param jointD the fourth joint.
    * @return the closest common ancestor.
    */
   public static RigidBodyReadOnly findCommonClosestAncestor(JointReadOnly jointA, JointReadOnly jointB, JointReadOnly jointC, JointReadOnly jointD)
   {
      RigidBodyReadOnly ancestor = jointA.getPredecessor();

      if (!MultiBodySystemTools.isAncestor(jointB.getPredecessor(), ancestor))
         ancestor = jointB.getPredecessor();

      if (!MultiBodySystemTools.isAncestor(jointC.getPredecessor(), ancestor))
         ancestor = jointC.getPredecessor();

      if (!MultiBodySystemTools.isAncestor(jointD.getPredecessor(), ancestor))
         ancestor = jointD.getPredecessor();

      return ancestor;
   }

   static class FourBarToJointConverter
   {
      private FourBarAngle fourBarAngle;
      private double sign;
      private double interiorAngleAtZero;

      public FourBarToJointConverter()
      {
      }

      public void set(FourBarAngle fourBarAngle, double sign, double interiorAngleAtZero)
      {
         this.fourBarAngle = fourBarAngle;
         this.sign = sign;
         this.interiorAngleAtZero = interiorAngleAtZero;
      }

      public double toJointAngle(double interiorAngle)
      {
         return sign * (interiorAngle - interiorAngleAtZero);
      }

      public double toJointDerivative(double interiorAngularDerivative)
      {
         return sign * interiorAngularDerivative;
      }

      public double toFourBarInteriorAngle(double jointAngle)
      {
         return sign * jointAngle + interiorAngleAtZero;
      }

      public double toFourBarInteriorAngularDerivative(double jointDerivative)
      {
         return sign * jointDerivative;
      }

      public FourBarAngle getFourBarAngle()
      {
         return fourBarAngle;
      }

      public double getSign()
      {
         return sign;
      }

      public double getInteriorAngleAtZero()
      {
         return interiorAngleAtZero;
      }

      @Override
      public String toString()
      {
         return "Converter for vertex: " + fourBarAngle + ", sign= " + sign + ", int. angle at zero= " + interiorAngleAtZero;
      }
   }
}
