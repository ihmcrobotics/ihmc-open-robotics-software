package us.ihmc.robotics.screwTheory;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.nameBasedHashCode.NameBasedHashCodeHolder;
import us.ihmc.robotics.nameBasedHashCode.NameBasedHashCodeTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/**
 * {@code RigidBody} describes a link which used with {@code InverseDynamicsJoint}s describe a robot
 * that can be used with our screw theory library.
 * <p>
 * This class gathers notably the physical properties of the link and its position in the robot,
 * i.e. its parent and children joints.
 * </p>
 * <p>
 * Here are a few examples of useful screw tools:
 * <ul>
 * <li>The {@link GeometricJacobian} can compute the Jacobian matrix of a kinematic chain.
 * <li>The {@link TwistCalculator} can compute all the {@code Twist} (angular and linear velocity)
 * of all the {@code RigidBody}s of a kinematic chain/tree.
 * <li>Similar to the {@code TwistCalculator}, the {@link SpatialAccelerationCalculator} can compute
 * all the spatial accelerations of all the {@code RigidBody}s of a kinematic chain/tree.
 * <li>Based on the recursive Newton-Euler algorithm, the {@link InverseDynamicsCalculator} computes
 * the desired joint torques {@code tau} from the joint configurations {@code q}, velocities
 * {@code qd}, desired accelerations {@code qddDesired}, and the list of external {@code Wrench}es
 * applied on the system.
 * <li>Other tools such as {@link CentroidalMomentumMatrix}, {@link ConvectiveTermCalculator},
 * {@link CentroidalMomentumRateTermCalculator}, are useful tools for developing whole-body control
 * framework.
 * </ul>
 * </p>
 */
public class RigidBody implements NameBasedHashCodeHolder
{
   /** This is where the physical properties of this rigid-body are stored. */
   private final RigidBodyInertia inertia;
   /**
    * This is a reference rigidly attached this rigid-body. It's usually centered at the
    * rigid-body's center of mass.
    */
   private final ReferenceFrame bodyFixedFrame;
   /**
    * The parent joint is the joint directly connected to a rigid-body and located between the
    * rigid-body and the robot's root.
    * <p>
    * The parent joint is equal to {@code null} when this rigid-body represents the root of the
    * robot.
    * </p>
    */
   private final InverseDynamicsJoint parentJoint;
   /**
    * The children joints are all the joints that are directly connected to a rigid-body and located
    * between the rigid-body and any end-effector of the robot.
    * <p>
    * There is no children joint when this rigid-body is an end-effector.
    * </p>
    */
   private final List<InverseDynamicsJoint> childrenJoints = new ArrayList<InverseDynamicsJoint>();
   private final List<InverseDynamicsJoint> childrenJointsReadOnly = Collections.unmodifiableList(childrenJoints);
   /**
    * The name of this rigid-body. It is important that this name is unique among all the
    * rigid-bodies of a robot.
    */
   private final String name;

   /**
    * A secondary unique hash code representing this rigid-body that is computed based on
    * {@link #name} and the parent joint name if any.
    */
   private final long nameBasedHashCode;

   public RigidBody(String bodyName, ReferenceFrame parentInertialFrame)
   {
      this(bodyName, new RigidBodyTransform(), parentInertialFrame, true);
   }

   public RigidBody(String bodyName, RigidBodyTransform transformToParent, ReferenceFrame parentInertialFrame) // root body constructor
   {
      this(bodyName, transformToParent, parentInertialFrame, false);
   }

   private RigidBody(String bodyName, RigidBodyTransform transformToParent, ReferenceFrame parentInertialFrame, boolean isZUpFrame) // root body constructor
   {
      if (bodyName == null)
         throw new IllegalArgumentException("Name can not be null");

      nameBasedHashCode = NameBasedHashCodeTools.computeStringHashCode(bodyName);
      this.name = bodyName;
      this.inertia = null;
      this.bodyFixedFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent(bodyName + "Frame", parentInertialFrame, transformToParent, true, isZUpFrame);
      this.parentJoint = null;
   }

   public RigidBody(String bodyName, InverseDynamicsJoint parentJoint, Matrix3DReadOnly momentOfInertia, double mass, RigidBodyTransform inertiaPose)
   {
      if (bodyName == null)
         throw new IllegalArgumentException("Name can not be null");

      this.name = bodyName;
      this.parentJoint = parentJoint;

      nameBasedHashCode = NameBasedHashCodeTools.combineHashCodes(name, parentJoint);
      ReferenceFrame frameAfterJoint = parentJoint.getFrameAfterJoint();
      bodyFixedFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent(bodyName + "CoM", frameAfterJoint, inertiaPose);
      inertia = new RigidBodyInertia(bodyFixedFrame, momentOfInertia, mass);
      inertia.getBodyFrame().checkReferenceFrameMatch(inertia.getExpressedInFrame()); // inertia should be expressed in body frame, otherwise it will change
      parentJoint.setSuccessor(this);
   }

   public RigidBodyInertia getInertia()
   {
      return inertia;
   }

   public RigidBodyInertia getInertiaCopy()
   {
      return new RigidBodyInertia(getInertia());
   }

   public ReferenceFrame getBodyFixedFrame()
   {
      return bodyFixedFrame;
   }

   public InverseDynamicsJoint getParentJoint()
   {
      return parentJoint;
   }

   public void addChildJoint(InverseDynamicsJoint joint)
   {
      this.childrenJoints.add(joint);
   }

   public List<InverseDynamicsJoint> getChildrenJoints()
   {
      return childrenJointsReadOnly;
   }

   public boolean hasChildrenJoints()
   {
      return !childrenJoints.isEmpty();
   }

   public boolean isRootBody()
   {
      return parentJoint == null;
   }

   public String getName()
   {
      return name;
   }

   public void getCoMOffset(FramePoint comOffsetToPack)
   {
      inertia.getCenterOfMassOffset(comOffsetToPack);
   }

   public void setCoMOffset(FramePoint comOffset)
   {
      inertia.setCenterOfMassOffset(comOffset);
   }

   public void updateFramesRecursively()
   {
      this.bodyFixedFrame.update();

      //      for (InverseDynamicsJoint joint : childrenJoints)
      for (int childIndex = 0; childIndex < childrenJoints.size(); childIndex++)
      {
         childrenJoints.get(childIndex).updateFramesRecursively();
      }
   }

   @Override
   public String toString()
   {
      StringBuilder builder = new StringBuilder();
      builder.append(name);
      //      builder.append(name + "\n");
      //      builder.append("Root body: " + isRootBody() + "\n");

      //      builder.append("Children joints: ");
      //
      //      if (childrenJoints.isEmpty())
      //      {
      //         builder.append("none");
      //      }
      //      else
      //      {
      //         Iterator<InverseDynamicsJoint> iterator = childrenJoints.iterator();
      //         while (iterator.hasNext())
      //         {
      //            InverseDynamicsJoint joint = iterator.next();
      //            builder.append(joint.getName());
      //
      //            if (iterator.hasNext())
      //            {
      //               builder.append(", ");
      //            }
      //         }
      //      }

      return builder.toString();
   }

   @Override
   public long getNameBasedHashCode()
   {
      return nameBasedHashCode;
   }
}
