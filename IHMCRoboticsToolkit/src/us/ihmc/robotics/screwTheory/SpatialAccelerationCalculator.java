package us.ihmc.robotics.screwTheory;

import java.util.ArrayList;
import java.util.LinkedHashMap;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.ReferenceFrameMismatchException;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/**
 * This class is a tool that can be used to compute the spatial acceleration of each
 * {@code RigidBody} composing a rigid-body system.
 * <p>
 * A new spatial acceleration calculator can be constructed via the constructor
 * {@link #SpatialAccelerationCalculator(RigidBody, ReferenceFrame, SpatialAccelerationVector, TwistCalculator, boolean, boolean, boolean)}.
 * Every time the system's state is changing the spatial acceleration calculator can be updated via
 * {@link #compute()}. Finally, the spatial acceleration of any rigid-body can be obtained as
 * follows:
 * <ul>
 * <li>{@link #getAccelerationOfBody(SpatialAccelerationVector, RigidBody)} provides the spatial
 * acceleration of any rigid-body with respect to the {@code inertialFrame}.
 * <li>{@link #getRelativeAcceleration(SpatialAccelerationVector, RigidBody, RigidBody)} provides
 * the spatial acceleration of any rigid-body with respect to another rigid-body of the same system.
 * <li>{@link #getLinearAccelerationOfBodyFixedPoint(FrameVector, RigidBody, FramePoint)} provides
 * the linear acceleration of a point of a rigid-body with respect to the {@code inertialFrame}.
 * <li>{@link #getLinearAccelerationOfBodyFixedPoint(FrameVector, RigidBody, RigidBody, FramePoint)}
 * provides the linear acceleration of a point of a rigid-body with respect to another rigid-body of
 * the same system.
 * </ul>
 * </p>
 */
// FIXME There is no test for this guy!
public class SpatialAccelerationCalculator
{
   /**
    * The root body of the system for which this {@code SpatialAccelerationCalculator} is available.
    */
   private final RigidBody rootBody;
   /**
    * The spatial acceleration of the root body. Even assuming that the root is fixed in world,
    * root's acceleration is usually set to be the opposite of the gravitational acceleration, such
    * that the effect of the gravity is naturally propagated to the entire system.
    */
   private final SpatialAccelerationVector rootAcceleration;
   /**
    * A {@code TwistCalculator} is needed to account for the centrifugal and Coriolis accelerations.
    */
   private final TwistCalculator twistCalculator;
   /**
    * Whether rigid-body accelerations resulting from centrifugal and Coriolis effects are
    * considered or ignored.
    */
   private final boolean doVelocityTerms;
   /**
    * Whether rigid-body accelerations resulting from joint accelerations are considered or ignored.
    */
   private final boolean doAccelerationTerms;
   /**
    * Whether the desired or actual joint accelerations are used when computing rigid-body
    * accelerations.
    */
   private final boolean useDesireds;

   /** Internal storage of the spatial accelerations of each body of the system. */
   private final LinkedHashMap<RigidBody, SpatialAccelerationVector> accelerations = new LinkedHashMap<RigidBody, SpatialAccelerationVector>();
   /** All the joints of the system properly ordered, i.e. from root to end-effectors. */
   private final ArrayList<InverseDynamicsJoint> allJoints = new ArrayList<InverseDynamicsJoint>();
   private final Twist tempJointTwist = new Twist();
   private final Twist tempTwistFromWorld = new Twist();
   private final SpatialAccelerationVector tempJointAcceleration = new SpatialAccelerationVector();

   /**
    * Creates a new {@code SpatialAccelerationCalculator} that will compute all the spatial
    * accelerations of all the rigid-bodies of the system to which {@code body} belongs.
    * 
    * @param body a body that belongs to the system this spatial acceleration calculator will be
    *           available for.
    * @param twistCalculator a twist calculator for the same system and the same inertial frame as
    *           this calculator is used to account for the centrifugal and Coriolis effects on the
    *           rigid-body accelerations.
    * @param gravity the magnitude of the gravitational acceleration. It is positive and the gravity
    *           field is assumed to be pulling the system down (toward z negative).
    * @param useDesireds whether the desired or actual joint accelerations are used to compute the
    *           rigid-body accelerations.
    */
   public SpatialAccelerationCalculator(RigidBody body, TwistCalculator twistCalculator, double gravity, boolean useDesireds)
   {
      this(body, ReferenceFrame.getWorldFrame(), ScrewTools.createGravitationalSpatialAcceleration(body, gravity), twistCalculator, true, useDesireds);
   }

   /**
    * Creates a new {@code SpatialAccelerationCalculator} that will compute all the spatial
    * accelerations of all the rigid-bodies of the system to which {@code body} belongs.
    * 
    * @param body a body that belongs to the system this spatial acceleration calculator will be
    *           available for.
    * @param inertialFrame non-moving frame with respect to which the spatial accelerations are
    *           computed. Typically {@link ReferenceFrame#getWorldFrame()} is used here.
    * @param rootAcceleration the spatial acceleration of the root. Even though the root is assumed
    *           to be non-moving, the {@code rootAcceleration} is usually set to the opposite of the
    *           gravitational acceleration, such the effect of the gravity is naturally propagated
    *           to the entire system.
    * @param twistCalculator a twist calculator for the same system and the same inertial frame as
    *           this calculator is used to account for the centrifugal and Coriolis effects on the
    *           rigid-body accelerations.
    * @param doVelocityTerms whether the centrifugal and Coriolis are considered or ignored in the
    *           computation of the rigid-body accelerations.
    * @param useDesireds whether the desired or actual joint accelerations are used to compute the
    *           rigid-body accelerations.
    */
   public SpatialAccelerationCalculator(RigidBody body, ReferenceFrame inertialFrame, SpatialAccelerationVector rootAcceleration,
                                        TwistCalculator twistCalculator, boolean doVelocityTerms, boolean useDesireds)
   {
      this(body, inertialFrame, rootAcceleration, twistCalculator, doVelocityTerms, true, useDesireds);
   }

   /**
    * Creates a new {@code SpatialAccelerationCalculator} that will compute all the spatial
    * accelerations of all the rigid-bodies of the system to which {@code body} belongs.
    * 
    * @param body a body that belongs to the system this spatial acceleration calculator will be
    *           available for.
    * @param inertialFrame non-moving frame with respect to which the spatial accelerations are
    *           computed. Typically {@link ReferenceFrame#getWorldFrame()} is used here.
    * @param rootAcceleration the spatial acceleration of the root. Even though the root is assumed
    *           to be non-moving, the {@code rootAcceleration} is usually set to the opposite of the
    *           gravitational acceleration, such the effect of the gravity is naturally propagated
    *           to the entire system.
    * @param twistCalculator a twist calculator for the same system and the same inertial frame as
    *           this calculator is used to account for the centrifugal and Coriolis effects on the
    *           rigid-body accelerations.
    * @param doVelocityTerms whether the centrifugal and Coriolis are considered or ignored in the
    *           computation of the rigid-body accelerations.
    * @param doAccelerationTerms whether the joint accelerations are considered or ignored in the
    *           computation of the rigid-body accelerations.
    * @param useDesireds whether the desired or actual joint accelerations are used to compute the
    *           rigid-body accelerations.
    */
   /*
    * FIXME Need to add some checks to verify that the root and inertial frame of the
    * twistCalculator are as the ones used here.
    */
   public SpatialAccelerationCalculator(RigidBody body, ReferenceFrame inertialFrame, SpatialAccelerationVector rootAcceleration,
                                        TwistCalculator twistCalculator, boolean doVelocityTerms, boolean doAccelerationTerms, boolean useDesireds)
   {
      this.rootBody = addAllPrecedingJoints(body);
      this.rootAcceleration = new SpatialAccelerationVector(rootAcceleration);
      this.twistCalculator = twistCalculator;
      this.doVelocityTerms = doVelocityTerms;
      this.doAccelerationTerms = doAccelerationTerms;
      this.useDesireds = useDesireds;

      addAllSuccedingJoints(body);
      populateMapsAndLists();
   }

   /**
    * Changes the spatial acceleration of the root. Even though the root is assumed to be
    * non-moving, the {@code rootAcceleration} is usually set to the opposite of the gravitational
    * acceleration, such the effect of the gravity is naturally propagated to the entire system.
    * 
    * @param newRootAcceleration the new spatial acceleration of the root.
    * @throws ReferenceFrameMismatchException if any of the reference frames of
    *            {@code newRootAcceleration} does not match this calculator's root spatial
    *            acceleration's frames.
    */
   public void setRootAcceleration(SpatialAccelerationVector newRootAcceleration)
   {
      rootAcceleration.checkReferenceFramesMatch(newRootAcceleration.getBodyFrame(), newRootAcceleration.getBaseFrame(),
                                                 newRootAcceleration.getExpressedInFrame());
      this.rootAcceleration.set(newRootAcceleration);
   }

   private void addAllSuccedingJoints(RigidBody body)
   {
      for (InverseDynamicsJoint inverseDynamicsJoint : body.getChildrenJoints())
      {
         if (inverseDynamicsJoint.getSuccessor() != null)
         {
            allJoints.add(inverseDynamicsJoint);
            addAllSuccedingJoints(inverseDynamicsJoint.getSuccessor());
         }
      }
   }

   private RigidBody addAllPrecedingJoints(RigidBody body)
   {
      ArrayList<InverseDynamicsJoint> jointsTillRootBody = new ArrayList<InverseDynamicsJoint>();

      RigidBody currentBody = body;
      while (currentBody.getParentJoint() != null)
      {
         jointsTillRootBody.add(currentBody.getParentJoint());
         currentBody = currentBody.getParentJoint().getPredecessor();
      }

      for (int i = jointsTillRootBody.size() - 1; i >= 0; i--)
      {
         allJoints.add(jointsTillRootBody.get(i));
      }

      return currentBody;
   }

   /**
    * Updates the internal memory to the new system state (configuration, velocity, and
    * acceleration). This method has to be called once every time the system state has changed, and
    * before calling the methods
    * {@link #getAccelerationOfBody(SpatialAccelerationVector, RigidBody)},
    * {@link #getRelativeAcceleration(SpatialAccelerationVector, RigidBody, RigidBody)},
    * {@link #getLinearAccelerationOfBodyFixedPoint(FrameVector, RigidBody, FramePoint)}, or
    * {@link #getLinearAccelerationOfBodyFixedPoint(FrameVector, RigidBody, RigidBody, FramePoint)}.
    * <p>
    * Starting from the root which has a known acceleration {@code rootAcceleration}, this method
    * updates the spatial accelerations by using the relation: <br>
    * A<sup>s, s</sup><sub>i</sub> = A<sup>p, s</sup><sub>i</sub> + A<sup>s, s</sup><sub>p</sub>
    * </br>
    * where 's' is the {@code successorFrame}, 'p' the {@code predecessorFrame}, and 'i' the
    * {@code inertialFrame}.
    * </p>
    */
   public void compute()
   {
      accelerations.get(rootBody).set(rootAcceleration);

      for (int jointIndex = 0; jointIndex < allJoints.size(); jointIndex++)
      {
         InverseDynamicsJoint joint = allJoints.get(jointIndex);
         computeSuccessorAcceleration(joint);
      }
   }

   private void computeSuccessorAcceleration(InverseDynamicsJoint joint)
   {
      RigidBody predecessor = joint.getPredecessor();
      RigidBody successor = joint.getSuccessor();
      ReferenceFrame successorFrame = successor.getBodyFixedFrame();
      joint.getPredecessorTwist(tempJointTwist);
      if (!doVelocityTerms)
         tempJointTwist.setToZero();

      if (useDesireds)
         joint.getDesiredSuccessorAcceleration(tempJointAcceleration);
      else
         joint.getSuccessorAcceleration(tempJointAcceleration);
      if (!doAccelerationTerms)
         tempJointAcceleration.setToZero();

      twistCalculator.getTwistOfBody(tempTwistFromWorld, predecessor);
      if (!doVelocityTerms)
         tempTwistFromWorld.setToZero();
      SpatialAccelerationVector successorAcceleration = accelerations.get(successor);
      successorAcceleration.set(accelerations.get(predecessor));
      successorAcceleration.changeFrame(successorFrame, tempJointTwist, tempTwistFromWorld);
      successorAcceleration.add(tempJointAcceleration);
   }

   /**
    * Packs the spatial acceleration of the given {@code rigidBody}. The resulting spatial
    * acceleration is the acceleration of the {@code rigidBody.getBodyFixedFrame()} with respect to
    * {@code inertialFrame}, expressed in {@code rigidBody.getBodyFixedFrame()}.
    * 
    * @param spatialAccelerationToPack the spatial acceleration of the {@code rigidBody} to pack.
    *           Modified.
    * @param rigidBody the rigid-body to get the acceleration of.
    */
   // FIXME Change the method signature to have spatialAccelerationToPack as the last argument.
   public void getAccelerationOfBody(SpatialAccelerationVector spatialAccelerationToPack, RigidBody rigidBody)
   {
      spatialAccelerationToPack.set(accelerations.get(rigidBody));
   }

   private final Twist twistOfCurrentWithRespectToNew = new Twist();
   private final Twist twistOfBodyWithRespectToBase = new Twist();
   private final SpatialAccelerationVector baseAcceleration = new SpatialAccelerationVector();

   /**
    * Computes and packs the spatial acceleration of the {@code body} relative to the given
    * {@code base}. The resulting spatial acceleration is the acceleration of the
    * {@code body.getBodyFixedFrame()} with respect to the {@code base.getBodyFixedFrame()},
    * expressed in {@code body.getBodyFixedFrame()}.
    * <p>
    * WARNING: This method assumes that the internal memory of this
    * {@code SpatialAccelerationCalculator} is up-to-date. The update of the internal memory is done
    * through the method {@link #compute()}.
    * </p>
    * <p>
    * The relative acceleration between the two rigid-bodies is calculated knowing their
    * accelerations with respect to the inertial frame: <br>
    * A<sup>b2, b2</sup><sub>b1</sub> = A<sup>b2, b2</sup><sub>i</sub> - A<sup>b1,
    * b2</sup><sub>i</sub> </br>
    * with 'b1' being the {@code base}, 'b2' the {@code body}, and 'i' the {@code inertialFrame}.
    * </p>
    * 
    * @param spatialAccelerationToPack the acceleration of {@code body} with respect to
    *           {@code base}. Modified.
    * @param base the rigid-body with respect to which the acceleration is to be computed.
    * @param body the rigid-body to compute the acceleration of.
    */
   // FIXME Change the method signature to have spatialAccelerationToPack as the last argument.
   public void getRelativeAcceleration(SpatialAccelerationVector spatialAccelerationToPack, RigidBody base, RigidBody body)
   {
      twistCalculator.getRelativeTwist(twistOfCurrentWithRespectToNew, body, base);
      twistOfCurrentWithRespectToNew.changeFrame(base.getBodyFixedFrame());

      twistCalculator.getTwistOfBody(twistOfBodyWithRespectToBase, base);

      getAccelerationOfBody(baseAcceleration, base);
      getAccelerationOfBody(spatialAccelerationToPack, body);

      baseAcceleration.changeFrame(spatialAccelerationToPack.getExpressedInFrame(), twistOfCurrentWithRespectToNew, twistOfBodyWithRespectToBase);
      spatialAccelerationToPack.sub(baseAcceleration);
   }

   private final SpatialAccelerationVector endEffectorAcceleration = new SpatialAccelerationVector();

   /**
    * Computes and packs the linear acceleration of the point {@code bodyFixedPoint} that is
    * attached to {@code body} with respect to {@code base}.
    * <p>
    * WARNING: This method assumes that the internal memory of this {@code TwistCalculator} is
    * up-to-date. The update of the internal memory is done through the method {@link #compute()}.
    * </p>
    * <p>
    * The approach used is the same as for
    * {@link #getRelativeAcceleration(SpatialAccelerationVector, RigidBody, RigidBody)}.
    * </p>
    * 
    * @param linearAccelerationToPack the linear acceleration of the body fixed point. Modified.
    * @param base the rigid-body with respect to which the acceleration is to be computed.
    * @param body the rigid-body to which {@code bodyFixedPoint} belongs.
    * @param bodyFixedPoint the point to compute the linear acceleration of. Not modified.
    */
   // FIXME This method looks somewhat suspicious when comparing with getRelativeAcceleration. Needs to be tested!
   // FIXME Change the method signature to have linearAccelerationToPack as the last argument.
   public void getLinearAccelerationOfBodyFixedPoint(FrameVector linearAccelerationToPack, RigidBody base, RigidBody body, FramePoint bodyFixedPoint)
   {
      twistCalculator.getRelativeTwist(twistOfCurrentWithRespectToNew, base, body);
      twistCalculator.getTwistOfBody(twistOfBodyWithRespectToBase, body);

      getAccelerationOfBody(baseAcceleration, base);
      getAccelerationOfBody(endEffectorAcceleration, body);

      endEffectorAcceleration.changeFrame(baseAcceleration.getBodyFrame(), twistOfCurrentWithRespectToNew, twistOfBodyWithRespectToBase);
      endEffectorAcceleration.sub(baseAcceleration);
      bodyFixedPoint.changeFrame(endEffectorAcceleration.getExpressedInFrame());

      twistOfCurrentWithRespectToNew.changeFrame(endEffectorAcceleration.getExpressedInFrame());
      endEffectorAcceleration.getAccelerationOfPointFixedInBodyFrame(twistOfCurrentWithRespectToNew, bodyFixedPoint, linearAccelerationToPack);
   }

   /**
    * Computes and packs the linear acceleration of the point {@code bodyFixedPoint} that is
    * attached to {@code body} with respect to {@code rootBody}.
    * <p>
    * WARNING: This method assumes that the internal memory of this {@code TwistCalculator} is
    * up-to-date. The update of the internal memory is done through the method {@link #compute()}.
    * </p>
    * <p>
    * See
    * {@link #getLinearAccelerationOfBodyFixedPoint(FrameVector, RigidBody, RigidBody, FramePoint)}.
    * </p>
    * 
    * @param linearAccelerationToPack the linear acceleration of the body fixed point. Modified.
    * @param body the rigid-body to which {@code bodyFixedPoint} belongs.
    * @param bodyFixedPoint the point to compute the linear acceleration of. Not modified.
    */
   // FIXME Change the method signature to have linearAccelerationToPack as the last argument.
   public void getLinearAccelerationOfBodyFixedPoint(FrameVector linearAccelerationToPack, RigidBody body, FramePoint bodyFixedPoint)
   {
      getLinearAccelerationOfBodyFixedPoint(linearAccelerationToPack, getRootBody(), body, bodyFixedPoint);
   }

   /**
    * Returns the reference to the root body of the system for which this twist calculator is
    * available.
    * 
    * @return the root body.
    */
   public RigidBody getRootBody()
   {
      return rootBody;
   }

   private void populateMapsAndLists()
   {
      accelerations.put(rootBody, new SpatialAccelerationVector());

      for (InverseDynamicsJoint joint : allJoints)
      {
         if (joint.getSuccessor() != null)
         {
            accelerations.put(joint.getSuccessor(), new SpatialAccelerationVector());
         }
      }
   }
}
