package us.ihmc.robotics.screwTheory;

import static us.ihmc.robotics.screwTheory.ScrewTools.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.apache.commons.lang3.mutable.MutableInt;

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
 * Every time the system's state is changing, the spatial acceleration calculator can be notified
 * via {@link #compute()}. Finally, the spatial acceleration of any rigid-body can be obtained as
 * follows:
 * <ul>
 * <li>{@link #getAccelerationOfBody(RigidBody, SpatialAccelerationVector)} provides the spatial
 * acceleration of any rigid-body with respect to the {@code inertialFrame}.
 * <li>{@link #getRelativeAcceleration(RigidBody, RigidBody, SpatialAccelerationVector)} provides
 * the spatial acceleration of any rigid-body with respect to another rigid-body of the same system.
 * <li>{@link #getLinearAccelerationOfBodyFixedPoint(RigidBody, FramePoint, FrameVector)} provides
 * the linear acceleration of a point of a rigid-body with respect to the {@code inertialFrame}.
 * <li>{@link #getLinearAccelerationOfBodyFixedPoint(RigidBody, RigidBody, FramePoint, FrameVector)}
 * provides the linear acceleration of a point of a rigid-body with respect to another rigid-body of
 * the same system.
 * </ul>
 * </p>
 */
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

   /**
    * Internal storage of the acceleration of each body the system. This is the map from
    * rigid-bodies to indices to use with {@code rigidBodiesWithAssignedAcceleration} and
    * {@code assignedAccelerations} for retrieving their acceleration if already computed. If no
    * acceleration has been computed yet, the index is equal to {@code -1}.
    */
   private final HashMap<RigidBody, MutableInt> rigidBodyToAssignedAccelerationIndex = new HashMap<>();
   /**
    * List of the rigid-bodies with an up-to-date acceleration. This list allows a garbage free
    * clearance of the {@code rigidBodyToAssignedAccelerationIndex} map.
    */
   private final List<RigidBody> rigidBodiesWithAssignedAcceleration;
   /**
    * The list of up-to-date accelerations assigned to rigid-bodies. The association acceleration
    * <-> rigid-body can be retrieved using the map {@code rigidBodyToAssignedAccelerationIndex}.
    */
   private final List<SpatialAccelerationVector> assignedAccelerations;
   /**
    * List of out-of-date accelerations used to recycle memory.
    */
   private final List<SpatialAccelerationVector> unnassignedAccelerations = new ArrayList<>();

   private final ReferenceFrame inertialFrame;

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
      this(body, ReferenceFrame.getWorldFrame(), createGravitationalSpatialAcceleration(ScrewTools.getRootBody(body), gravity), twistCalculator, true, useDesireds);
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
   public SpatialAccelerationCalculator(RigidBody body, ReferenceFrame inertialFrame, SpatialAccelerationVector rootAcceleration,
                                        TwistCalculator twistCalculator, boolean doVelocityTerms, boolean doAccelerationTerms, boolean useDesireds)
   {
      this.inertialFrame = inertialFrame;
      this.rootBody = ScrewTools.getRootBody(body);
      this.rootAcceleration = new SpatialAccelerationVector(rootBody.getBodyFixedFrame(), inertialFrame, rootBody.getBodyFixedFrame());
      setRootAcceleration(rootAcceleration);
      this.twistCalculator = twistCalculator;
      inertialFrame.checkReferenceFrameMatch(twistCalculator.getInertialFrame());
      this.doVelocityTerms = doVelocityTerms;
      this.doAccelerationTerms = doAccelerationTerms;
      this.useDesireds = useDesireds;

      int numberOfRigidBodies = ScrewTools.computeSubtreeSuccessors(ScrewTools.computeSubtreeJoints(rootBody)).length;
      while (unnassignedAccelerations.size() < numberOfRigidBodies)
         unnassignedAccelerations.add(new SpatialAccelerationVector());
      assignedAccelerations = new ArrayList<>(numberOfRigidBodies);
      rigidBodiesWithAssignedAcceleration = new ArrayList<>(numberOfRigidBodies);

      assignedAccelerations.add(rootAcceleration);
      rigidBodiesWithAssignedAcceleration.add(rootBody);
      rigidBodyToAssignedAccelerationIndex.put(rootBody, new MutableInt(0));
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
      ReferenceFrame rootBodyFrame = rootBody.getBodyFixedFrame();
      rootAcceleration.checkReferenceFramesMatch(rootBodyFrame, inertialFrame, rootBodyFrame);
      this.rootAcceleration.set(newRootAcceleration);
   }

   /**
    * Notifies the system has changed state (configuration, velocity, and acceleration).
    * <p>
    * This method has to be called once every time the system state has changed, and before calling
    * the methods {@link #getAccelerationOfBody(RigidBody, SpatialAccelerationVector)} and
    * {@link #getRelativeAcceleration(RigidBody, RigidBody, SpatialAccelerationVector)}.
    * </p>
    * <p>
    * WARNING: the {@link TwistCalculator} used in this has also to be notified every time the
    * system changes state.
    * </p>
    */
   // TODO rename to reset
   public void compute()
   {
      while (rigidBodiesWithAssignedAcceleration.size() > 1)
         rigidBodyToAssignedAccelerationIndex.get(rigidBodiesWithAssignedAcceleration.remove(rigidBodiesWithAssignedAcceleration.size() - 1)).setValue(-1);

      while (assignedAccelerations.size() > 1)
         unnassignedAccelerations.add(assignedAccelerations.remove(assignedAccelerations.size() - 1));

      rigidBodyToAssignedAccelerationIndex.get(rootBody).setValue(0);
   }

   /**
    * Updates if necessary and packs the acceleration of the given {@code body}.
    * <p>
    * The result is the acceleration of the {@code body.getBodyFixedFrame()}, with respect to the
    * {@code inertialFrame}, expressed in the {@code body.getBodyFixedFrame()}.
    * </p>
    * <p>
    * WARNING: This method assumes that the internal memory of this and {@code twistCalculator} is
    * up-to-date. The user has to notify this calculator every time the system state has changed,
    * this is done through the method {@link #compute()} and notify the {@link TwistCalculator} the
    * same way.
    * </p>
    * <p>
    * In the case the acceleration of the given {@code body} has been computed already no extra
    * computation is done. However, if there is no up-to-date acceleration for this rigid-body, it
    * is then updated by a recursive approach using the following relation: <br>
    * A<sup>b, b</sup><sub>i</sub> = A<sup>p, b</sup><sub>i</sub> + A<sup>b, b</sup><sub>p</sub>
    * </br>
    * where 'b' is the {@code body} frame, 'p' the predecessor frame, and 'i' the inertial frame.
    * <br>
    * Starting from the given {@code body}, its acceleration can be updated using the acceleration
    * of the predecessor to the parent joint. The acceleration of the predecessor is updated in the
    * same manner. This is done recursively until the predecessor has an up-to-date acceleration or
    * is the root body.
    * </p>
    * @param body the rigid-body to get the acceleration of.
    * @param accelerationToPack the acceleration of the {@code body} to pack. Modified.
    */
   public void getAccelerationOfBody(RigidBody body, SpatialAccelerationVector accelerationToPack)
   {
      accelerationToPack.set(computeOrGetAccelerationOfBody(body));
   }

   /**
    * Temporary twist used for intermediate garbage free operations. To use only in the method
    * {@link #getRelativeAcceleration(RigidBody, RigidBody, SpatialAccelerationVector)}.
    */
   private final Twist twistOfCurrentWithRespectToNew = new Twist();
   /**
    * Temporary twist used for intermediate garbage free operations. To use only in the method
    * {@link #getRelativeAcceleration(RigidBody, RigidBody, SpatialAccelerationVector)}.
    */
   private final Twist twistOfBodyWithRespectToBase = new Twist();
   /**
    * Temporary acceleration used for intermediate garbage free operations. To use only in the
    * method {@link #getRelativeAcceleration(RigidBody, RigidBody, SpatialAccelerationVector)}.
    */
   private final SpatialAccelerationVector baseAcceleration = new SpatialAccelerationVector();

   /**
    * Computes and packs the spatial acceleration of the {@code body} relative to the given
    * {@code base}. The resulting spatial acceleration is the acceleration of the
    * {@code body.getBodyFixedFrame()} with respect to the {@code base.getBodyFixedFrame()},
    * expressed in {@code body.getBodyFixedFrame()}.
    * <p>
    * WARNING: This method assumes that the internal memory of this
    * {@code SpatialAccelerationCalculator} and {@code twistCalculator} is up-to-date. The user has
    * to notify this calculator every time the system state has changed, this is done through the
    * method {@link #compute()}.
    * </p>
    * <p>
    * The relative acceleration between the two rigid-bodies is calculated knowing their
    * accelerations with respect to the inertial frame using the method
    * {@link #getAccelerationOfBody(RigidBody, SpatialAccelerationVector)}: <br>
    * A<sup>b2, b2</sup><sub>b1</sub> = A<sup>b2, b2</sup><sub>i</sub> - A<sup>b1,
    * b2</sup><sub>i</sub> </br>
    * with 'b1' being the {@code base}, 'b2' the {@code body}, and 'i' the {@code inertialFrame}.
    * </p>
    * @param base the rigid-body with respect to which the acceleration is to be computed.
    * @param body the rigid-body to compute the acceleration of.
    * @param accelerationToPack the acceleration of {@code body} with respect to {@code base}.
    *           Modified.
    */
   public void getRelativeAcceleration(RigidBody base, RigidBody body, SpatialAccelerationVector accelerationToPack)
   {
      twistCalculator.getRelativeTwist(body, base, twistOfCurrentWithRespectToNew);
      twistOfCurrentWithRespectToNew.changeFrame(base.getBodyFixedFrame());

      twistCalculator.getTwistOfBody(base, twistOfBodyWithRespectToBase);

      getAccelerationOfBody(base, baseAcceleration);
      getAccelerationOfBody(body, accelerationToPack);

      baseAcceleration.changeFrame(accelerationToPack.getExpressedInFrame(), twistOfCurrentWithRespectToNew, twistOfBodyWithRespectToBase);
      accelerationToPack.sub(baseAcceleration);
   }

   /**
    * Temporary acceleration used for intermediate garbage free operations. To use only in the
    * method
    * {@link #getLinearVelocityOfBodyFixedPoint(RigidBody, RigidBody, FramePoint, FrameVector)} and
    * {@link #getLinearAccelerationOfBodyFixedPoint(RigidBody, FramePoint, FrameVector)}.
    */
   private final SpatialAccelerationVector accelerationForGetLinearAccelerationOfBodyFixedPoint = new SpatialAccelerationVector();
   /**
    * Temporary point used for intermediate garbage free operations. To use only in the method
    * {@link #getLinearVelocityOfBodyFixedPoint(RigidBody, RigidBody, FramePoint, FrameVector)} and
    * {@link #getLinearAccelerationOfBodyFixedPoint(RigidBody, FramePoint, FrameVector)}.
    */
   private final FramePoint pointForGetLinearAccelerationOfBodyFixedPoint = new FramePoint();
   /**
    * Temporary twist used for intermediate garbage free operations. To use only in the method
    * {@link #getLinearVelocityOfBodyFixedPoint(RigidBody, RigidBody, FramePoint, FrameVector)} and
    * {@link #getLinearAccelerationOfBodyFixedPoint(RigidBody, FramePoint, FrameVector)}.
    */
   private final Twist twistForGetLinearAccelerationOfBodyFixedPoint = new Twist();

   /**
    * Computes and packs the linear acceleration of the point {@code bodyFixedPoint} that is
    * attached to {@code body} with respect to {@code base}.
    * <p>
    * WARNING: This method assumes that the internal memory of this {@code TwistCalculator} is
    * up-to-date. The update of the internal memory is done through the method {@link #compute()}.
    * </p>
    * <p>
    * The approach used is the same as for
    * {@link #getRelativeAcceleration(RigidBody, RigidBody, SpatialAccelerationVector)}.
    * </p>
    * @param base the rigid-body with respect to which the acceleration is to be computed.
    * @param body the rigid-body to which {@code bodyFixedPoint} belongs.
    * @param bodyFixedPoint the point to compute the linear acceleration of. Not modified.
    * @param linearAccelerationToPack the linear acceleration of the body fixed point. Modified.
    */
   public void getLinearAccelerationOfBodyFixedPoint(RigidBody base, RigidBody body, FramePoint bodyFixedPoint, FrameVector linearAccelerationToPack)
   {
      FramePoint localPoint = pointForGetLinearAccelerationOfBodyFixedPoint;
      Twist localTwist = twistForGetLinearAccelerationOfBodyFixedPoint;
      SpatialAccelerationVector localAcceleration = accelerationForGetLinearAccelerationOfBodyFixedPoint;

      getRelativeAcceleration(base, body, localAcceleration);

      ReferenceFrame baseFrame = localAcceleration.getBaseFrame();
      twistCalculator.getRelativeTwist(base, body, localTwist);
      localPoint.setIncludingFrame(bodyFixedPoint);

      /*
       * By changing the expressedInFrame from bodyFrame to baseFrame, there is no need to use the
       * more expensive changeFrame(ReferenceFrame, Twist, Twist).
       */
      localAcceleration.getExpressedInFrame().checkReferenceFrameMatch(localAcceleration.getBodyFrame());
      localAcceleration.changeFrameNoRelativeMotion(baseFrame);
      localTwist.changeFrame(baseFrame);
      localPoint.changeFrame(baseFrame);

      localAcceleration.getAccelerationOfPointFixedInBodyFrame(localTwist, localPoint, linearAccelerationToPack);
   }

   /**
    * Computes and packs the linear acceleration of the point {@code bodyFixedPoint} that is
    * attached to {@code body} with respect to {@code inertialFrame}.
    * <p>
    * Note that the root acceleration is considered in this calculation. To get the linear
    * acceleration without considering the root acceleration use
    * {@link #getLinearAccelerationOfBodyFixedPoint(RigidBody, RigidBody, FramePoint, FrameVector)}
    * providing {@link #getRootBody()} as the {@code base}.
    * </p>
    * <p>
    * WARNING: This method assumes that the internal memory of this {@code TwistCalculator} is
    * up-to-date. The update of the internal memory is done through the method {@link #compute()}.
    * </p>
    * @param body the rigid-body to which {@code bodyFixedPoint} belongs.
    * @param bodyFixedPoint the point to compute the linear acceleration of. Not modified.
    * @param linearAccelerationToPack the linear acceleration of the body fixed point. Modified.
    */
   public void getLinearAccelerationOfBodyFixedPoint(RigidBody body, FramePoint bodyFixedPoint, FrameVector linearAccelerationToPack)
   {
      FramePoint localPoint = pointForGetLinearAccelerationOfBodyFixedPoint;
      Twist localTwist = twistForGetLinearAccelerationOfBodyFixedPoint;
      SpatialAccelerationVector localAcceleration = accelerationForGetLinearAccelerationOfBodyFixedPoint;

      getAccelerationOfBody(body, localAcceleration);

      ReferenceFrame baseFrame = localAcceleration.getBaseFrame();
      twistCalculator.getTwistOfBody(body, localTwist);
      localPoint.setIncludingFrame(bodyFixedPoint);

      /*
       * By changing the expressedInFrame from bodyFrame to baseFrame, there is no need to use the
       * more expensive changeFrame(ReferenceFrame, Twist, Twist).
       */
      localAcceleration.getExpressedInFrame().checkReferenceFrameMatch(localAcceleration.getBodyFrame());
      localAcceleration.changeFrameNoRelativeMotion(baseFrame);
      localTwist.changeFrame(baseFrame);
      localPoint.changeFrame(baseFrame);

      localAcceleration.getAccelerationOfPointFixedInBodyFrame(localTwist, localPoint, linearAccelerationToPack);
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

   /**
    * Whether rigid-body accelerations resulting from centrifugal and Coriolis effects are
    * considered or ignored.
    * 
    * @return {@code true} if this spatial acceleration calculator considers the velocity terms.
    */
   public boolean areVelocitiesConsidered()
   {
      return doVelocityTerms;
   }

   /**
    * 
    */
   private final Twist twistForComputeOrGetTwistOfBody1 = new Twist();
   private final Twist twistForComputeOrGetTwistOfBody2 = new Twist();
   private final SpatialAccelerationVector accelerationForComputeOrGetAccelerationOfBody = new SpatialAccelerationVector();

   /**
    * Retrieves or computes the acceleration with respect to the inertial frame of the given
    * {@code body}.
    * <p>
    * WARNING: The returned {@code SpatialAccelerationVector} object will remain associated with the
    * given {@code body} and <b> should NOT be modified </b>.
    * </p>
    * <p>
    * In the case the acceleration of the given {@code body} has been computed already no extra
    * computation is done. However, if there is no up-to-date acceleration for this rigid-body, it
    * is then updated by a recursive approach using the following relation: <br>
    * A<sup>b, b</sup><sub>i</sub> = A<sup>p, b</sup><sub>i</sub> + A<sup>b, b</sup><sub>p</sub>
    * </br>
    * where 'b' is the {@code body} frame, 'p' the predecessor frame, and 'i' the inertial frame.
    * <br>
    * Starting from the given {@code body}, its acceleration can be updated using the acceleration
    * of the predecessor to the parent joint. The acceleration of the predecessor is updated in the
    * same manner. This is done recursively until the predecessor has an up-to-date twist or is the
    * root body.
    * </p>
    * 
    * @param body the rigid-body to get the twist of.
    * @return the twist of the rigid-body with respect to the inertial frame.
    */
   private SpatialAccelerationVector computeOrGetAccelerationOfBody(RigidBody body)
   {
      SpatialAccelerationVector acceleration = retrieveAssignedAcceleration(body);

      if (acceleration == null)
      {
         /*
          * The body twist has not been computed yet. Going up toward the root until we get the
          * twist of a rigidBody that is up-to-date, and then going back step-by-step to this body
          * while updating twists.
          */
         ReferenceFrame bodyFrame = body.getBodyFixedFrame();
         InverseDynamicsJoint parentJoint = body.getParentJoint();
         RigidBody predecessor = parentJoint.getPredecessor();
         SpatialAccelerationVector accelerationOfPredecessor = computeOrGetAccelerationOfBody(predecessor);
         acceleration = assignAndGetEmptyAcceleration(body);

         Twist localJointTwist = twistForComputeOrGetTwistOfBody1;
         Twist localPredecessorTwist = twistForComputeOrGetTwistOfBody2;
         SpatialAccelerationVector localJointAcceleration = accelerationForComputeOrGetAccelerationOfBody;

         if (doVelocityTerms)
         {
            parentJoint.getPredecessorTwist(localJointTwist);
            twistCalculator.getTwistOfBody(predecessor, localPredecessorTwist);
         }
         else
         {
            localJointTwist.setToZero(predecessor.getBodyFixedFrame(), bodyFrame, predecessor.getBodyFixedFrame());
            localPredecessorTwist.setToZero(predecessor.getBodyFixedFrame(), inertialFrame, predecessor.getBodyFixedFrame());
         }

         if (doAccelerationTerms)
         {
            if (useDesireds)
               parentJoint.getDesiredSuccessorAcceleration(localJointAcceleration);
            else
               parentJoint.getSuccessorAcceleration(localJointAcceleration);
         }
         else
         {
            localJointAcceleration.setToZero(bodyFrame, predecessor.getBodyFixedFrame(), bodyFrame);
         }

         acceleration.set(accelerationOfPredecessor);
         acceleration.changeFrame(bodyFrame, localJointTwist, localPredecessorTwist);
         acceleration.add(localJointAcceleration);
      }

      return acceleration;
   }

   /**
    * Retrieves in the internal memory, the up-to-date acceleration associated with the given
    * {@code body}.
    * 
    * @param body the query.
    * @return the up-to-date acceleration of the given {@code body}, returns {@code null} if no
    *         acceleration could be found.
    */
   private SpatialAccelerationVector retrieveAssignedAcceleration(RigidBody body)
   {
      MutableInt mutableIndex = rigidBodyToAssignedAccelerationIndex.get(body);

      if (mutableIndex == null)
      {
         mutableIndex = new MutableInt(-1);
         rigidBodyToAssignedAccelerationIndex.put(body, mutableIndex);
      }

      int assignedAccelerationIndex = mutableIndex.intValue();
      if (assignedAccelerationIndex == -1)
         return null;
      else
         return assignedAccelerations.get(assignedAccelerationIndex);
   }

   /**
    * Assigned an empty acceleration to the given {@code body} and returns it.
    * 
    * @param body the rigid-body to assign a twist to.
    * @return the twist newly assigned to the rigid-body.
    */
   private SpatialAccelerationVector assignAndGetEmptyAcceleration(RigidBody body)
   {
      SpatialAccelerationVector newAssignedAcceleration;

      if (unnassignedAccelerations.isEmpty())
         newAssignedAcceleration = new SpatialAccelerationVector();
      else
         newAssignedAcceleration = unnassignedAccelerations.remove(unnassignedAccelerations.size() - 1);

      MutableInt mutableIndex = rigidBodyToAssignedAccelerationIndex.get(body);

      if (mutableIndex == null)
      {
         mutableIndex = new MutableInt();
         rigidBodyToAssignedAccelerationIndex.put(body, mutableIndex);
      }

      mutableIndex.setValue(assignedAccelerations.size());

      rigidBodiesWithAssignedAcceleration.add(body);
      assignedAccelerations.add(newAssignedAcceleration);
      return newAssignedAcceleration;
   }
}
