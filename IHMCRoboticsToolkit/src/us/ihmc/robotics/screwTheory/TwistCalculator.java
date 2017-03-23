package us.ihmc.robotics.screwTheory;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.apache.commons.lang3.mutable.MutableInt;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/**
 * This class is a tool that can be used to compute the twist of each {@code RigidBody} composing a
 * rigid-body system.
 * <p>
 * A new twist calculator can be constructed via the constructor
 * {@link #TwistCalculator(ReferenceFrame, RigidBody)}. Every time the system's state is changing,
 * the twist calculator can be notified via {@link #compute()}. Finally, the twist of any rigid-body
 * can be obtained as follows:
 * <ul>
 * <li>{@link #getTwistOfBody(Twist, RigidBody)} provides the twist of any rigid-body with respect
 * to the {@code inertialFrame}.
 * <li>{@link #getRelativeTwist(Twist, RigidBody, RigidBody)} provides the twist of any rigid-body
 * with respect to another rigid-body of the same system.
 * </ul>
 * </p>
 */
public class TwistCalculator
{
   /** The root body of the system for which this {@code TwistCalculator} is available. */
   private final RigidBody rootBody;
   /** Twist of the root body. */
   private final Twist rootTwist;

   /**
    * Internal storage of the twist of each body the system. This is the map from rigid-bodies to
    * indices to use with {@code rigidBodiesWithAssignedTwist} and {@code assignedTwists} for
    * retrieving their twist if already computed. If no twist has been computed yet, the index is
    * equal to {@code -1}.
    */
   private final HashMap<RigidBody, MutableInt> rigidBodyToAssignedTwistIndex = new HashMap<>();
   /**
    * List of the rigid-bodies with an up-to-date. This list allows a garbage free clearance of the
    * {@code rigidBodyToAssignedTwistIndex} map.
    */
   private final List<RigidBody> rigidBodiesWithAssignedTwist;
   /**
    * The list of up-to-date twists assigned to rigid-bodies. The association twist <-> rigid-body
    * can be retrieved using the map {@code rigidBodyToAssignedTwistIndex}.
    */
   private final List<Twist> assignedTwists;
   /**
    * List of out-of-date twists used to recycle memory.
    */
   private final List<Twist> unnassignedTwists = new ArrayList<>();

   /**
    * Creates a new {@code TwistCalculator} that will compute all the twists of all the rigid-bodies
    * of the system to which {@code body} belongs.
    * 
    * @param inertialFrame non-moving frame with respect to which the twists are computed. Typically
    *           {@link ReferenceFrame#getWorldFrame()} is used here.
    * @param body a body that belongs to the system this twist calculator will be available for.
    */
   public TwistCalculator(ReferenceFrame inertialFrame, RigidBody body)
   {
      this.rootBody = ScrewTools.getRootBody(body);
      this.rootTwist = new Twist(rootBody.getBodyFixedFrame(), inertialFrame, rootBody.getBodyFixedFrame());

      int numberOfRigidBodies = ScrewTools.computeSubtreeSuccessors(ScrewTools.computeSubtreeJoints(rootBody)).length;
      while (unnassignedTwists.size() < numberOfRigidBodies)
         unnassignedTwists.add(new Twist());
      assignedTwists = new ArrayList<>(numberOfRigidBodies);
      rigidBodiesWithAssignedTwist = new ArrayList<>(numberOfRigidBodies);

      assignedTwists.add(rootTwist);
      rigidBodiesWithAssignedTwist.add(rootBody);
      rigidBodyToAssignedTwistIndex.put(rootBody, new MutableInt(0));
   }

   /**
    * Notifies the system has changed state (configuration and velocity).
    * <p>
    * This method has to be called once every time the system state has changed, and before calling
    * the methods {@link #getTwistOfBody(Twist, RigidBody)} and
    * {@link #getRelativeTwist(Twist, RigidBody, RigidBody)}.
    * </p>
    */
   // TODO rename to reset
   public void compute()
   {
      while (rigidBodiesWithAssignedTwist.size() > 1)
         rigidBodyToAssignedTwistIndex.get(rigidBodiesWithAssignedTwist.remove(rigidBodiesWithAssignedTwist.size() - 1)).setValue(-1);

      while (assignedTwists.size() > 1)
         unnassignedTwists.add(assignedTwists.remove(assignedTwists.size() - 1));

      rigidBodyToAssignedTwistIndex.get(rootBody).setValue(0);
   }

   /**
    * Updates if necessary and packs the twist of the given {@code rigidBody}.
    * <p>
    * The resulting twist is the twist of the {@code rigidBody.getBodyFixedFrame()}, with respect to
    * the {@code inertialFrame}, expressed in the {@code rigidBody.getBodyFixedFrame()}.
    * </p>
    * <p>
    * WARNING: This method assumes that the internal memory of this {@code TwistCalculator} is
    * up-to-date. The user has to notify this calculator every time the system state has changed,
    * this is done through the method {@link #compute()}.
    * </p>
    * <p>
    * In the case the twist of the given {@code rigidBody} has been computed already no extra
    * computation is done. However, if there is no up-to-date twist for this rigid-body, it is then
    * updated by a recursive approach using the following relation: <br>
    * T<sup>r, r</sup><sub>i</sub> = T<sup>p, r</sup><sub>i</sub> + T<sup>r, r</sup><sub>p</sub>
    * </br>
    * where 'r' is the {@code rigidBody} frame, 'p' the predecessor frame, and 'i' the inertial
    * frame. <br>
    * Starting from the given {@code rigidBody}, its twist can be updated using the twist of the
    * predecessor to the parent joint. The twist of the predecessor is updated in the same manner.
    * This is done recursively until the predecessor has an up-to-date twist or is the root body.
    * </p>
    * 
    * @param twistToPack the twist of the {@code rigidBody} to pack. Modified.
    * @param rigidBody the rigid-body to get the twist of.
    */
   // FIXME Change the method signature to have twistToPack as the last argument.
   public void getTwistOfBody(Twist twistToPack, RigidBody rigidBody)
   {
      twistToPack.set(computeOrGetTwistOfBody(rigidBody));
   }

   /**
    * Temporary twist used for intermediate garbage free operations. To use only in the method
    * {@link #getRelativeTwist(Twist, RigidBody, RigidBody)}.
    */
   private final Twist twistForGetRelativeTwist = new Twist();

   /**
    * Computes and packs the twist of the given {@code body} relative to the given {@code base}.
    * <p>
    * The resulting twist is the twist of the {@code body.getBodyFixedFrame()}, with respect to
    * {@code base.getBodyFixedFrame()}, expressed in {@code body.getBodyFixedFrame()}.
    * </p>
    * <p>
    * WARNING: This method assumes that the internal memory of this {@code TwistCalculator} is
    * up-to-date. The update of the internal memory is done through the method {@link #compute()}.
    * </p>
    * <p>
    * The relative twist between the two rigid-bodies is calculated knowing their twists with
    * respect to the inertial frame using the method {@link #getTwistOfBody(Twist, RigidBody)}: <br>
    * T<sup>b2, b2</sup><sub>b1</sub> = T<sup>b2, b2</sup><sub>i</sub> - T<sup>b1,
    * b2</sup><sub>i</sub> </br>
    * with 'b1' being the {@code base}, 'b2' the {@code body}, and 'i' the {@code inertialFrame}.
    * </p>
    * 
    * @param twistToPack the twist of the {@code body} relative to the {@code base}. Modified.
    * @param base the rigid-body with respect to which the twist is to be computed.
    * @param body the rigid-body to compute the twist of.
    */
   // FIXME Change the method signature to have twistToPack as the last argument.
   public void getRelativeTwist(Twist twistToPack, RigidBody base, RigidBody body)
   {
      twistToPack.set(computeOrGetTwistOfBody(body));
      twistForGetRelativeTwist.set(computeOrGetTwistOfBody(base));
      twistForGetRelativeTwist.changeFrame(twistToPack.getExpressedInFrame());
      twistToPack.sub(twistForGetRelativeTwist);
   }

   /**
    * Temporary twist used for intermediate garbage free operations. To use only in the method
    * {@link #getLinearVelocityOfBodyFixedPoint(FrameVector, RigidBody, RigidBody, FramePoint)}.
    */
   private final Twist twistForGetLinearVelocityOfBodyFixedPoint = new Twist();
   /**
    * Temporary point used for intermediate garbage free operations. To use only in the method
    * {@link #getLinearVelocityOfBodyFixedPoint(FrameVector, RigidBody, RigidBody, FramePoint)}.
    */
   private final FramePoint pointForGetLinearVelocityOfBodyFixedPoint = new FramePoint();

   /**
    * Computes and packs the linear velocity of a point defined by {@code bodyFixedPoint} that is
    * fixed to the given {@code body}.
    * <p>
    * The result will be the linear velocity of the {@code bodyFixedPoint} with respect to the
    * {@code inertialFrame}. The vector is expressed in {@code inertialFrame}.
    * </p>
    * 
    * @param linearVelocityToPack the linear velocity of the body fixed point with respect to the
    *           inertial frame. Modified.
    * @param body the rigid-body to which {@code bodyFixedPoint} is attached to.
    * @param bodyFixedPoint the coordinates of the point attached to {@code body} that linear
    *           velocity is to be computed. Not modified.
    */
   public void getLinearVelocityOfBodyFixedPoint(FrameVector linearVelocityToPack, RigidBody body, FramePoint bodyFixedPoint)
   {
      FramePoint localPoint = pointForGetLinearVelocityOfBodyFixedPoint;
      Twist localTwist = twistForGetLinearVelocityOfBodyFixedPoint;

      getTwistOfBody(localTwist, body);
      ReferenceFrame baseFrame = localTwist.getBaseFrame();
      localPoint.setIncludingFrame(bodyFixedPoint);

      localTwist.changeFrame(baseFrame);
      localPoint.changeFrame(baseFrame);

      localTwist.getLinearVelocityOfPointFixedInBodyFrame(linearVelocityToPack, localPoint);
   }

   /**
    * Computes and packs the linear velocity of a point defined by {@code bodyFixedPoint} that is
    * fixed to the given {@code body} with respect to the given {@code base}.
    * <p>
    * The result will be the linear velocity of the {@code bodyFixedPoint} with respect to the
    * {@code base.getBodyFixedFrame()}. The vector is expressed in {@code base.getBodyFixedFrame()}.
    * </p>
    * 
    * @param linearVelocityToPack the linear velocity of the body fixed point with respect to the
    *           inertial frame. Modified.
    * @param base the rigid-body with respect to which the linear velocity is to be computed.
    * @param body the rigid-body to which {@code bodyFixedPoint} is attached to.
    * @param bodyFixedPoint the coordinates of the point attached to {@code body} that linear
    *           velocity is to be computed. Not modified.
    */
   public void getLinearVelocityOfBodyFixedPoint(FrameVector linearVelocityToPack, RigidBody base, RigidBody body, FramePoint bodyFixedPoint)
   {
      FramePoint localPoint = pointForGetLinearVelocityOfBodyFixedPoint;
      Twist localTwist = twistForGetLinearVelocityOfBodyFixedPoint;

      getRelativeTwist(localTwist, base, body);
      ReferenceFrame baseFrame = localTwist.getBaseFrame();
      localPoint.setIncludingFrame(bodyFixedPoint);

      localTwist.changeFrame(baseFrame);
      localPoint.changeFrame(baseFrame);

      localTwist.getLinearVelocityOfPointFixedInBodyFrame(linearVelocityToPack, localPoint);
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
    * Temporary twist used for intermediate garbage free operations. To use only in the method
    * {@link #computeOrGetTwistOfBody(RigidBody)}.
    */
   private final Twist twistForComputeOrGetTwistOfBody = new Twist();

   /**
    * Retrieves or computes the twist with respect to the inertial frame of the given
    * {@code rigidBody}.
    * <p>
    * WARNING: The returned {@code Twist} object will remain associated with the given
    * {@code rigidBody} and <b> should NOT be modified </b>.
    * </p>
    * <p>
    * In the case the twist of the given {@code rigidBody} has been computed already no extra
    * computation is done. However, if there is no up-to-date twist for this rigid-body, it is then
    * updated by a recursive approach using the following relation: <br>
    * T<sup>r, r</sup><sub>i</sub> = T<sup>p, r</sup><sub>i</sub> + T<sup>r, r</sup><sub>p</sub>
    * </br>
    * where 'r' is the {@code rigidBody} frame, 'p' the predecessor frame, and 'i' the inertial
    * frame. <br>
    * Starting from the given {@code rigidBody}, its twist can be updated using the twist of the
    * predecessor to the parent joint. The twist of the predecessor is updated in the same manner.
    * This is done recursively until the predecessor has an up-to-date twist or is the root body.
    * </p>
    * 
    * @param rigidBody the rigid-body to get the twist of.
    * @return the twist of the rigid-body with respect to the inertial frame.
    */
   private Twist computeOrGetTwistOfBody(RigidBody rigidBody)
   {
      Twist twist = retrieveAssignedTwist(rigidBody);

      if (twist == null)
      {
         /*
          * The body twist has not been computed yet. Going up toward the root until we get the
          * twist of a rigidBody that is up-to-date, and then going back step-by-step to this body
          * while updating twists.
          */
         ReferenceFrame bodyFrame = rigidBody.getBodyFixedFrame();
         InverseDynamicsJoint parentJoint = rigidBody.getParentJoint();
         RigidBody predecessor = parentJoint.getPredecessor();
         Twist twistOfPredecessor = computeOrGetTwistOfBody(predecessor);
         twist = assignAndGetEmptyTwist(rigidBody);

         parentJoint.getSuccessorTwist(twistForComputeOrGetTwistOfBody);

         twist.set(twistOfPredecessor);
         twist.changeFrame(bodyFrame);
         twist.add(twistForComputeOrGetTwistOfBody);
      }

      return twist;
   }

   /**
    * Retrieves in the internal memory, the up-to-date twist associated with the given
    * {@code rigidBody}.
    * 
    * @param rigidBody the query.
    * @return the up-to-date twist of the given {@code rigidBody}, returns {@code null} if no twist
    *         could be found.
    */
   private Twist retrieveAssignedTwist(RigidBody rigidBody)
   {
      MutableInt mutableIndex = rigidBodyToAssignedTwistIndex.get(rigidBody);

      if (mutableIndex == null)
      {
         mutableIndex = new MutableInt(-1);
         rigidBodyToAssignedTwistIndex.put(rigidBody, mutableIndex);
      }

      int assignedTwistIndex = mutableIndex.intValue();
      if (assignedTwistIndex == -1)
         return null;
      else
         return assignedTwists.get(assignedTwistIndex);
   }

   /**
    * Assigned an empty twist to the given {@code rigidBody} and returns it.
    * 
    * @param rigidBody the rigid-body to assign a twist to.
    * @return the twist newly assigned to the rigid-body.
    */
   private Twist assignAndGetEmptyTwist(RigidBody rigidBody)
   {
      Twist newAssignedTwist;

      if (unnassignedTwists.isEmpty())
         newAssignedTwist = new Twist();
      else
         newAssignedTwist = unnassignedTwists.remove(unnassignedTwists.size() - 1);

      MutableInt mutableIndex = rigidBodyToAssignedTwistIndex.get(rigidBody);

      if (mutableIndex == null)
      {
         mutableIndex = new MutableInt();
         rigidBodyToAssignedTwistIndex.put(rigidBody, mutableIndex);
      }

      mutableIndex.setValue(assignedTwists.size());

      rigidBodiesWithAssignedTwist.add(rigidBody);
      assignedTwists.add(newAssignedTwist);
      return newAssignedTwist;
   }
}
