package us.ihmc.robotics.screwTheory;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.apache.commons.lang3.mutable.MutableInt;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;

/**
 * This class is a tool that can be used to compute the twist of each {@code RigidBody} composing a
 * rigid-body system.
 * <p>
 * A new twist calculator can be constructed via the constructor
 * {@link #TwistCalculator(ReferenceFrame, RigidBodyBasics)}. Every time the system's state is changing,
 * the twist calculator can be notified via {@link #compute()}. Finally, the twist of any rigid-body
 * can be obtained as follows:
 * <ul>
 * <li>{@link #getTwistOfBody(RigidBodyBasics, Twist)} provides the twist of any rigid-body with respect
 * to the {@code inertialFrame}.
 * <li>{@link #getRelativeTwist(RigidBodyBasics, RigidBodyBasics, Twist)} provides the twist of any rigid-body
 * with respect to another rigid-body of the same system.
 * <li>{@link #getLinearVelocityOfBodyFixedPoint(RigidBodyBasics, FramePoint3D, FrameVector3D)} provides the
 * linear velocity of a point of a rigid-body with respect to the {@code inertialFrame}.
 * <li>{@link #getLinearVelocityOfBodyFixedPoint(RigidBodyBasics, RigidBodyBasics, FramePoint3D, FrameVector3D)}
 * provides the linear velocity of a point of a rigid-body with respect to another rigid-body of the
 * same system.
 * </ul>
 * </p>
 */
@Deprecated
public class TwistCalculator
{
   /**
    * Non-moving frame with respect to which the twists are computed. Typically set to
    * {@link ReferenceFrame#getWorldFrame()}.
    */
   private final ReferenceFrame inertialFrame;
   /** The root body of the system for which this {@code TwistCalculator} is available. */
   private final RigidBodyBasics rootBody;
   /** Twist of the root body. */
   private final Twist rootTwist;

   /**
    * Internal storage of the twist of each body the system. This is the map from rigid-bodies to
    * indices to use with {@code rigidBodiesWithAssignedTwist} and {@code assignedTwists} for
    * retrieving their twist if already computed. If no twist has been computed yet, the index is
    * equal to {@code -1}.
    */
   private final HashMap<RigidBodyBasics, MutableInt> rigidBodyToAssignedTwistIndex = new HashMap<>();
   /**
    * List of the rigid-bodies with an up-to-date twist. This list allows a garbage free clearance
    * of the {@code rigidBodyToAssignedTwistIndex} map.
    */
   private final List<RigidBodyBasics> rigidBodiesWithAssignedTwist;
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
   public TwistCalculator(ReferenceFrame inertialFrame, RigidBodyBasics body)
   {
      this.inertialFrame = inertialFrame;
      this.rootBody = MultiBodySystemTools.getRootBody(body);
      this.rootTwist = new Twist(rootBody.getBodyFixedFrame(), inertialFrame, rootBody.getBodyFixedFrame());

      int numberOfRigidBodies = MultiBodySystemTools.collectSubtreeSuccessors(MultiBodySystemTools.collectSubtreeJoints(rootBody)).length;
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
    * the methods {@link #getTwistOfBody(RigidBodyBasics, Twist)} and
    * {@link #getRelativeTwist(RigidBodyBasics, RigidBodyBasics, Twist)}.
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
    * Updates if necessary and packs the twist of the given {@code body}.
    * <p>
    * The result is the twist of the {@code body.getBodyFixedFrame()}, with respect to the
    * {@code inertialFrame}, expressed in the {@code body.getBodyFixedFrame()}.
    * </p>
    * <p>
    * WARNING: This method assumes that the internal memory of this {@code TwistCalculator} is
    * up-to-date. The user has to notify this calculator every time the system state has changed,
    * this is done through the method {@link #compute()}.
    * </p>
    * <p>
    * In the case the twist of the given {@code body} has been computed already no extra computation
    * is done. However, if there is no up-to-date twist for this rigid-body, it is then updated by a
    * recursive approach using the following relation: <br>
    * T<sup>b, b</sup><sub>i</sub> = T<sup>p, b</sup><sub>i</sub> + T<sup>b, b</sup><sub>p</sub>
    * </br>
    * where 'b' is the {@code body} frame, 'p' the predecessor frame, and 'i' the inertial frame.
    * <br>
    * Starting from the given {@code body}, its twist can be updated using the twist of the
    * predecessor to the parent joint. The twist of the predecessor is updated in the same manner.
    * This is done recursively until the predecessor has an up-to-date twist or is the root body.
    * </p>
    * 
    * @param body the rigid-body to get the twist of.
    * @param twistToPack the twist of the {@code body} to pack. Modified.
    */
   public void getTwistOfBody(RigidBodyBasics body, Twist twistToPack)
   {
      twistToPack.setIncludingFrame(computeOrGetTwistOfBody(body));
   }

   /**
    * Temporary twist used for intermediate garbage free operations. To use only in the method
    * {@link #getRelativeTwist(RigidBodyBasics, RigidBodyBasics, Twist)}.
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
    * up-to-date. The user has to notify this calculator every time the system state has changed,
    * this is done through the method {@link #compute()}.
    * </p>
    * <p>
    * The relative twist between the two rigid-bodies is calculated knowing their twists with
    * respect to the inertial frame using the method {@link #getTwistOfBody(RigidBodyBasics, Twist)}: <br>
    * T<sup>b2, b2</sup><sub>b1</sub> = T<sup>b2, b2</sup><sub>i</sub> - T<sup>b1,
    * b2</sup><sub>i</sub> </br>
    * with 'b1' being the {@code base}, 'b2' the {@code body}, and 'i' the {@code inertialFrame}.
    * </p>
    * 
    * @param base the rigid-body with respect to which the twist is to be computed.
    * @param body the rigid-body to compute the twist of.
    * @param twistToPack the twist of the {@code body} relative to the {@code base}. Modified.
    */
   public void getRelativeTwist(RigidBodyBasics base, RigidBodyBasics body, Twist twistToPack)
   {
      twistToPack.setIncludingFrame(computeOrGetTwistOfBody(body));
      twistForGetRelativeTwist.setIncludingFrame(computeOrGetTwistOfBody(base));
      twistForGetRelativeTwist.changeFrame(twistToPack.getReferenceFrame());
      twistToPack.sub(twistForGetRelativeTwist);
   }

   /**
    * Temporary twist used for intermediate garbage free operations. To use only in the method
    * {@link #getAngularVelocityOfBody(RigidBodyBasics, FrameVector3D)}.
    */
   private final Twist twistForGetAngularVelocityOfBody = new Twist();

   /**
    * Computes and packs the angular velocity of the given {@code body} with respect to
    * {@code inertialFrame}.
    * <p>
    * The result will be the angular velocity of {@code body.getBodyFixedFrame()} with respect to
    * {@code inertialFrame} expressed in {@code body.getBodyFixedFrame()}.
    * </p>
    * 
    * @param body the rigid-body to compute the angular velocity of.
    * @param angularVelocityToPack the angular velocity of the given {@code body}. Modified.
    */
   public void getAngularVelocityOfBody(RigidBodyBasics body, FrameVector3D angularVelocityToPack)
   {
      getTwistOfBody(body, twistForGetAngularVelocityOfBody);
      angularVelocityToPack.setIncludingFrame(twistForGetAngularVelocityOfBody.getAngularPart());
   }

   /**
    * Computes and packs the angular velocity of the given {@code body} with respect to the given
    * {@code base}.
    * <p>
    * The result will be the angular velocity of {@code body.getBodyFixedFrame()} with respect to
    * {@code base.getBodyFixedFrame()} expressed in {@code body.getBodyFixedFrame()}.
    * </p>
    * 
    * @param base the rigid-body with respect to which the angular velocity is to be computed.
    * @param body the rigid-body to compute the angular velocity of.
    * @param angularVelocityToPack the angular velocity of {@code body} relative to the
    *           {@code base}. Modified.
    */
   public void getRelativeAngularVelocity(RigidBodyBasics base, RigidBodyBasics body, FrameVector3D angularVelocityToPack)
   {
      getRelativeTwist(base, body, twistForGetAngularVelocityOfBody);
      angularVelocityToPack.setIncludingFrame(twistForGetAngularVelocityOfBody.getAngularPart());
   }

   /**
    * Temporary twist used for intermediate garbage free operations. To use only in the method
    * {@link #getLinearVelocityOfBodyFixedPoint(RigidBodyBasics, RigidBodyBasics, FramePoint3D, FrameVector3D)}.
    */
   private final Twist twistForGetLinearVelocityOfBodyFixedPoint = new Twist();
   /**
    * Temporary point used for intermediate garbage free operations. To use only in the method
    * {@link #getLinearVelocityOfBodyFixedPoint(RigidBodyBasics, RigidBodyBasics, FramePoint3D, FrameVector3D)}.
    */
   private final FramePoint3D pointForGetLinearVelocityOfBodyFixedPoint = new FramePoint3D();

   /**
    * Computes and packs the linear velocity of a point defined by {@code bodyFixedPoint} that is
    * fixed to the given {@code body}.
    * <p>
    * The result will be the linear velocity of the {@code bodyFixedPoint} with respect to the
    * {@code inertialFrame}. The vector is expressed in {@code inertialFrame}.
    * </p>
    * 
    * @param body the rigid-body to which {@code bodyFixedPoint} is attached to.
    * @param bodyFixedPoint the coordinates of the point attached to {@code body} that linear
    *           velocity is to be computed. Not modified.
    * @param linearVelocityToPack the linear velocity of the body fixed point with respect to the
    *           inertial frame. Modified.
    */
   public void getLinearVelocityOfBodyFixedPoint(RigidBodyBasics body, FramePoint3D bodyFixedPoint, FrameVector3D linearVelocityToPack)
   {
      FramePoint3D localPoint = pointForGetLinearVelocityOfBodyFixedPoint;
      Twist localTwist = twistForGetLinearVelocityOfBodyFixedPoint;

      getTwistOfBody(body, localTwist);
      ReferenceFrame baseFrame = localTwist.getBaseFrame();
      localPoint.setIncludingFrame(bodyFixedPoint);

      localTwist.changeFrame(baseFrame);
      localPoint.changeFrame(baseFrame);

      localTwist.getLinearVelocityAt(localPoint, linearVelocityToPack);
   }

   /**
    * Computes and packs the linear velocity of a point defined by {@code bodyFixedPoint} that is
    * fixed to the given {@code body} with respect to the given {@code base}.
    * <p>
    * The result will be the linear velocity of the {@code bodyFixedPoint} with respect to the
    * {@code base.getBodyFixedFrame()}. The vector is expressed in {@code base.getBodyFixedFrame()}.
    * </p>
    * 
    * @param base the rigid-body with respect to which the linear velocity is to be computed.
    * @param body the rigid-body to which {@code bodyFixedPoint} is attached to.
    * @param bodyFixedPoint the coordinates of the point attached to {@code body} that linear
    *           velocity is to be computed. Not modified.
    * @param linearVelocityToPack the linear velocity of the body fixed point with respect to the
    *           inertial frame. Modified.
    */
   public void getLinearVelocityOfBodyFixedPoint(RigidBodyBasics base, RigidBodyBasics body, FramePoint3D bodyFixedPoint, FrameVector3D linearVelocityToPack)
   {
      FramePoint3D localPoint = pointForGetLinearVelocityOfBodyFixedPoint;
      Twist localTwist = twistForGetLinearVelocityOfBodyFixedPoint;

      getRelativeTwist(base, body, localTwist);
      ReferenceFrame baseFrame = localTwist.getBaseFrame();
      localPoint.setIncludingFrame(bodyFixedPoint);

      localTwist.changeFrame(baseFrame);
      localPoint.changeFrame(baseFrame);

      localTwist.getLinearVelocityAt(localPoint, linearVelocityToPack);
   }

   /**
    * Returns the reference to the root body of the system for which this twist calculator is
    * available.
    * 
    * @return the root body.
    */
   public RigidBodyBasics getRootBody()
   {
      return rootBody;
   }

   /**
    * Gets the inertial frame: non-moving frame with respect to which the twists are computed.
    * Typically set to {@link ReferenceFrame#getWorldFrame()}.
    * 
    * @return the inertial frame used by this twist calculator.
    */
   public ReferenceFrame getInertialFrame()
   {
      return inertialFrame;
   }

   /**
    * Temporary twist used for intermediate garbage free operations. To use only in the method
    * {@link #computeOrGetTwistOfBody(RigidBodyBasics)}.
    */
   private final Twist twistForComputeOrGetTwistOfBody = new Twist();

   /**
    * Retrieves or computes the twist with respect to the inertial frame of the given {@code body}.
    * <p>
    * WARNING: The returned {@code Twist} object will remain associated with the given {@code body}
    * and <b> should NOT be modified </b>.
    * </p>
    * <p>
    * In the case the twist of the given {@code body} has been computed already no extra computation
    * is done. However, if there is no up-to-date twist for this rigid-body, it is then updated by a
    * recursive approach using the following relation: <br>
    * T<sup>b, b</sup><sub>i</sub> = T<sup>p, b</sup><sub>i</sub> + T<sup>b, b</sup><sub>p</sub>
    * </br>
    * where 'b' is the {@code body} frame, 'p' the predecessor frame, and 'i' the inertial frame.
    * <br>
    * Starting from the given {@code body}, its twist can be updated using the twist of the
    * predecessor to the parent joint. The twist of the predecessor is updated in the same manner.
    * This is done recursively until the predecessor has an up-to-date twist or is the root body.
    * </p>
    * 
    * @param body the rigid-body to get the twist of.
    * @return the twist of the rigid-body with respect to the inertial frame.
    */
   private TwistReadOnly computeOrGetTwistOfBody(RigidBodyBasics body)
   {
      Twist twist = retrieveAssignedTwist(body);

      if (twist == null)
      {
         /*
          * The body twist has not been computed yet. Going up toward the root until we get the
          * twist of a rigidBody that is up-to-date, and then going back step-by-step to this body
          * while updating twists.
          */
         ReferenceFrame bodyFrame = body.getBodyFixedFrame();
         JointBasics parentJoint = body.getParentJoint();
         RigidBodyBasics predecessor = parentJoint.getPredecessor();
         TwistReadOnly twistOfPredecessor = computeOrGetTwistOfBody(predecessor);
         twist = assignAndGetEmptyTwist(body);

         parentJoint.getSuccessorTwist(twistForComputeOrGetTwistOfBody);

         twist.setIncludingFrame(twistOfPredecessor);
         twist.changeFrame(bodyFrame);
         twist.add(twistForComputeOrGetTwistOfBody);
      }

      return twist;
   }

   /**
    * Retrieves in the internal memory, the up-to-date twist associated with the given {@code body}.
    * 
    * @param body the query.
    * @return the up-to-date twist of the given {@code body}, returns {@code null} if no twist could
    *         be found.
    */
   private Twist retrieveAssignedTwist(RigidBodyBasics body)
   {
      MutableInt mutableIndex = rigidBodyToAssignedTwistIndex.get(body);

      if (mutableIndex == null)
      {
         mutableIndex = new MutableInt(-1);
         rigidBodyToAssignedTwistIndex.put(body, mutableIndex);
      }

      int assignedTwistIndex = mutableIndex.intValue();
      if (assignedTwistIndex == -1)
         return null;
      else
         return assignedTwists.get(assignedTwistIndex);
   }

   /**
    * Assigned an empty twist to the given {@code body} and returns it.
    * 
    * @param body the rigid-body to assign a twist to.
    * @return the twist newly assigned to the rigid-body.
    */
   private Twist assignAndGetEmptyTwist(RigidBodyBasics body)
   {
      Twist newAssignedTwist;

      if (unnassignedTwists.isEmpty())
         newAssignedTwist = new Twist();
      else
         newAssignedTwist = unnassignedTwists.remove(unnassignedTwists.size() - 1);

      MutableInt mutableIndex = rigidBodyToAssignedTwistIndex.get(body);

      if (mutableIndex == null)
      {
         mutableIndex = new MutableInt();
         rigidBodyToAssignedTwistIndex.put(body, mutableIndex);
      }

      mutableIndex.setValue(assignedTwists.size());

      rigidBodiesWithAssignedTwist.add(body);
      assignedTwists.add(newAssignedTwist);
      return newAssignedTwist;
   }
}
