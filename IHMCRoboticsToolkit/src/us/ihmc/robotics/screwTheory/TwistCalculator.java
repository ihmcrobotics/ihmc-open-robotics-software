package us.ihmc.robotics.screwTheory;

import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import java.util.ArrayList;
import java.util.LinkedHashMap;

/**
 * This class is a tool that can be used to compute the twist of each
 * {@code RigidBody} composing a rigid-body system.
 * <p>
 * A new twist calculator can be constructed via the constructor {@link #TwistCalculator(ReferenceFrame, RigidBody)}.
 * Every time the system's state is changing, the twist calculator can be updated via {@link #compute()}.
 * Finally, the twist of any rigid-body can be obtained as follows:
 * <ul>
 *    <li> {@link #getTwistOfBody(Twist, RigidBody)} provides the twist of any rigid-body with respect to the {@code inertialFrame}.
 *    <li> {@link #getRelativeTwist(Twist, RigidBody, RigidBody)} provides the twist of any rigid-body with respect to another
 *     rigid-body of the same system.
 * </ul>
 * </p>
 */
/*
 * FIXME There is no test for this guy!
 * TODO The current implementation is somewhat risky as the List with all the joints has to be properly filled.
 * In addition to this potential issue, the TwistCalculator requires to recompute all the twists of all the rigid bodies
 * of the system, disregarding the actual usage of these twists.
 * 
 * A better implementation could use the method compute() to somehow mark the twists as outdated,
 * and only when the method getTwistOfRigidBody() or getRelativeTwist() the necessary twists are updated.
 */
public class TwistCalculator 
{
   /** The root body of the system for which this {@code TwistCalculator} is available. */
   private final RigidBody rootBody;
   /** Twist of the root body. */
   private final Twist rootTwist;

   /** Internal storage of the twist of each body the system. */
   private final LinkedHashMap<RigidBody, Twist> twists = new LinkedHashMap<RigidBody, Twist>();
   /** All the joints of the system properly ordered, i.e. from root to end-effectors. */
   private final ArrayList<InverseDynamicsJoint> allJoints = new ArrayList<InverseDynamicsJoint>();
   private final Twist tempTwist = new Twist();

   /**
    * Creates a new {@code TwistCalculator} that will compute all the twists of all the rigid-bodies
    * of the system to which {@code body} belongs.
    * 
    * @param inertialFrame non-moving frame with respect to which the twists are computed.
    *  Typically {@link ReferenceFrame#getWorldFrame()} is used here.
    * @param body a body that belongs to the system this twist calculator will be available for.
    */
   public TwistCalculator(ReferenceFrame inertialFrame, RigidBody body)
   {
      this.rootBody = addAllPrecedingJoints(body);
      this.rootTwist = new Twist(rootBody.getBodyFixedFrame(), inertialFrame, rootBody.getBodyFixedFrame());
      addAllSucceedingJoints(body);
      populateMapsAndLists(inertialFrame);
   }

   private void addAllSucceedingJoints(RigidBody body)
   {
      for (InverseDynamicsJoint inverseDynamicsJoint : body.getChildrenJoints())
      {
         if (inverseDynamicsJoint.getSuccessor() != null)
         {
            allJoints.add(inverseDynamicsJoint);
            addAllSucceedingJoints(inverseDynamicsJoint.getSuccessor());
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
    * Updates the internal memory to the new system state (configuration and velocity).
    * This method has to be called once every time the system state has changed, and 
    * before calling the methods {@link #getTwistOfBody(Twist, RigidBody)} and
    * {@link #getRelativeTwist(Twist, RigidBody, RigidBody)}.
    * <p>
    * Starting from the root which has a constant twist {@link #rootTwist}, this
    * method updates the twists by using the relation:
    * <br> T<sup>s, s</sup><sub>i</sub> = T<sup>p, s</sup><sub>i</sub> + T<sup>s, s</sup><sub>p</sub> </br>
    * where 's' is the {@code successorFrame}, 'p' the {@code predecessorFrame}, and
    * 'i' the {@code inertialFrame}.
    * </p>
    */
   public void compute()
   {
      twists.get(rootBody).set(rootTwist);

      for (int jointIndex = 0; jointIndex < allJoints.size(); jointIndex++)
      {
         InverseDynamicsJoint joint = allJoints.get(jointIndex);
         computeSuccessorTwist(joint);
      }
   }

   private void computeSuccessorTwist(InverseDynamicsJoint joint)
   {
      RigidBody predecessor = joint.getPredecessor();
      RigidBody successor = joint.getSuccessor();
      ReferenceFrame successorFrame = successor.getBodyFixedFrame();

      joint.getSuccessorTwist(tempTwist);

      Twist successorTwist = twists.get(successor);
      successorTwist.set(twists.get(predecessor));
      successorTwist.changeFrame(successorFrame);
      successorTwist.add(tempTwist);
   }

   /**
    * Packs the twist of the given {@code rigidBody}.
    * The resulting twist is the twist of the {@code rigidBody.getBodyFixedFrame()}, with
    * respect to the {@code inertialFrame}, expressed in the {@code rigidBody.getBodyFixedFrame()}.
    * <p>
    * WARNING: This method assumes that the internal memory of this {@code TwistCalculator}
    * is up-to-date.
    * The update of the internal memory is done through the method {@link #compute()}.
    * </p>
    * 
    * @param twistToPack the twist of the {@code rigidBody} to pack. Modified.
    * @param rigidBody the rigid-body to get the twist of.
    */
   // FIXME Change the method signature to have twistToPack as the last argument.
   public void getTwistOfBody(Twist twistToPack, RigidBody rigidBody)
   {
      twistToPack.set(twists.get(rigidBody));
   }

   /**
    * Computes and packs the twist of the given {@code body} relative to the
    * given {@code base}.
    * The resulting twist is the twist of the {@code body.getBodyFixedFrame()}, with respect to 
    * {@code base.getBodyFixedFrame()}, expressed in {@code body.getBodyFixedFrame()}.
    * <p>
    * WARNING: This method assumes that the internal memory of this {@code TwistCalculator}
    * is up-to-date.
    * The update of the internal memory is done through the method {@link #compute()}.
    * </p>
    * <p>
    * The relative twist between the two rigid-bodies is calculated knowing
    * their twists with respect to the inertial frame:
    * <br> T<sup>b2, b2</sup><sub>b1</sub> = T<sup>b2, b2</sup><sub>i</sub> - T<sup>b1, b2</sup><sub>i</sub> </br>
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
      twistToPack.set(twists.get(body));
      tempTwist.set(twists.get(base));
      tempTwist.changeFrame(twistToPack.getExpressedInFrame());
      twistToPack.sub(tempTwist);
   }

   /**
    * Returns the reference to the root body of the system
    * for which this twist calculator is available.
    * 
    * @return the root body.
    */
   public RigidBody getRootBody()
   {
      return rootBody;
   }

   private void populateMapsAndLists(ReferenceFrame inertialFrame)
   {
      addTwist(inertialFrame, rootBody);
      
      for (InverseDynamicsJoint joint : allJoints)
      {
         if (joint.getSuccessor() != null)
         {
            addTwist(inertialFrame, joint.getSuccessor());
         }
      }
   }
   
   private void addTwist(ReferenceFrame inertialFrame, RigidBody body)
   {
   	ReferenceFrame bodyFixedFrame = body.getBodyFixedFrame();
      twists.put(body, new Twist(bodyFixedFrame, inertialFrame, bodyFixedFrame));
   }
}
