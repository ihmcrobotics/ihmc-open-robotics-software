package us.ihmc.avatar.factory;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.robot.ExternalWrenchPointDefinition;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimRigidBodyBasics;
import us.ihmc.scs2.simulation.robot.trackers.ExternalWrenchPoint;
import us.ihmc.yoVariables.euclid.YoVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * This class implements a constraint for enforcing the closure of a kinematic loop.
 * <p>
 * This implementation uses a simple PD-control scheme to correct for violation of the loop closure.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class LoopClosureSoftConstraintSCS2 implements Controller
{
   private String name;

   private YoVector3D proportionalGains;
   private YoVector3D derivativeGains;
   private final ExternalWrenchPointDefinition constraintDefinitionA, constraintDefinitionB;
   private ExternalWrenchPoint constraintA;
   private ExternalWrenchPoint constraintB;
   private YoDouble positionErrorMagnitude;
   private YoDouble rotationErrorMagnitude;
   private YoVector3D positionError;
   private YoVector3D rotationError;
   private YoVector3D linearVelocityError;
   private YoVector3D angularVelocityError;

   private YoVector3D feedForwardForce;
   private YoVector3D feedForwardMoment;

   private final Vector3D offsetFromParentJoint = new Vector3D();
   private final Vector3D offsetFromLinkParentJoint = new Vector3D();
   private final Matrix3D constraintForceSubSpace = new Matrix3D();
   private final Matrix3D constraintMomentSubSpace = new Matrix3D();
   private final Vector3D proportionalTermLinear = new Vector3D();
   private final Vector3D derivativeTermLinear = new Vector3D();
   private final Vector3D proportionalTermAngular = new Vector3D();
   private final Vector3D derivativeTermAngular = new Vector3D();

   private SimJointBasics parentJoint;

   private final Quaternion quaternionDifference = new Quaternion();
   private final Vector3D force = new Vector3D();
   private final Vector3D moment = new Vector3D();

   /**
    * Creates a new constraint for closing a kinematic loop.
    * <p>
    * This constraint is general and the sub-space in which it operates has to be provided. Here's a
    * couple examples:
    * <ul>
    * <li>For a constraint that only allows rotation around the z-axis, the matrices defining the
    * sub-space should be as follows:
    *
    * <pre>
    *                           / 1 0 0 \
    * constraintForceSubSpace = | 0 1 0 |
    *                           \ 0 0 1 /
    *                            / 1 0 0 \
    * constraintMomentSubSpace = | 0 1 0 |
    *                            \ 0 0 0 /
    * </pre>
    *
    * Note that by having <tt>constraintForceSubSpace</tt> be identity, the entire linear space is
    * constrained, while by having the last row of <tt>constraintMomentSubSpace</tt> be only zeros, the
    * z-axis is not constrained.
    * <li>For a constraint that only allows translation along the y-axis, the matrices defining the
    * sub-space should be as follows:
    *
    * <pre>
    *                           / 1 0 0 \
    * constraintForceSubSpace = | 0 0 0 |
    *                           \ 0 0 1 /
    *                            / 1 0 0 \
    * constraintMomentSubSpace = | 0 1 0 |
    *                            \ 0 0 1 /
    * </pre>
    * </ul>
    * In other words, these matrices can be seen as selection matrices used for selecting which forces
    * and moments are to be applied.
    * </p>
    *
    * @param name                      the name of the constraint, {@code YoVariable}s will be created
    *                                  using this name.
    * @param offsetFromParentJoint     the position of the constraint with respect to the parent joint.
    * @param offsetFromLinkParentJoint the position of the constraint with respect to the parent joint
    *                                  of the associated link. Note that the link's parent joint is
    *                                  expected to be different from this constraint's parent joint.
    * @param robot                     the robot is used for getting access to its {@code YoRegistry}.
    * @param constraintForceSubSpace   defines the linear part of the sub-space in which the constraint
    *                                  is to be enforced.
    * @param constraintMomentSubSpace  defines the angular part of the sub-space in which the
    *                                  constraint is to be enforced.
    */
   public LoopClosureSoftConstraintSCS2(String name,
                                        Tuple3DReadOnly offsetFromParentJoint,
                                        Tuple3DReadOnly offsetFromLinkParentJoint,
                                        Matrix3DReadOnly constraintForceSubSpace,
                                        Matrix3DReadOnly constraintMomentSubSpace)
   {
      this.name = name;
      this.offsetFromParentJoint.set(offsetFromParentJoint);
      this.offsetFromLinkParentJoint.set(offsetFromLinkParentJoint);
      this.constraintForceSubSpace.set(constraintForceSubSpace);
      this.constraintMomentSubSpace.set(constraintMomentSubSpace);

      constraintDefinitionA = new ExternalWrenchPointDefinition(name + "A", offsetFromParentJoint);
      constraintDefinitionB = new ExternalWrenchPointDefinition(name + "B", offsetFromLinkParentJoint);
   }

   public void setGains(double proportionalGain, double derivativeGain)
   {
      proportionalGains.set(proportionalGain, proportionalGain, proportionalGain);
      derivativeGains.set(derivativeGain, derivativeGain, derivativeGain);
   }

   /**
    * Sets the gains to use for enforcing this constraint.
    * <p>
    * Note that the gains are applied on the error in the local coordinates of the parent joints.
    * </p>
    *
    * @param proportionalGains the gains to apply on the position and rotation errors.
    * @param derivativeGains   the gains to apply on the linear and angular velocity errors.
    */
   public void setGains(Tuple3DReadOnly proportionalGains, Tuple3DReadOnly derivativeGains)
   {
      this.proportionalGains.set(proportionalGains);
      this.derivativeGains.set(derivativeGains);
   }

   public void setParentJoint(SimJointBasics parentJoint)
   {
      this.parentJoint = parentJoint;
      constraintA = parentJoint.getAuxialiryData().addExternalWrenchPoint(constraintDefinitionA);

      YoRegistry registry = parentJoint.getRegistry();
      proportionalGains = new YoVector3D(name + "ProportionalGain", registry);
      derivativeGains = new YoVector3D(name + "DerivativeGain", registry);
      positionErrorMagnitude = new YoDouble(name + "PositionErrorMagnitude", registry);
      rotationErrorMagnitude = new YoDouble(name + "RotationErrorMagnitude", registry);

      positionError = new YoVector3D(name + "PositionError", registry);
      rotationError = new YoVector3D(name + "RotationError", registry);
      linearVelocityError = new YoVector3D(name + "LinearVelocityError", registry);
      angularVelocityError = new YoVector3D(name + "AngularVelocityError", registry);
      feedForwardForce = new YoVector3D(name + "FeedForwardForce", registry);
      feedForwardMoment = new YoVector3D(name + "FeedForwardMoment", registry);
   }

   public void setRigidBody(SimRigidBodyBasics rigidBody)
   {
      constraintB = rigidBody.getParentJoint().getAuxialiryData().addExternalWrenchPoint(constraintDefinitionB);
   }

   private boolean isFirstUpdate = true;

   @Override
   public void doControl()
   {
      if (isFirstUpdate)
      {
         if (proportionalGains.containsNaN() || derivativeGains.containsNaN())
            throw new IllegalArgumentException("The gains for the loop closure constraint: " + name
                  + " have not been configured. If created from description, see: " + LoopClosureSoftConstraintSCS2.class.getSimpleName());
         if (TupleTools.isTupleZero(proportionalGains, 0.0) || TupleTools.isTupleZero(derivativeGains, 0.0))
            LogTools.warn("The gains for the loop closure constraint: " + name + " have not been configured. If created from description, see: "
                  + LoopClosureSoftConstraintSCS2.class.getSimpleName());

         isFirstUpdate = false;
      }

      MovingReferenceFrame jointFrame = parentJoint.getFrameAfterJoint();
      ReferenceFrame rootFrame = jointFrame.getRootFrame();
      MovingReferenceFrame frameA = constraintA.getFrame();
      MovingReferenceFrame frameB = constraintB.getFrame();

      // Position error in world
      positionError.sub(constraintB.getPose().getPosition(), constraintA.getPose().getPosition());
      // Position error in A's local coordinates.
      rootFrame.transformFromThisToDesiredFrame(frameA, positionError);
      // Applying the sub-space on the error so the visualization is accurate, i.e. doesn't incorporate error that does not matter.
      constraintForceSubSpace.transform(positionError);
      positionErrorMagnitude.set(positionError.length());

      // Rotation error in A's local coordinates.
      quaternionDifference.difference(constraintA.getPose().getOrientation(), constraintB.getPose().getOrientation()); // This the orientation from B to A
      quaternionDifference.normalizeAndLimitToPi();
      quaternionDifference.getRotationVector(rotationError);
      constraintMomentSubSpace.transform(rotationError);
      rotationErrorMagnitude.set(rotationError.length());

      // Linear velocity error in A's local coordinates
      linearVelocityError.set(constraintB.getTwist().getLinearPart());
      frameB.transformFromThisToDesiredFrame(frameA, linearVelocityError);
      linearVelocityError.sub(constraintA.getTwist().getLinearPart());
      constraintForceSubSpace.transform(linearVelocityError);

      // Angular velocity in A's local coordinates
      angularVelocityError.set(constraintB.getTwist().getAngularPart());
      frameB.transformFromThisToDesiredFrame(frameA, angularVelocityError);
      angularVelocityError.sub(constraintA.getTwist().getAngularPart());
      constraintMomentSubSpace.transform(angularVelocityError);

      proportionalTermLinear.set(positionError);
      proportionalTermLinear.scale(proportionalGains.getX(), proportionalGains.getY(), proportionalGains.getZ());
      proportionalTermAngular.set(rotationError);
      proportionalTermAngular.scale(proportionalGains.getX(), proportionalGains.getY(), proportionalGains.getZ());
      derivativeTermLinear.set(linearVelocityError);
      derivativeTermLinear.scale(derivativeGains.getX(), derivativeGains.getY(), derivativeGains.getZ());
      derivativeTermAngular.set(angularVelocityError);
      derivativeTermAngular.scale(derivativeGains.getX(), derivativeGains.getY(), derivativeGains.getZ());

      force.set(proportionalTermLinear);
      force.add(derivativeTermLinear);
      force.add(feedForwardForce);
      moment.set(proportionalTermAngular);
      moment.add(derivativeTermAngular);
      moment.add(feedForwardMoment);

      constraintA.getWrench().set(moment, force);

      frameA.transformFromThisToDesiredFrame(frameB, force);
      frameA.transformFromThisToDesiredFrame(frameB, moment);

      force.negate();
      moment.negate();
      constraintB.getWrench().set(moment, force);
   }

   /**
    * Set the feed-forward force and moment to be applied at the constraint in addition to the error
    * terms.
    */
   public void setFeedForward(Vector3DReadOnly force, Vector3DReadOnly moment)
   {
      constraintForceSubSpace.transform(force, feedForwardForce);
      constraintMomentSubSpace.transform(moment, feedForwardMoment);
   }

   @Override
   public String getName()
   {
      return name;
   }
}
