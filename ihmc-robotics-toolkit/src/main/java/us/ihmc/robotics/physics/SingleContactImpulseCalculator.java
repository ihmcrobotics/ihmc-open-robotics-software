package us.ihmc.robotics.physics;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.algorithms.ForwardDynamicsCalculator;
import us.ihmc.mecano.algorithms.MultiBodyResponseCalculator;
import us.ihmc.mecano.algorithms.interfaces.RigidBodyAccelerationProvider;
import us.ihmc.mecano.algorithms.interfaces.RigidBodyTwistProvider;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.SpatialImpulse;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialImpulseReadOnly;

/**
 * From <i>"Per-Contact Iteration Method for Solving Contact Dynamics"</i>
 *
 * @author Sylvain Bertrand
 */
public class SingleContactImpulseCalculator implements ImpulseBasedConstraintCalculator
{
   private double beta1 = 0.35;
   private double beta2 = 0.95;
   private double beta3 = 1.15;
   private double gamma = 1.0e-6;
   private final ContactParameters contactParameters = new ContactParameters(0.7, 0.0, 0.0, 1.0);

   private boolean isFirstUpdate = false;
   private boolean isImpulseZero = false;
   private boolean isContactClosing = false;

   private final ForwardDynamicsCalculator forwardDynamicsCalculatorA;
   private final ForwardDynamicsCalculator forwardDynamicsCalculatorB;

   private final MultiBodyResponseCalculator responseCalculatorA;
   private final MultiBodyResponseCalculator responseCalculatorB;

   private final FramePoint3D pointA = new FramePoint3D();
   private final FramePoint3D pointB = new FramePoint3D();

   private final FrameVector3D velocityNoImpulseA = new FrameVector3D();
   private final FrameVector3D velocityNoImpulseB = new FrameVector3D();

   private final FrameVector3D velocityDueToOtherImpulseA = new FrameVector3D();
   private final FrameVector3D velocityDueToOtherImpulseB = new FrameVector3D();

   private final FrameVector3D velocityA = new FrameVector3D();
   private final FrameVector3D velocityB = new FrameVector3D();
   private final FrameVector3D velocityRelative = new FrameVector3D();
   private final FrameVector3D velocityRelativePrevious = new FrameVector3D();
   private final FrameVector3D velocityRelativeChange = new FrameVector3D();

   private final DenseMatrix64F inverseApparentInertiaA = new DenseMatrix64F(3, 3);
   private final DenseMatrix64F inverseApparentInertiaB = new DenseMatrix64F(3, 3);

   private final SpatialImpulse impulseA = new SpatialImpulse();
   private final SpatialImpulse impulseB = new SpatialImpulse();
   private final FrameVector3D impulsePreviousA = new FrameVector3D();
   private final FrameVector3D impulseChangeA = new FrameVector3D();

   private final Point3D contactFramePosition = new Point3D();
   private final Quaternion contactFrameOrientation = new Quaternion();
   private final ReferenceFrame contactFrame;

   private final DenseMatrix64F c = new DenseMatrix64F(3, 1);
   private final DenseMatrix64F M_inv = new DenseMatrix64F(3, 3);
   private final DenseMatrix64F lambda_v_0 = new DenseMatrix64F(3, 1);
   private final LinearSolver<DenseMatrix64F> linearSolver = LinearSolverFactory.symmPosDef(3);
   private final LinearSolver<DenseMatrix64F> svdSolver = LinearSolverFactory.pseudoInverse(true);
   private final DenseMatrix64F M_inv_solverCopy = new DenseMatrix64F(3, 3);

   private final RigidBodyBasics rootA, rootB;

   private RigidBodyTwistProvider externalRigidBodyTwistModifier;
   private final ImpulseBasedRigidBodyTwistProvider rigidBodyTwistModifierA, rigidBodyTwistModifierB;
   private final ImpulseBasedJointTwistProvider jointTwistModifierA, jointTwistModifierB;

   private CollisionResult collisionResult;
   private RigidBodyBasics contactingBodyA, contactingBodyB;
   private MovingReferenceFrame bodyFrameA, bodyFrameB;

   public SingleContactImpulseCalculator(ReferenceFrame rootFrame, RigidBodyBasics rootBodyA, ForwardDynamicsCalculator forwardDynamicsCalculatorA,
                                         RigidBodyBasics rootBodyB, ForwardDynamicsCalculator forwardDynamicsCalculatorB)
   {
      this.forwardDynamicsCalculatorA = forwardDynamicsCalculatorA;
      this.forwardDynamicsCalculatorB = forwardDynamicsCalculatorB;

      rootA = rootBodyA;
      rootB = rootBodyB;

      contactFrame = new ReferenceFrame("contactFrame", rootFrame, true, false)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.set(contactFrameOrientation, contactFramePosition);
         }
      };

      velocityA.setReferenceFrame(contactFrame);
      velocityB.setReferenceFrame(contactFrame);
      velocityRelative.setReferenceFrame(contactFrame);
      velocityRelativePrevious.setReferenceFrame(contactFrame);
      velocityRelativeChange.setReferenceFrame(contactFrame);

      impulsePreviousA.setReferenceFrame(contactFrame);
      impulseChangeA.setReferenceFrame(contactFrame);

      responseCalculatorA = new MultiBodyResponseCalculator(forwardDynamicsCalculatorA);

      if (forwardDynamicsCalculatorB != null)
         responseCalculatorB = new MultiBodyResponseCalculator(forwardDynamicsCalculatorB);
      else
         responseCalculatorB = null;

      rigidBodyTwistModifierA = new ImpulseBasedRigidBodyTwistProvider(rootFrame, rootA);
      jointTwistModifierA = new ImpulseBasedJointTwistProvider(rootA);

      if (forwardDynamicsCalculatorB != null)
      {
         rigidBodyTwistModifierB = new ImpulseBasedRigidBodyTwistProvider(rootFrame, rootB);
         jointTwistModifierB = new ImpulseBasedJointTwistProvider(rootB);
      }
      else
      {
         rigidBodyTwistModifierB = null;
         jointTwistModifierB = null;
      }
   }

   public void setTolerance(double gamma)
   {
      this.gamma = gamma;
   }

   public void setContactParameters(ContactParametersReadOnly parameters)
   {
      contactParameters.set(parameters);
   }

   @Override
   public void setExternalTwistModifier(RigidBodyTwistProvider externalRigidBodyTwistModifier)
   {
      this.externalRigidBodyTwistModifier = externalRigidBodyTwistModifier;
   }

   public void setCollision(CollisionResult collisionResult)
   {
      Collidable collidableA = collisionResult.getCollidableA();
      Collidable collidableB = collisionResult.getCollidableB();

      if (collidableA.getRootBody() != rootA || collidableB.getRootBody() != rootB)
         throw new IllegalArgumentException("Robot mismatch!");

      this.collisionResult = collisionResult;
      contactingBodyA = collidableA.getRigidBody();
      contactingBodyB = collidableB.getRigidBody();
      bodyFrameA = contactingBodyA.getBodyFixedFrame();
      bodyFrameB = contactingBodyB == null ? null : contactingBodyB.getBodyFixedFrame();
   }

   @Override
   public void initialize(double dt)
   {
      EuclidGeometryTools.orientation3DFromZUpToVector3D(collisionResult.getCollisionAxisForA(), contactFrameOrientation);
      contactFramePosition.set(collisionResult.getPointOnARootFrame());
      contactFrame.update();

      pointA.setIncludingFrame(collisionResult.getCollisionData().getPointOnA());
      pointA.changeFrame(bodyFrameA);

      computeContactPointVelocity(dt, rootA, contactingBodyA, forwardDynamicsCalculatorA.getAccelerationProvider(false), pointA, velocityNoImpulseA);
      velocityNoImpulseA.changeFrame(contactFrame);

      pointB.setIncludingFrame(collisionResult.getCollisionData().getPointOnB());

      if (rootB != null)
      {
         pointB.changeFrame(bodyFrameB);
         computeContactPointVelocity(dt, rootB, contactingBodyB, forwardDynamicsCalculatorB.getAccelerationProvider(false), pointB, velocityNoImpulseB);
         velocityNoImpulseB.changeFrame(contactFrame);
      }

      isFirstUpdate = true;
   }

   @Override
   public void updateInertia(List<? extends RigidBodyBasics> rigidBodyTargets, List<? extends JointBasics> jointTargets)
   {
      rigidBodyTwistModifierA.clear(3);
      jointTwistModifierA.clear(3);
      if (rigidBodyTargets != null)
         rigidBodyTwistModifierA.addAll(rigidBodyTargets);
      if (jointTargets != null)
         jointTwistModifierA.addAll(jointTargets);

      if (rootB != null)
      {
         rigidBodyTwistModifierB.clear(3);
         jointTwistModifierB.clear(3);
         if (rigidBodyTargets != null)
            rigidBodyTwistModifierB.addAll(rigidBodyTargets);
         if (jointTargets != null)
            jointTwistModifierB.addAll(jointTargets);
      }

      // First we evaluate M^-1 that is the inverse of the apparent inertia considering both bodies interacting in this contact.
      computeApparentInertiaInverse(contactingBodyA, responseCalculatorA, rigidBodyTwistModifierA, jointTwistModifierA, inverseApparentInertiaA);

      if (rootB != null)
      {
         computeApparentInertiaInverse(contactingBodyB, responseCalculatorB, rigidBodyTwistModifierB, jointTwistModifierB, inverseApparentInertiaB);
         CommonOps.add(inverseApparentInertiaA, inverseApparentInertiaB, M_inv);
      }
      else
      {
         M_inv.set(inverseApparentInertiaA);
      }
   }

   public static void computeContactPointVelocity(double dt, RigidBodyReadOnly rootBody, RigidBodyReadOnly contactingBody,
                                                  RigidBodyAccelerationProvider noVelocityRigidBodyAccelerationProvider, FramePoint3DReadOnly contactPoint,
                                                  FrameVector3DBasics linearVelocityToPack)
   {
      MovingReferenceFrame bodyFixedFrame = contactingBody.getBodyFixedFrame();

      // Using acceleration relative to the root to avoid integrating the gravitational acceleration.
      SpatialAccelerationReadOnly contactingBodyAcceleration = noVelocityRigidBodyAccelerationProvider.getRelativeAcceleration(rootBody, contactingBody);
      contactingBodyAcceleration.getLinearAccelerationAt(null, contactPoint, linearVelocityToPack);
      linearVelocityToPack.scale(dt);
      double vx = linearVelocityToPack.getX();
      double vy = linearVelocityToPack.getY();
      double vz = linearVelocityToPack.getZ();

      bodyFixedFrame.getTwistOfFrame().getLinearVelocityAt(contactPoint, linearVelocityToPack);
      linearVelocityToPack.add(vx, vy, vz);
   }

   private final FrameVector3D collisionPositionTerm = new FrameVector3D();

   @Override
   public void updateImpulse(double dt, double alpha, boolean ignoreOtherImpulses)
   {
      if (!ignoreOtherImpulses && externalRigidBodyTwistModifier != null)
      {
         // Compute the change in twist due to other impulses.
         velocityDueToOtherImpulseA.setIncludingFrame(externalRigidBodyTwistModifier.getLinearVelocityOfBodyFixedPoint(contactingBodyA, pointA));
         velocityDueToOtherImpulseA.changeFrame(contactFrame);
         velocityA.add(velocityNoImpulseA, velocityDueToOtherImpulseA);
      }
      else
      {
         velocityA.set(velocityNoImpulseA);
      }

      if (rootB != null)
      {
         if (!ignoreOtherImpulses && externalRigidBodyTwistModifier != null)
         {
            // Compute the change in twist due to other impulses.
            velocityDueToOtherImpulseB.setIncludingFrame(externalRigidBodyTwistModifier.getLinearVelocityOfBodyFixedPoint(contactingBodyB, pointB));
            velocityDueToOtherImpulseB.changeFrame(contactFrame);
            velocityB.add(velocityNoImpulseB, velocityDueToOtherImpulseB);
         }
         else
         {
            velocityB.set(velocityNoImpulseB);
         }

         velocityRelative.sub(velocityA, velocityB);
      }
      else
      {
         velocityRelative.set(velocityA);
      }

      if (isFirstUpdate)
      {
         velocityRelativePrevious.set(velocityRelative);
         velocityRelativeChange.set(velocityRelative);
      }
      else
      {
         velocityRelativeChange.sub(velocityRelative, velocityRelativePrevious);
         velocityRelativePrevious.set(velocityRelative);
      }

      /*
       * Modifying the contact velocity that the solver is trying to cancel. For instance, the coefficient
       * of restitution is 1.0, the velocity is doubled, which results in an impulse which magnitude is
       * doubled, such that, the post-impact velocity is opposite of the pre-impact velocity along the
       * collision axis.
       */
      if (contactParameters.getCoefficientOfRestitution() != 0.0)
         velocityRelative.scale(1.0 + contactParameters.getCoefficientOfRestitution());

      /*
       * Computing the correction term based on the penetration of the two collidables. This assumes that
       * the two shapes are inter-penetrating. The penetration distance is transformed into a velocity
       * that would allow to correct the error in a single tick, this velocity is scaled with the user
       * parameter error-reduction-parameter which is in [0, 1]. The resulting is subtracted to the
       * relative velocity that the solver is trying to cancel, this way the calculator will implicitly
       * account for the error.
       */
      if (contactParameters.getErrorReductionParameter() != 0.0)
      {
         collisionPositionTerm.setIncludingFrame(collisionResult.getPointOnBRootFrame());
         collisionPositionTerm.sub(collisionResult.getPointOnARootFrame());
         collisionPositionTerm.scale(contactParameters.getErrorReductionParameter() / dt);
         collisionPositionTerm.changeFrame(velocityRelative.getReferenceFrame());
         velocityRelative.sub(collisionPositionTerm);
      }

      isContactClosing = velocityRelative.getZ() < 0.0;
      impulseA.setToZero(bodyFrameA, contactFrame);

      if (isContactClosing)
      { // Closing contact, impulse needs to be calculated.
         double mu = contactParameters.getCoefficientOfFriction();

         if (EuclidCoreTools.isZero(mu, 1.0e-12))
         { // Trivial case, i.e. there is no friction => the impulse is along the collision axis.
            impulseA.getLinearPart().setZ(-velocityRelative.getZ() / M_inv.get(2, 2));
         }
         else
         {
            velocityRelative.get(c);
            LinearSolver<DenseMatrix64F> solver;
            if (CommonOps.det(M_inv) > 1.0e-6)
               solver = linearSolver;
            else
               solver = svdSolver; // TODO This is not enough to cover the degenerate case, need to decompose the problem to work in reduced space, i.e. either 2D or 1D.

            M_inv_solverCopy.set(M_inv);
            solver.setA(M_inv_solverCopy);
            solver.solve(c, lambda_v_0);
            CommonOps.changeSign(lambda_v_0);

            if (lambda_v_0.get(2) > -1.0e-12 && ContactImpulseTools.isInsideFrictionCone(mu, lambda_v_0))
            { // Contact is sticking, i.e. satisfies Coulomb's friction cone while canceling velocity.
               impulseA.getLinearPart().set(lambda_v_0);
            }
            else
            { // Contact is slipping, that's the though case.
               ContactImpulseTools.computeSlipLambda(beta1, beta2, beta3, gamma, mu, M_inv, lambda_v_0, c, impulseA.getLinearPart(), false);
            }
         }
      }

      if (impulseA.getLinearPart().getZ() < 0.0)
         throw new IllegalStateException("Malformed impulse");

      /*
       * Based on the documentation of ODE, this seems to be the way the constraint force mixing should be
       * applied.
       */
      if (contactParameters.getConstraintForceMixing() != 1.0)
         impulseA.getLinearPart().scale(contactParameters.getConstraintForceMixing());

      if (isFirstUpdate)
      {
         if (isContactClosing)
         {
            impulseChangeA.set(impulseA.getLinearPart());
            isImpulseZero = !isContactClosing;
         }
         else
         {
            impulseChangeA.setToZero(contactFrame);
            isImpulseZero = true;
         }
      }
      else
      {
         impulseA.getLinearPart().interpolate(impulsePreviousA, impulseA.getLinearPart(), alpha);
         impulseChangeA.sub(impulseA.getLinearPart(), impulsePreviousA);
         isImpulseZero = impulseA.getLinearPart().length() < 1.0e-6;
      }

      if (isImpulseZero)
      {
         if (rootB != null)
         {
            impulseB.setToZero(contactingBodyB.getBodyFixedFrame(), impulseA.getReferenceFrame());
         }
      }
      else
      {
         if (rootB != null)
         {
            impulseB.setIncludingFrame(contactingBodyB.getBodyFixedFrame(), impulseA);
            impulseB.negate();
         }
      }

      impulsePreviousA.set(impulseA.getLinearPart());
      isFirstUpdate = false;
   }

   @Override
   public void updateTwistModifiers()
   {
      if (isImpulseZero)
      {
         rigidBodyTwistModifierA.setImpulseToZero();
         jointTwistModifierA.setImpulseToZero();

         if (rootB != null)
         {
            rigidBodyTwistModifierB.setImpulseToZero();
            jointTwistModifierB.setImpulseToZero();
         }
      }
      else
      {
         rigidBodyTwistModifierA.setImpulse(impulseA.getLinearPart());
         jointTwistModifierA.setImpulse(impulseA.getLinearPart());

         if (rootB != null)
         {
            rigidBodyTwistModifierB.setImpulse(impulseB.getLinearPart());
            jointTwistModifierB.setImpulse(impulseB.getLinearPart());
         }
      }
   }

   @Override
   public void finalizeImpulse()
   {
      responseCalculatorA.applyRigidBodyImpulse(contactingBodyA, impulseA);
      if (rootB != null)
         responseCalculatorB.applyRigidBodyImpulse(contactingBodyB, impulseB);
   }

   private final SpatialImpulse testImpulse = new SpatialImpulse();
   private final Twist testTwist = new Twist();

   private void computeApparentInertiaInverse(RigidBodyBasics body, MultiBodyResponseCalculator calculator,
                                              ImpulseBasedRigidBodyTwistProvider rigidBodyTwistModifierToUpdate,
                                              ImpulseBasedJointTwistProvider jointTwistModifierToUpdate, DenseMatrix64F inertiaMatrixToPack)
   {
      calculator.reset();
      inertiaMatrixToPack.reshape(3, 3);
      RigidBodyTwistProvider twistChangeProvider = calculator.getTwistChangeProvider();

      for (int axis = 0; axis < 3; axis++)
      {
         testImpulse.setIncludingFrame(body.getBodyFixedFrame(), contactFrame, EuclidCoreTools.zeroVector3D, Axis3D.values[axis]);

         if (!calculator.applyRigidBodyImpulse(body, testImpulse))
            throw new IllegalStateException("Something went wrong with the response calculator");

         testTwist.setIncludingFrame(twistChangeProvider.getTwistOfBody(body));
         testTwist.changeFrame(contactFrame);
         testTwist.getLinearPart().get(0, axis, inertiaMatrixToPack);

         for (RigidBodyBasics externalTarget : rigidBodyTwistModifierToUpdate.getRigidBodies())
         {
            DenseMatrix64F externalInertiaMatrix = rigidBodyTwistModifierToUpdate.getApparentInertiaMatrixInverse(externalTarget);
            twistChangeProvider.getTwistOfBody(externalTarget).get(0, axis, externalInertiaMatrix);
         }

         for (JointBasics externalTarget : jointTwistModifierToUpdate.getJoints())
         {
            DenseMatrix64F externalInertiaMatrix = jointTwistModifierToUpdate.getApparentInertiaMatrixInverse(externalTarget);
            CommonOps.insert(calculator.getJointTwistChange(externalTarget), externalInertiaMatrix, 0, axis);
         }

         calculator.reset();
      }
   }

   public void readExternalWrench(double dt, List<ExternalWrenchReader> externalWrenchReaders)
   {
      externalWrenchReaders.forEach(reader -> readExternalWrench(dt, reader));
   }

   public void readExternalWrench(double dt, ExternalWrenchReader externalWrenchReader)
   {
      Wrench externalWrenchA = new Wrench();
      externalWrenchA.setIncludingFrame(impulseA.getBodyFrame(), impulseA);
      externalWrenchA.scale(1.0 / dt);
      externalWrenchA.changeFrame(bodyFrameA);
      externalWrenchReader.readExternalWrench(contactingBodyA, externalWrenchA);

      if (rootB != null)
      {
         Wrench externalWrenchB = new Wrench();
         externalWrenchB.setIncludingFrame(impulseB.getBodyFrame(), impulseB);
         externalWrenchB.scale(1.0 / dt);
         externalWrenchB.changeFrame(bodyFrameB);
         externalWrenchReader.readExternalWrench(contactingBodyB, externalWrenchB);
      }
   }

   @Override
   public double getImpulseUpdate()
   {
      return impulseChangeA.length();
   }

   @Override
   public double getVelocityUpdate()
   {
      return velocityRelativeChange.length();
   }

   @Override
   public boolean isConstraintActive()
   {
      return !isImpulseZero;
   }

   public boolean isContactClosing()
   {
      return isContactClosing;
   }

   public CollisionResult getCollisionResult()
   {
      return collisionResult;
   }

   public RigidBodyBasics getContactingBodyA()
   {
      return contactingBodyA;
   }

   public ForwardDynamicsCalculator getForwardDynamicsCalculatorA()
   {
      return forwardDynamicsCalculatorA;
   }

   public RigidBodyBasics getContactingBodyB()
   {
      return contactingBodyB;
   }

   public ForwardDynamicsCalculator getForwardDynamicsCalculatorB()
   {
      return forwardDynamicsCalculatorB;
   }

   @Override
   public int getNumberOfRobotsInvolved()
   {
      return responseCalculatorB == null ? 1 : 2;
   }

   @Override
   public RigidBodyBasics getRootBody(int index)
   {
      return index == 0 ? rootA : rootB;
   }

   @Override
   public RigidBodyTwistProvider getRigidBodyTwistChangeProvider(int index)
   {
      return index == 0 ? rigidBodyTwistModifierA : rigidBodyTwistModifierB;
   }

   @Override
   public JointStateProvider getJointTwistChangeProvider(int index)
   {
      return index == 0 ? jointTwistModifierA : jointTwistModifierB;
   }

   @Override
   public DenseMatrix64F getJointVelocityChange(int index)
   {
      return index == 0 ? getJointVelocityChangeA() : getJointVelocityChangeB();
   }

   public DenseMatrix64F getJointVelocityChangeA()
   {
      if (!isConstraintActive())
         return null;

      return responseCalculatorA.propagateImpulse();
   }

   public DenseMatrix64F getJointVelocityChangeB()
   {
      if (rootB == null || !isConstraintActive())
         return null;

      return responseCalculatorB.propagateImpulse();
   }

   public SpatialImpulseReadOnly getImpulseA()
   {
      return impulseA;
   }

   public SpatialImpulseReadOnly getImpulseB()
   {
      return impulseB;
   }

   public FramePoint3DReadOnly getPointA()
   {
      return pointA;
   }

   public FramePoint3DReadOnly getPointB()
   {
      return pointB;
   }

   public FrameVector3DReadOnly getVelocityRelative()
   {
      return velocityRelative;
   }

   public FrameVector3DReadOnly getVelocityNoImpulseA()
   {
      return velocityNoImpulseA;
   }

   public FrameVector3DReadOnly getVelocityNoImpulseB()
   {
      return velocityNoImpulseB;
   }

   public FrameVector3DReadOnly getVelocityDueToOtherImpulseA()
   {
      return velocityDueToOtherImpulseA;
   }

   public FrameVector3DReadOnly getVelocityDueToOtherImpulseB()
   {
      return velocityDueToOtherImpulseB;
   }

   public MultiBodyResponseCalculator getResponseCalculatorA()
   {
      return responseCalculatorA;
   }

   public MultiBodyResponseCalculator getResponseCalculatorB()
   {
      return responseCalculatorB;
   }

   public ContactParametersBasics getContactParameters()
   {
      return contactParameters;
   }

   @Override
   public List<? extends JointBasics> getJointTargets()
   {
      return Collections.emptyList();
   }

   @Override
   public List<? extends RigidBodyBasics> getRigidBodyTargets()
   {
      if (rootB == null)
         return Collections.singletonList(contactingBodyA);
      else
         return Arrays.asList(contactingBodyA, contactingBodyB);
   }

   public void printForUnitTest()
   {
      System.err.println(ContactImpulseTools.toStringForUnitTest(beta1,
                                                                 beta2,
                                                                 beta3,
                                                                 gamma,
                                                                 contactParameters.getCoefficientOfFriction(),
                                                                 M_inv,
                                                                 lambda_v_0,
                                                                 c));
   }
}
