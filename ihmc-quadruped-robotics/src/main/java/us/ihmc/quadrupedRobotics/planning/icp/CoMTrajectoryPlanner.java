package us.ihmc.quadrupedRobotics.planning.icp;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.*;

import java.util.List;

// This guy assumes that the final phase is always the "stopping" phase, where the CoM is supposed to come to rest.
// This means that the final CoP is the terminal ICP location
public class CoMTrajectoryPlanner
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DenseMatrix64F coefficientMultipliers = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F xCoefficientConstants = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F yCoefficientConstants = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F zCoefficientConstants = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F xCoefficientVector = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F yCoefficientVector = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F zCoefficientVector = new DenseMatrix64F(0, 1);

   private final FramePoint3D currentCoMPosition = new FramePoint3D();
   private final FramePoint3D finalDCMPosition = new FramePoint3D();

   private final LinearSolver<DenseMatrix64F> solver = LinearSolverFactory.linear(0);

   private final YoDouble omega;
   private final double gravityZ;
   private double nominalCoMHeight;

   private final List<QuadrupedContactPhase> contactSequence;

   private final YoFramePoint3D desiredCoMPosition = new YoFramePoint3D("desiredCoMPosition", worldFrame, registry);
   private final YoFrameVector3D desiredCoMVelocity = new YoFrameVector3D("desiredComVelocity", worldFrame, registry);
   private final YoFramePoint3D desiredDCMPosition = new YoFramePoint3D("desiredDCMPosition", worldFrame, registry);
   private final YoFrameVector3D desiredDCMVelocity = new YoFrameVector3D("desiredDCMVelocity", worldFrame, registry);

   private final YoFramePoint3D firstCoefficient = new YoFramePoint3D("comFirstCoefficient", worldFrame, registry);
   private final YoFramePoint3D secondCoefficient = new YoFramePoint3D("comSecondCoefficient", worldFrame, registry);

   private int numberOfConstraints = 0;


   public CoMTrajectoryPlanner(List<QuadrupedContactPhase> contactSequence, YoDouble omega, double gravityZ, double nominalCoMHeight, YoVariableRegistry parentRegistry)
   {
      this.contactSequence = contactSequence;
      this.omega = omega;
      this.nominalCoMHeight = nominalCoMHeight;
      this.gravityZ = Math.abs(gravityZ);

      parentRegistry.addChild(registry);
   }

   /**
    * Sets the nominal CoM height to be used by the planner.
    */
   public void setNominalCoMHeight(double nominalCoMHeight)
   {
      this.nominalCoMHeight = nominalCoMHeight;
   }

   public void solveForTrajectory()
   {
      int size = 2 * contactSequence.size();
      coefficientMultipliers.reshape(size, size);
      xCoefficientConstants.reshape(size, 1);
      yCoefficientConstants.reshape(size, 1);
      zCoefficientConstants.reshape(size, 1);
      xCoefficientVector.reshape(size, 1);
      yCoefficientVector.reshape(size, 1);
      zCoefficientVector.reshape(size, 1);

      coefficientMultipliers.zero();
      xCoefficientConstants.zero();
      yCoefficientConstants.zero();
      zCoefficientConstants.zero();
      xCoefficientVector.zero();
      yCoefficientVector.zero();
      zCoefficientVector.zero();

      int numberOfPhases = contactSequence.size();
      int numberOfTransitions = numberOfPhases - 1;

      numberOfConstraints = 0;

      // set initial constraint
      setCoMPositionEqualityInContact(0, 0.0, currentCoMPosition);

      // add transition continuity constraints
      for (int transition = 0; transition < numberOfTransitions; transition++)
      {
         int previousSequence = transition;
         int nextSequence = transition + 1;
         setPositionContinuity(previousSequence, nextSequence);
         setVelocityContinuity(previousSequence, nextSequence);
      }

      // set terminal constraint
      QuadrupedContactPhase lastContactPhase = contactSequence.get(numberOfPhases - 1);
      finalDCMPosition.set(lastContactPhase.getCopPosition());
      finalDCMPosition.addZ(nominalCoMHeight);
      setDCMTerminalConstraint(numberOfPhases - 1, finalDCMPosition);

      // TODO this can probably be made more efficient by inverting the multipliers matrix
      // solve for coefficients
      solver.setA(coefficientMultipliers);
      solver.solve(xCoefficientConstants, xCoefficientVector);
      solver.solve(yCoefficientConstants, yCoefficientVector);
      solver.solve(zCoefficientConstants, zCoefficientVector);

      firstCoefficient.setX(xCoefficientVector.get(getFirstCoefficient(0)));
      firstCoefficient.setY(yCoefficientVector.get(getFirstCoefficient(0)));
      firstCoefficient.setZ(zCoefficientVector.get(getFirstCoefficient(0)));

      secondCoefficient.setX(xCoefficientVector.get(getSecondCoefficient(0)));
      secondCoefficient.setY(yCoefficientVector.get(getSecondCoefficient(0)));
      secondCoefficient.setZ(zCoefficientVector.get(getSecondCoefficient(0)));
   }

   /**
    * Computes the desired values.
    * @param timeInPhase time in the current phase. Note that this assumes that the phase starts at 0.0.
    */
   public void compute(double timeInPhase)
   {
      QuadrupedContactPhase currentContactPhase = contactSequence.get(0);

      ContactState contactState = currentContactPhase.getContactState();
      double firstCoefficientPositionMultiplier = getFirstCoefficientPositionMultiplier(contactState, timeInPhase);
      double secondCoefficientPositionMultiplier = getSecondCoefficientPositionMultiplier(contactState, timeInPhase);

      double firstCoefficientVelocityMultiplier = getFirstCoefficientVelocityMultiplier(contactState, timeInPhase);
      double secondCoefficientVelocityMultiplier = getSecondCoefficientVelocityMultiplier(contactState, timeInPhase);
      double gravityPositionEffect = getGravityPositionEffect(contactState, timeInPhase);
      double gravityVelocityEffect = getGravityVelocityEffect(contactState, timeInPhase);

      if (contactState == ContactState.IN_CONTACT)
      {
         desiredCoMPosition.set(currentContactPhase.getCopPosition());
         desiredCoMPosition.addZ(nominalCoMHeight);
      }
      else
      {
         desiredCoMPosition.setToZero();
      }
      desiredCoMPosition.scaleAdd(firstCoefficientPositionMultiplier, firstCoefficient, desiredCoMPosition);
      desiredCoMPosition.scaleAdd(secondCoefficientPositionMultiplier, secondCoefficient, desiredCoMPosition);
      desiredCoMPosition.addZ(gravityPositionEffect);

      desiredCoMVelocity.setToZero();
      desiredCoMVelocity.scaleAdd(firstCoefficientVelocityMultiplier, firstCoefficient, desiredCoMVelocity);
      desiredCoMVelocity.scaleAdd(secondCoefficientVelocityMultiplier, secondCoefficient, desiredCoMVelocity);
      desiredCoMVelocity.addZ(gravityVelocityEffect);

      desiredDCMPosition.scaleAdd(1.0 / omega.getDoubleValue(), desiredCoMVelocity, desiredCoMPosition);

      desiredDCMVelocity.set(firstCoefficient);
      desiredDCMVelocity.scale(2.0 * firstCoefficientVelocityMultiplier);
   }

   public void setCurrentCoMPosition(FramePoint3DReadOnly currentCoMPosition)
   {
      this.currentCoMPosition.setIncludingFrame(currentCoMPosition);
   }

   public FramePoint3DReadOnly getDesiredDCMPosition()
   {
      return desiredDCMPosition;
   }

   public FrameVector3DReadOnly getDesiredDCMVelocity()
   {
      return desiredDCMVelocity;
   }

   public FramePoint3DReadOnly getDesiredCoMPosition()
   {
      return desiredCoMPosition;
   }

   /**
    * <p> Sets the continuity constraint on the initial position. This DOES result in a initial discontinuity on the desired DCM location. </p>
    * <p> This constraint should be used for the initial position of the center of mass to properly initialize the trajectory. </p><
    * <p> Recall that the equation for the center of mass is defined by </p>
    * <p>
    *    x<sub>i</sub>(t) = c<sub>0,i</sub> e<sup>&omega; t</sup> + c<sub>1,i</sub> e<sup>-&omega; t</sup> + c<sub>2,i</sub> t + c<sub>3,i</sub>
    * </p>
    * <p>
    *    This results in the following constraint:
    * </p>
    * <p>
    *    c<sub>0,i</sub> e<sup>&omega; t</sup> + c<sub>1,i</sub> e<sup>-&omega; t</sup> + r<sub>vrp,i</sub>= x<sub>0</sub>
    * </p>
    * <p>
    *    c<sub>2,i</sub> = c<sub>3,i</sub> = 0
    * </p>
    * @param sequenceId i in the above equations
    * @param timeInPhaseForConstraint t in the above equations
    * @param centerOfMassLocationForConstraint x<sub>0</sub> in the above equations
    */
   private void setCoMPositionEqualityInContact(int sequenceId, double timeInPhaseForConstraint, FramePoint3DReadOnly centerOfMassLocationForConstraint)
   {
      centerOfMassLocationForConstraint.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      double omega = this.omega.getDoubleValue();

      double c0, c1, bX, bY, bZ;
      if (timeInPhaseForConstraint > 0)
      {
         c0 = Math.exp(omega * timeInPhaseForConstraint);
         c1 = Math.exp(-omega * timeInPhaseForConstraint);
      }
      else
      {
         c0 = 1.0;
         c1 = 1.0;
      }
      bX = centerOfMassLocationForConstraint.getX() - contactSequence.get(sequenceId).getCopPosition().getX();
      bY = centerOfMassLocationForConstraint.getY() - contactSequence.get(sequenceId).getCopPosition().getY();
      bZ = centerOfMassLocationForConstraint.getZ() - contactSequence.get(sequenceId).getCopPosition().getZ() - nominalCoMHeight;

      coefficientMultipliers.set(numberOfConstraints, getFirstCoefficient(sequenceId), c0);
      coefficientMultipliers.set(numberOfConstraints, getSecondCoefficient(sequenceId), c1);
      xCoefficientConstants.add(numberOfConstraints, 0, bX);
      yCoefficientConstants.add(numberOfConstraints, 0, bY);
      zCoefficientConstants.add(numberOfConstraints, 0, bZ);

      numberOfConstraints++;
   }

   /**
    * <p> Sets the terminal constraint on the center of mass trajectory. This constraint states that the final position should be equal to
    * {@param terminalPosition} and the desired velocity should be equal to 0. </p>
    * <p> Recall that the equation for the center of mass position is defined by </p>
    * <p>
    *    x<sub>i</sub>(t) = c<sub>0,i</sub> e<sup>&omega; t</sup> + c<sub>1,i</sub> e<sup>-&omega; t</sup> + c<sub>2,i</sub> t + c<sub>3,i</sub> + r<sub>vrp,i</sub>
    * </p>
    * <p> and the center of mass velocity is defined by </p>
    * <p>
    *    v<sub>i</sub>(t) = &omega; c<sub>0,i</sub> e<sup>&omega; t</sup> - &omega; c<sub>1,i</sub> e<sup>-&omega; t</sup> + c<sub>2,i</sub>
    * </p>
    * <p> When combined, this makes the DCM trajectory </p>
    * <p>
    *    &xi;<sub>i</sub>(t) = c<sub>0,i</sub> 2.0 e<sup>&omega; t</sup> + r<sub>vrp,i</sub>
    * </p>
    * <p> The number of variables can be reduced because of this knowledge, making the constraint:</p>
    * <p>
    *    c<sub>0,i</sub> 2.0 e<sup>&omega; T<sub>i</sub></sup> + r<sub>vrp,i</sub> = x<sub>f</sub>
    * </p>
    * <p>
    *    c<sub>2,i</sub> = c<sub>3,i</sub> = 0
    * </p>
    * @param sequenceId i in the above equations
    * @param terminalDCMPosition desired final location. x<sub>f</sub> in the above equations.
    */
   private void setDCMTerminalConstraint(int sequenceId, FramePoint3DReadOnly terminalDCMPosition)
   {
      terminalDCMPosition.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      double omega = this.omega.getDoubleValue();

      double duration = contactSequence.get(sequenceId).getTimeInterval().getDuration();

      double c0 = 2.0 * Math.exp(omega * duration);
      double bX = terminalDCMPosition.getX() - contactSequence.get(sequenceId).getCopPosition().getX();
      double bY = terminalDCMPosition.getY() - contactSequence.get(sequenceId).getCopPosition().getY();
      double bZ = terminalDCMPosition.getZ() - (contactSequence.get(sequenceId).getCopPosition().getZ() + nominalCoMHeight);

      coefficientMultipliers.set(numberOfConstraints, getFirstCoefficient(sequenceId), c0);
      xCoefficientConstants.add(numberOfConstraints, 0, bX);
      yCoefficientConstants.add(numberOfConstraints, 0, bY);
      zCoefficientConstants.add(numberOfConstraints, 0, bZ);

      numberOfConstraints++;
   }

   /**
    * Sets up the continuity constraint on the CoM position at a state change.
    * <p>
    *    The CoM position equation is as follows:
    * </p>
    * <p>
    *    x<sub>i</sub>(t) = c<sub>0,i</sub> e<sup>&omega; t</sup> + c<sub>1,i</sub> e<sup>-&omega; t</sup> + c<sub>2,i</sub> t + c<sub>3,i</sub> + r<sub>vrp,i</sub>
    * </p>
    * <p> If both the previous state and the next state are in contact, the constraint used is </p>
    * <p>
    *    c<sub>0,i-1</sub> e<sup>&omega; T<sub>i-1</sub></sup> + c<sub>1,i-1</sub> e<sup>-&omega; T<sub>i-1</sub></sup> + r<sub>vrp,i-1</sub>
    *    = c<sub>0,i</sub> e<sup>&omega; 0</sup> + c<sub>1,i</sub> e<sup>-&omega; 0</sup> + r<sub>vrp,i</sub>
    * </p>
    * <p>
    *    c<sub>2,i-1</sub> = c<sub>3,i-1</sub> = c<sub>2,i</sub> = c<sub>3,i</sub> = 0
    * </p>
    * <p> If the previous state is in contact and the next state is in flight, the constraint used is </p>
    * <p>
    *    c<sub>0,i-1</sub> e<sup>&omega; T<sub>i-1</sub></sup> + c<sub>1,i-1</sub> e<sup>-&omega; T<sub>i-1</sub></sup> + r<sub>vrp,i-1</sub>
    *    = c<sub>2,i</sub> 0 + c<sub>3,i</sub>
    * </p>
    * <p>
    *    c<sub>2,i-1</sub> = c<sub>3,i-1</sub> = c<sub>0,i</sub> = c<sub>1,i</sub> = 0
    * </p>
    * <p> If the previous state is in flight and the next state is in contact, the constraint used is </p>
    * <p>
    *    c<sub>2,i-1</sub> 0 + c<sub>3,i-1</sub>
    *       = c<sub>0,i</sub> e<sup>&omega; 0</sup> + c<sub>1,i</sub> e<sup>-&omega; 0</sup> + r<sub>vrp,i</sub>
    * </p>
    * <p>
    *    c<sub>0,i-1</sub> = c<sub>1,i-1</sub> = c<sub>2,i</sub> = c<sub>3,i</sub> = 0
    * </p>
    * @param previousSequence i-1 in the above equations.
    * @param nextSequence i in the above equations.
    */
   private void setPositionContinuity(int previousSequence, int nextSequence)
   {
      QuadrupedContactPhase previousContact = contactSequence.get(previousSequence);
      QuadrupedContactPhase nextContact = contactSequence.get(nextSequence);

      double previousDuration = previousContact.getTimeInterval().getDuration();
      double previousBX, previousBY, previousBZ;
      if (previousContact.getContactState() == ContactState.IN_CONTACT)
      {
         previousBX = -previousContact.getCopPosition().getX();
         previousBY = -previousContact.getCopPosition().getY();
         previousBZ = -(previousContact.getCopPosition().getZ() + nominalCoMHeight);
      }
      else
      {
         previousBX = 0.0;
         previousBY = 0.0;
         previousBZ = 0.0;
      }

      double previousC0 = getFirstCoefficientPositionMultiplier(previousContact.getContactState(), previousDuration);
      double previousC1 = getSecondCoefficientPositionMultiplier(previousContact.getContactState(), previousDuration);
      double previousGravityEffect = -getGravityPositionEffect(previousContact.getContactState(), previousDuration);

      double nextBX, nextBY, nextBZ;
      if (nextContact.getContactState() == ContactState.IN_CONTACT)
      {
         nextBX = nextContact.getCopPosition().getX();
         nextBY = nextContact.getCopPosition().getY();
         nextBZ = nextContact.getCopPosition().getZ() + nominalCoMHeight;
      }
      else
      {
         nextBX = 0.0;
         nextBY = 0.0;
         nextBZ = 0.0;
      }

      double nextC0 = -getFirstCoefficientPositionMultiplier(nextContact.getContactState(), 0.0);
      double nextC1 = -getSecondCoefficientPositionMultiplier(nextContact.getContactState(), 0.0);
      double nextGravityEffort = getGravityPositionEffect(nextContact.getContactState(), 0.0);

      coefficientMultipliers.set(numberOfConstraints, getFirstCoefficient(previousSequence), previousC0);
      coefficientMultipliers.set(numberOfConstraints, getSecondCoefficient(previousSequence), previousC1);
      coefficientMultipliers.set(numberOfConstraints, getFirstCoefficient(nextSequence), nextC0);
      coefficientMultipliers.set(numberOfConstraints, getSecondCoefficient(nextSequence), nextC1);
      xCoefficientConstants.add(numberOfConstraints, 0, previousBX + nextBX);
      yCoefficientConstants.add(numberOfConstraints, 0, previousBY + nextBY);
      zCoefficientConstants.add(numberOfConstraints, 0, previousBZ + nextBZ + previousGravityEffect + nextGravityEffort);

      numberOfConstraints++;
   }

   /**
    * Sets up the continuity constraint on the CoM velocity at a state change.
    * <p>
    *    The CoM velocity equation is as follows:
    * </p>
    * <p>
    *    v<sub>i</sub>(t) = &omega; c<sub>0,i</sub> e<sup>&omega; t</sup> - &omega; c<sub>1,i</sub> e<sup>-&omega; t</sup> + c<sub>2,i</sub>
    * </p>
    * <p> If both the previous state and the next state are in contact, the constraint used is </p>
    * <p>
    *    &omega; c<sub>0,i-1</sub> e<sup>&omega; T<sub>i-1</sub></sup> - &omega; c<sub>1,i-1</sub> e<sup>-&omega; T<sub>i-1</sub></sup>
    *    = &omega; c<sub>0,i</sub> e<sup>&omega; 0</sup> - &omega; c<sub>1,i</sub> e<sup>-&omega; 0</sup>
    * </p>
    * <p> If the previous state is in contact and the next state is in flight, the constraint used is </p>
    * <p>
    *    &omega; c<sub>0,i-1</sub> e<sup>&omega; T<sub>i-1</sub></sup> - &omega; c<sub>1,i-1</sub> e<sup>-&omega; T<sub>i-1</sub></sup>
    *    = c<sub>2,i</sub>
    * </p>
    * <p> If the previous state is in flight and the next state is in contact, the constraint used is </p>
    * <p>
    *    c<sub>2,i-1</sub>
    *       = &omega; c<sub>0,i</sub> e<sup>&omega; 0</sup> - &omega; c<sub>1,i</sub> e<sup>-&omega; 0</sup>
    * </p>
    * @param previousSequence i-1 in the above equations.
    * @param nextSequence i in the above equations.
    */
   private void setVelocityContinuity(int previousSequence, int nextSequence)
   {
      QuadrupedContactPhase previousContact = contactSequence.get(previousSequence);
      QuadrupedContactPhase nextContact = contactSequence.get(nextSequence);

      double previousDuration = previousContact.getTimeInterval().getDuration();

      double previousC0 = getFirstCoefficientVelocityMultiplier(previousContact.getContactState(), previousDuration);
      double previousC1 = getSecondCoefficientVelocityMultiplier(previousContact.getContactState(), previousDuration);
      double previousGravityEffect = -getGravityVelocityEffect(previousContact.getContactState(), previousDuration);

      double nextC0 = -getFirstCoefficientVelocityMultiplier(nextContact.getContactState(), 0.0);
      double nextC1 = -getSecondCoefficientVelocityMultiplier(nextContact.getContactState(), 0.0);
      double nextGravityEffect = getGravityVelocityEffect(nextContact.getContactState(), 0.0);

      coefficientMultipliers.set(numberOfConstraints, getFirstCoefficient(previousSequence), previousC0);
      coefficientMultipliers.set(numberOfConstraints, getSecondCoefficient(previousSequence), previousC1);
      coefficientMultipliers.set(numberOfConstraints, getFirstCoefficient(nextSequence), nextC0);
      coefficientMultipliers.set(numberOfConstraints, getSecondCoefficient(nextSequence), nextC1);
      xCoefficientConstants.add(numberOfConstraints, 0, 0.0);
      yCoefficientConstants.add(numberOfConstraints, 0, 0.0);
      zCoefficientConstants.add(numberOfConstraints, 0, previousGravityEffect + nextGravityEffect);

      numberOfConstraints++;
   }

   private double getFirstCoefficientPositionMultiplier(ContactState contactState, double timeInPhase)
   {
      if (contactState == ContactState.IN_CONTACT)
      {
         return Math.exp(omega.getDoubleValue() * timeInPhase);
      }
      else
      {
         return timeInPhase;
      }
   }

   private double getSecondCoefficientPositionMultiplier(ContactState contactState, double timeInPhase)
   {
      if (contactState == ContactState.IN_CONTACT)
      {
         return Math.exp(-omega.getDoubleValue() * timeInPhase);
      }
      else
      {
         return 1.0;
      }
   }

   private double getFirstCoefficientVelocityMultiplier(ContactState contactState, double timeInPhase)
   {
      if (contactState == ContactState.IN_CONTACT)
      {
         return omega.getDoubleValue() * Math.exp(omega.getDoubleValue() * timeInPhase);
      }
      else
      {
         return 1.0;
      }
   }

   private double getSecondCoefficientVelocityMultiplier(ContactState contactState, double timeInPhase)
   {
      if (contactState == ContactState.IN_CONTACT)
      {
         return -omega.getDoubleValue() * Math.exp(-omega.getDoubleValue() * timeInPhase);
      }
      else
      {
         return 0.0;
      }
   }

   private double getGravityPositionEffect(ContactState contactState, double timeInPhase)
   {
      if (contactState == ContactState.IN_CONTACT)
      {
         return 0.0;
      }
      else
      {
         return -gravityZ * timeInPhase * timeInPhase;
      }
   }

   private double getGravityVelocityEffect(ContactState contactState, double timeInPhase)
   {
      if (contactState == ContactState.IN_CONTACT)
      {
         return 0.0;
      }
      else
      {
         return -2.0 * gravityZ * timeInPhase;
      }
   }

   private int getFirstCoefficient(int sequenceId)
   {
      return 2 * sequenceId;
   }

   private int getSecondCoefficient(int sequenceId)
   {
      return 2 * sequenceId + 1;
   }

}
