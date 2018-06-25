package us.ihmc.quadrupedRobotics.planning.icp;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint2D;
import us.ihmc.yoVariables.variable.YoFrameVector2D;

import java.util.List;

public class CoMTrajectoryPlanner
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DenseMatrix64F coefficientMultipliers = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F xCoefficientConstants = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F yCoefficientConstants = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F xCoefficientVector = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F yCoefficientVector = new DenseMatrix64F(0, 1);

   private final FramePoint2D currentCoMPosition = new FramePoint2D();

   private final LinearSolver<DenseMatrix64F> solver = LinearSolverFactory.linear(0);

   private final YoDouble omega;

   private final List<QuadrupedContactPhase> contactSequence;

   private final YoFramePoint2D desiredCoMPosition = new YoFramePoint2D("desiredCoMPosition", worldFrame, registry);
   private final YoFrameVector2D desiredCoMVelocity = new YoFrameVector2D("desiredComVelocity", worldFrame, registry);
   private final YoFramePoint2D desiredICPPosition = new YoFramePoint2D("desiredICPPosition", worldFrame, registry);
   private final YoFrameVector2D desiredICPVelocity = new YoFrameVector2D("desiredICPVelocity", worldFrame, registry);

   private final YoFramePoint2D firstCoefficient = new YoFramePoint2D("comFirstCoefficient", worldFrame, registry);
   private final YoFramePoint2D secondCoefficient = new YoFramePoint2D("comSecondCoefficient", worldFrame, registry);

   public CoMTrajectoryPlanner(List<QuadrupedContactPhase> contactSequence, YoDouble omega, YoVariableRegistry parentRegistry)
   {
      this.contactSequence = contactSequence;
      this.omega = omega;

      parentRegistry.addChild(registry);
   }

   private int numberOfConstraints = 0;

   public void solveForTrajectory()
   {
      int size = 2 * contactSequence.size();
      coefficientMultipliers.reshape(size, size);
      xCoefficientConstants.reshape(size, 1);
      yCoefficientConstants.reshape(size, 1);
      xCoefficientVector.reshape(size, 1);
      yCoefficientVector.reshape(size, 1);

      int numberOfPhases = contactSequence.size();
      int numberOfTransitions = numberOfPhases - 1;

      numberOfConstraints = 0;

      // set initial constraint
      setPositionEqualityInContact(0, 0.0, currentCoMPosition);

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
      setCapturePointTerminalConstraint(numberOfPhases - 1, lastContactPhase.getCopPosition());

      // solve for coefficients
      solver.setA(coefficientMultipliers);
      solver.solve(xCoefficientConstants, xCoefficientVector);
      solver.solve(yCoefficientConstants, yCoefficientVector);

      firstCoefficient.setX(xCoefficientVector.get(getFirstCoefficient(0)));
      firstCoefficient.setY(yCoefficientVector.get(getFirstCoefficient(0)));

      secondCoefficient.setX(xCoefficientVector.get(getSecondCoefficient(0)));
      secondCoefficient.setY(yCoefficientVector.get(getSecondCoefficient(0)));
   }


   public void compute(double timeInPhase)
   {
      QuadrupedContactPhase currentContactPhase = contactSequence.get(0);

      ContactState contactState = currentContactPhase.getContactState();
      double firstCoefficientPositionMultiplier = getFirstCoefficientPositionMultiplier(contactState, timeInPhase);
      double secondCoefficientPositionMultiplier = getSecondCoefficientPositionMultiplier(contactState, timeInPhase);

      double firstCoefficientVelocityMultiplier = getFirstCoefficientVelocityMultiplier(contactState, timeInPhase);
      double secondCoefficientVelocityMultiplier = getSecondCoefficientVelocityMultiplier(contactState, timeInPhase);

      if (contactState == ContactState.IN_CONTACT)
         desiredCoMPosition.set(currentContactPhase.getCopPosition());
      else
         desiredCoMPosition.setToZero();
      desiredCoMPosition.scaleAdd(firstCoefficientPositionMultiplier, firstCoefficient, desiredCoMPosition);
      desiredCoMPosition.scaleAdd(secondCoefficientPositionMultiplier, secondCoefficient, desiredCoMPosition);

      desiredCoMVelocity.setToZero();
      desiredCoMVelocity.scaleAdd(firstCoefficientVelocityMultiplier, firstCoefficient, desiredCoMVelocity);
      desiredCoMVelocity.scaleAdd(secondCoefficientVelocityMultiplier, secondCoefficient, desiredCoMVelocity);

      desiredICPPosition.scaleAdd(1.0 / omega.getDoubleValue(), desiredCoMVelocity, desiredCoMPosition);

      desiredICPVelocity.set(firstCoefficient);
      desiredICPVelocity.scale(2.0 * firstCoefficientVelocityMultiplier);
   }

   public void setCurrentCoMPosition(FramePoint2DReadOnly currentCoMPosition)
   {
      this.currentCoMPosition.setIncludingFrame(currentCoMPosition);
   }

   public FramePoint2DReadOnly getDesiredICPPosition()
   {
      return desiredICPPosition;
   }

   public FrameVector2DReadOnly getDesiredICPVelocity()
   {
      return desiredICPVelocity;
   }

   public FramePoint2DReadOnly getDesiredCoMPosition()
   {
      return desiredCoMPosition;
   }


   private void setPositionEqualityInContact(int sequenceId, double timeInPhase, FramePoint2DReadOnly point)
   {
      point.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      double omega = this.omega.getDoubleValue();

      double c0, c1, bX, bY;
      if (timeInPhase > 0)
      {
         c0 = Math.exp(omega * timeInPhase);
         c1 = Math.exp(-omega * timeInPhase);
      }
      else
      {
         c0 = 1.0;
         c1 = 1.0;
      }
      bX = point.getX() - contactSequence.get(sequenceId).getCopPosition().getX();
      bY = point.getY() - contactSequence.get(sequenceId).getCopPosition().getY();

      coefficientMultipliers.set(numberOfConstraints, getFirstCoefficient(sequenceId), c0);
      coefficientMultipliers.set(numberOfConstraints, getSecondCoefficient(sequenceId), c1);
      xCoefficientConstants.add(numberOfConstraints, 0, bX);
      yCoefficientConstants.add(numberOfConstraints, 0, bY);

      numberOfConstraints++;
   }

   private void setCapturePointTerminalConstraint(int sequenceId, FramePoint3DReadOnly point)
   {
      point.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      double omega = this.omega.getDoubleValue();

      double duration = contactSequence.get(sequenceId).getTimeInterval().getDuration();

      double c0 = 2.0 * Math.exp(omega * duration);
      double bX = point.getX() - contactSequence.get(sequenceId).getCopPosition().getX();
      double bY = point.getY() - contactSequence.get(sequenceId).getCopPosition().getY();

      coefficientMultipliers.set(numberOfConstraints, getFirstCoefficient(sequenceId), c0);
      xCoefficientConstants.add(numberOfConstraints, 0, bX);
      yCoefficientConstants.add(numberOfConstraints, 0, bY);

      numberOfConstraints++;
   }

   private void setPositionContinuity(int previousSequence, int nextSequence)
   {
      double omega = this.omega.getDoubleValue();

      double previousC0, previousC1, previousBX, previousBY;
      double nextC0, nextC1, nextBX, nextBY;

      QuadrupedContactPhase previousContact = contactSequence.get(previousSequence);
      QuadrupedContactPhase nextContact = contactSequence.get(nextSequence);

      double previousDuration = previousContact.getTimeInterval().getDuration();

      if (previousContact.getContactState() == ContactState.IN_CONTACT)
      {
         previousC0 = Math.exp(omega * previousDuration);
         previousC1 = Math.exp(-omega * previousDuration);
         previousBX = -previousContact.getCopPosition().getX();
         previousBY = -previousContact.getCopPosition().getY();
      }
      else
      {
         previousC0 = previousDuration;
         previousC1 = 1.0;
         previousBX = 0.0;
         previousBY = 0.0;
      }

      previousC0 = getFirstCoefficientPositionMultiplier(previousContact.getContactState(), previousDuration);
      previousC1 = getSecondCoefficientPositionMultiplier(previousContact.getContactState(), previousDuration);

      if (nextContact.getContactState() == ContactState.IN_CONTACT)
      {
         nextC0 = -1.0;
         nextC1 = -1.0;
         nextBX = nextContact.getCopPosition().getX();
         nextBY = nextContact.getCopPosition().getY();
      }
      else
      {
         nextC0 = 0.0;
         nextC1 = -1.0;
         nextBX = 0.0;
         nextBY = 0.0;
      }

      nextC0 = -getFirstCoefficientPositionMultiplier(nextContact.getContactState(), 0.0);
      nextC1 = -getSecondCoefficientPositionMultiplier(nextContact.getContactState(), 0.0);

      coefficientMultipliers.set(numberOfConstraints, getFirstCoefficient(previousSequence), previousC0);
      coefficientMultipliers.set(numberOfConstraints, getSecondCoefficient(previousSequence), previousC1);
      coefficientMultipliers.set(numberOfConstraints, getFirstCoefficient(nextSequence), nextC0);
      coefficientMultipliers.set(numberOfConstraints, getSecondCoefficient(nextSequence), nextC1);
      xCoefficientConstants.add(numberOfConstraints, 0, previousBX + nextBX);
      yCoefficientConstants.add(numberOfConstraints, 0, previousBY + nextBY);

      numberOfConstraints++;
   }

   private void setVelocityContinuity(int previousSequence, int nextSequence)
   {
      QuadrupedContactPhase previousContact = contactSequence.get(previousSequence);
      QuadrupedContactPhase nextContact = contactSequence.get(nextSequence);

      double previousDuration = previousContact.getTimeInterval().getDuration();

      double previousC0 = getFirstCoefficientVelocityMultiplier(previousContact.getContactState(), previousDuration);
      double previousC1 = getSecondCoefficientVelocityMultiplier(previousContact.getContactState(), previousDuration);

      /*
      double previousC0, previousC1;
      if (previousContact.getContactState() == ContactState.IN_CONTACT)
      {
         previousC0 = omega * Math.exp(omega * previousDuration);
         previousC1 = -omega * Math.exp(-omega * previousDuration);
      }
      else
      {
         previousC0 = 1.0;
         previousC1 = 0.0;
      }
      */

      double nextC0 = -getFirstCoefficientVelocityMultiplier(nextContact.getContactState(), 0.0);
      double nextC1 = -getSecondCoefficientVelocityMultiplier(nextContact.getContactState(), 0.0);

      /*
      double nextC0, nextC1;
      if (nextContact.getContactState() == ContactState.IN_CONTACT)
      {
         nextC0 = -omega;
         nextC1 = omega;
      }
      else
      {
         nextC0 = -1.0;
         nextC1 = 0.0;
      }
      */

      coefficientMultipliers.set(numberOfConstraints, getFirstCoefficient(previousSequence), previousC0);
      coefficientMultipliers.set(numberOfConstraints, getSecondCoefficient(previousSequence), previousC1);
      coefficientMultipliers.set(numberOfConstraints, getFirstCoefficient(nextSequence), nextC0);
      coefficientMultipliers.set(numberOfConstraints, getSecondCoefficient(nextSequence), nextC1);
      xCoefficientConstants.add(numberOfConstraints, 0, 0.0);
      yCoefficientConstants.add(numberOfConstraints, 0, 0.0);

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

   private int getFirstCoefficient(int sequenceId)
   {
      return 2 * sequenceId;
   }

   private int getSecondCoefficient(int sequenceId)
   {
      return 2 * sequenceId + 1;
   }

}
