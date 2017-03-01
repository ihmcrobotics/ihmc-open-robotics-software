package us.ihmc.quadrupedRobotics.planning.trajectory;

import org.apache.commons.lang3.mutable.MutableDouble;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.quadrupedRobotics.planning.QuadrupedCenterOfPressureTools;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedContactPhase;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedContactSequence;
import us.ihmc.quadrupedRobotics.util.TimeInterval;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.*;

import java.util.ArrayList;

public class QuadrupedPiecewisePolynomialCopTrajectory
{
   private class YoTimedPolynomial extends YoPolynomial
   {
      TimeInterval timeInterval;

      public YoTimedPolynomial(String name, int maxCoefficients, YoVariableRegistry registry)
      {
         super(name, maxCoefficients, registry);
         timeInterval = new TimeInterval();
      }

      public void setTimeInterval(double startTime, double endTime)
      {
         this.timeInterval.setInterval(startTime, endTime);
      }

      public TimeInterval getTimeInterval()
      {
         return timeInterval;
      }
   }

   YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BooleanYoVariable trajectoryInitialized;
   private final TimeInterval trajectoryTimeInterval;
   private final FramePoint copPositionAtCurrentTime;
   private final YoFramePoint yoCopPositionAtCurrentTime;
   private final QuadrantDependentList<FramePoint> solePositionAtCurrentTime;
   private final QuadrantDependentList<ContactState> contactStateAtCurrentTime;
   private final QuadrantDependentList<MutableDouble> contactPressureAtCurrentTime;
   private final EndDependentList<IntegerYoVariable> numberOfContactPhasesPerEnd;
   private final EndDependentList<IntegerYoVariable> numberOfPressurePolynomialsPerEnd;
   private final EndDependentList<ArrayList<QuadrupedTimedContactPhase>> contactPhasesPerEnd;
   private final EndDependentList<ArrayList<YoTimedPolynomial>> pressurePolynomialsPerEnd;
   private final DoubleYoVariable copShiftDuration;

   public QuadrupedPiecewisePolynomialCopTrajectory(int maximumNumberOfContactPhases, double copShiftDuration, YoVariableRegistry parentRegistry)
   {
      int maximumNumberOfTrajectorySegments = 3 * maximumNumberOfContactPhases;
      trajectoryInitialized = new BooleanYoVariable("copTrajectoryInitialized", registry);
      trajectoryTimeInterval = new TimeInterval();
      copPositionAtCurrentTime = new FramePoint();
      yoCopPositionAtCurrentTime = new YoFramePoint("copPositionAtCurrentTime", ReferenceFrame.getWorldFrame(), registry);
      solePositionAtCurrentTime = new QuadrantDependentList<>();
      contactStateAtCurrentTime = new QuadrantDependentList<>();
      contactPressureAtCurrentTime = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         solePositionAtCurrentTime.set(robotQuadrant, new FramePoint());
         contactStateAtCurrentTime.set(robotQuadrant, ContactState.IN_CONTACT);
         contactPressureAtCurrentTime.set(robotQuadrant, new MutableDouble(0.25));
      }
      numberOfContactPhasesPerEnd = new EndDependentList<>();
      numberOfPressurePolynomialsPerEnd = new EndDependentList<>();
      contactPhasesPerEnd = new EndDependentList<>();
      pressurePolynomialsPerEnd = new EndDependentList<>();
      for (RobotEnd robotEnd : RobotEnd.values)
      {
         String robotEndPrefix = robotEnd.getCamelCaseNameForStartOfExpression();
         String robotEndModifier = robotEnd.getCamelCaseNameForMiddleOfExpression();
         numberOfContactPhasesPerEnd.set(robotEnd, new IntegerYoVariable("numberOf" + robotEndModifier + "ContactPhases", registry));
         numberOfPressurePolynomialsPerEnd.set(robotEnd, new IntegerYoVariable("numberOf" + robotEndModifier + "PressurePolynomials", registry));
         contactPhasesPerEnd.set(robotEnd, new ArrayList<QuadrupedTimedContactPhase>(maximumNumberOfContactPhases));
         pressurePolynomialsPerEnd.set(robotEnd, new ArrayList<YoTimedPolynomial>(maximumNumberOfTrajectorySegments));
         for (int i = 0; i < maximumNumberOfContactPhases; i++)
            contactPhasesPerEnd.get(robotEnd).add(i, new QuadrupedTimedContactPhase());
         for (int i = 0; i < maximumNumberOfTrajectorySegments; i++)
            pressurePolynomialsPerEnd.get(robotEnd).add(i, new YoTimedPolynomial(robotEndPrefix + "PressureTrajectoryPolynomial" + i, 4, registry));
      }
      this.copShiftDuration = new DoubleYoVariable("copShiftDuration", registry);
      this.copShiftDuration.set(copShiftDuration);

      if (parentRegistry != null)
      {
         parentRegistry.addChild(registry);
      }
   }

   public void initializeTrajectory(QuadrupedTimedContactSequence contactSequence)
   {
      if (contactSequence.size() < 1)
      {
         throw new RuntimeException("Contact sequence must be greater than zero.");
      }
      if (contactSequence.size() > contactPhasesPerEnd.get(RobotEnd.FRONT).size())
      {
         throw new RuntimeException("Contact sequence size exceeds the maximum number of contact phases.");
      }
      trajectoryTimeInterval.setStartTime(contactSequence.get(0).getTimeInterval().getStartTime());
      trajectoryTimeInterval.setEndTime(contactSequence.get(contactSequence.size() - 1).getTimeInterval().getEndTime());

      for (RobotEnd robotEnd : RobotEnd.values)
      {
         numberOfContactPhasesPerEnd.get(robotEnd).set(0);
         numberOfPressurePolynomialsPerEnd.get(robotEnd).set(0);

         // compute end-specific contact phases for front or hind legs
         QuadrupedTimedContactPhase contactPhase = contactSequence.get(0);
         QuadrupedTimedContactPhase endContactPhase = contactPhasesPerEnd.get(robotEnd).get(0);
         endContactPhase.set(contactPhase);
         numberOfContactPhasesPerEnd.get(robotEnd).increment();
         for (int i = 1; i < contactSequence.size(); i++)
         {
            contactPhase = contactSequence.get(i);
            if (!isEqualEndContactState(robotEnd, contactPhase.getContactState(), endContactPhase.getContactState()))
            {
               endContactPhase.getTimeInterval().setEndTime(contactPhase.getTimeInterval().getStartTime());
               endContactPhase = contactPhasesPerEnd.get(robotEnd).get(numberOfContactPhasesPerEnd.get(robotEnd).getIntegerValue());
               endContactPhase.set(contactPhase);
               numberOfContactPhasesPerEnd.get(robotEnd).increment();
            }
         }
         endContactPhase.getTimeInterval().setEndTime(contactPhase.getTimeInterval().getEndTime());

         // compute constant pressure polynomial given a single end contact phase
         if (numberOfContactPhasesPerEnd.get(robotEnd).getIntegerValue() == 1)
         {
            TimeInterval timeInterval = contactPhasesPerEnd.get(robotEnd).get(0).getTimeInterval();
            QuadrantDependentList<ContactState> contactState = contactPhasesPerEnd.get(robotEnd).get(0).getContactState();
            if (isEndDoubleSupportState(robotEnd, contactState))
            {
               addConstantPressurePolynomial(robotEnd, timeInterval.getStartTime(), timeInterval.getEndTime(), 0.5);
            }
            else
            {
               addConstantPressurePolynomial(robotEnd, timeInterval.getStartTime(), timeInterval.getEndTime(),
                     getNormalizedLeftSidePressure(robotEnd, contactState));
            }
            continue;
         }

         // compute cubic pressure polynomials given multiple end contact phases
         for (int i = 0; i < numberOfContactPhasesPerEnd.get(robotEnd).getIntegerValue(); i++)
         {
            double shiftDuration = copShiftDuration.getDoubleValue();
            double startTime, endTime;
            TimeInterval timeInterval = contactPhasesPerEnd.get(robotEnd).get(i).getTimeInterval();
            QuadrantDependentList<ContactState> contactState = contactPhasesPerEnd.get(robotEnd).get(i).getContactState();
            if (isEndDoubleSupportState(robotEnd, contactState))
            {
               if (i == 0)
               {
                  // initial double support phase
                  QuadrantDependentList<ContactState> nextContactState = contactPhasesPerEnd.get(robotEnd).get(i + 1).getContactState();
                  if (timeInterval.getDuration() > shiftDuration)
                  {
                     startTime = timeInterval.getStartTime();
                     endTime = timeInterval.getEndTime() - shiftDuration;
                     addConstantPressurePolynomial(robotEnd, startTime, endTime, 0.5);
                  }
                  startTime = timeInterval.getEndTime() - shiftDuration;
                  endTime = timeInterval.getEndTime();
                  addCubicPressurePolynomial(robotEnd, startTime, endTime, 0.5, 0, getNormalizedLeftSidePressure(robotEnd, nextContactState), 0);
               }
               else if (i < numberOfContactPhasesPerEnd.get(robotEnd).getIntegerValue() - 1)
               {
                  // intermediate double support phase
                  QuadrantDependentList<ContactState> pastContactState = contactPhasesPerEnd.get(robotEnd).get(i - 1).getContactState();
                  QuadrantDependentList<ContactState> nextContactState = contactPhasesPerEnd.get(robotEnd).get(i + 1).getContactState();
                  if (timeInterval.getDuration() > 2 * shiftDuration)
                  {
                     startTime = timeInterval.getStartTime();
                     endTime = timeInterval.getStartTime() + shiftDuration;
                     addCubicPressurePolynomial(robotEnd, startTime, endTime, getNormalizedLeftSidePressure(robotEnd, pastContactState), 0, 0.5, 0);

                     startTime = timeInterval.getStartTime() + shiftDuration;
                     endTime = timeInterval.getEndTime() - shiftDuration;
                     addConstantPressurePolynomial(robotEnd, startTime, endTime, 0.5);

                     startTime = timeInterval.getEndTime() - shiftDuration;
                     endTime = timeInterval.getEndTime();
                     addCubicPressurePolynomial(robotEnd, startTime, endTime, 0.5, 0, getNormalizedLeftSidePressure(robotEnd, nextContactState), 0);
                  }
                  else
                  {
                     startTime = timeInterval.getStartTime();
                     endTime = timeInterval.getEndTime();
                     addCubicPressurePolynomial(robotEnd, startTime, endTime, getNormalizedLeftSidePressure(robotEnd, pastContactState), 0,
                           getNormalizedLeftSidePressure(robotEnd, nextContactState), 0);
                  }
               }
               else
               {
                  QuadrantDependentList<ContactState> pastContactState = contactPhasesPerEnd.get(robotEnd).get(i - 1).getContactState();
                  // final double support phase
                  startTime = timeInterval.getStartTime();
                  endTime = timeInterval.getStartTime() + shiftDuration;
                  addCubicPressurePolynomial(robotEnd, startTime, endTime, getNormalizedLeftSidePressure(robotEnd, pastContactState), 0, 0.5, 0);
                  if (timeInterval.getDuration() > shiftDuration)
                  {
                     startTime = timeInterval.getStartTime() + shiftDuration;
                     endTime = timeInterval.getEndTime();
                     addConstantPressurePolynomial(robotEnd, startTime, endTime, 0.5);
                  }
               }
            }
            else
            {
               // zero or single support phase
               startTime = timeInterval.getStartTime();
               endTime = timeInterval.getEndTime();
               addConstantPressurePolynomial(robotEnd, startTime, endTime, getNormalizedLeftSidePressure(robotEnd, contactState));
            }
         }
      }
      trajectoryInitialized.set(true);
      computeTrajectory(trajectoryTimeInterval.getStartTime());
   }

   public void computeTrajectory(double currentTime)
   {
      if (!trajectoryInitialized.getBooleanValue())
      {
         throw new RuntimeException("Trajectory must be initialized before calling computeTrajectory.");
      }
      currentTime = MathTools.clamp(currentTime, trajectoryTimeInterval.getStartTime(), trajectoryTimeInterval.getEndTime());

      for (RobotEnd robotEnd : RobotEnd.values)
      {
         int numberOfContactPhases = numberOfContactPhasesPerEnd.get(robotEnd).getIntegerValue();
         ArrayList<QuadrupedTimedContactPhase> contactPhases = contactPhasesPerEnd.get(robotEnd);
         for (int i = 0; i < numberOfContactPhases; i++)
         {
            QuadrupedTimedContactPhase contactPhase = contactPhases.get(i);
            double startTime = contactPhase.getTimeInterval().getStartTime();
            double endTime = contactPhase.getTimeInterval().getEndTime();
            if (currentTime >= startTime && currentTime <= endTime)
            {
               for (RobotSide robotSide : RobotSide.values)
               {
                  RobotQuadrant robotQuadrant = RobotQuadrant.getQuadrant(robotEnd, robotSide);
                  contactStateAtCurrentTime.set(robotQuadrant, contactPhase.getContactState().get(robotQuadrant));
                  solePositionAtCurrentTime.get(robotQuadrant).setIncludingFrame(contactPhase.getSolePosition().get(robotQuadrant));
               }
               break;
            }
         }
      }

      for (RobotEnd robotEnd : RobotEnd.values)
      {
         int numberOfPressurePolynomials = numberOfPressurePolynomialsPerEnd.get(robotEnd).getIntegerValue();
         ArrayList<YoTimedPolynomial> polynomials = pressurePolynomialsPerEnd.get(robotEnd);
         for (int i = 0; i < numberOfPressurePolynomials; i++)
         {
            YoTimedPolynomial polynomial = polynomials.get(i);
            double startTime = polynomial.getTimeInterval().getStartTime();
            double endTime = polynomial.getTimeInterval().getEndTime();
            if (currentTime >= startTime && currentTime <= endTime)
            {
               polynomial.compute(currentTime - startTime);
               double normalizedLeftSidePressure = polynomial.getPosition();
               double numberOfEndContacts = getNumberOfEndContacts(robotEnd, contactStateAtCurrentTime);
               double numberOfOppositeEndContacts = getNumberOfEndContacts(robotEnd.getOppositeEnd(), contactStateAtCurrentTime);
               if (numberOfEndContacts == 0)
               {
                  contactPressureAtCurrentTime.get(RobotQuadrant.getQuadrant(robotEnd, RobotSide.LEFT)).setValue(0.0);
                  contactPressureAtCurrentTime.get(RobotQuadrant.getQuadrant(robotEnd, RobotSide.RIGHT)).setValue(0.0);
               }
               else if (numberOfOppositeEndContacts == 0)
               {
                  contactPressureAtCurrentTime.get(RobotQuadrant.getQuadrant(robotEnd, RobotSide.LEFT)).setValue(normalizedLeftSidePressure);
                  contactPressureAtCurrentTime.get(RobotQuadrant.getQuadrant(robotEnd, RobotSide.RIGHT)).setValue(1.0 - normalizedLeftSidePressure);
               }
               else
               {
                  contactPressureAtCurrentTime.get(RobotQuadrant.getQuadrant(robotEnd, RobotSide.LEFT)).setValue(0.5 * normalizedLeftSidePressure);
                  contactPressureAtCurrentTime.get(RobotQuadrant.getQuadrant(robotEnd, RobotSide.RIGHT)).setValue(0.5 * (1.0 - normalizedLeftSidePressure));
               }
               break;
            }
         }
      }

      QuadrupedCenterOfPressureTools.computeCenterOfPressure(copPositionAtCurrentTime, solePositionAtCurrentTime, contactPressureAtCurrentTime);
      yoCopPositionAtCurrentTime.setAndMatchFrame(copPositionAtCurrentTime);
   }

   public void getPosition(FramePoint copPositionAtCurrentTime)
   {
      copPositionAtCurrentTime.setIncludingFrame(yoCopPositionAtCurrentTime.getFrameTuple());
   }

   private boolean isEqualEndContactState(RobotEnd robotEnd, QuadrantDependentList<ContactState> contactStateA,
         QuadrantDependentList<ContactState> contactStateB)
   {
      boolean isEqual = true;
      for (RobotSide robotSide : RobotSide.values)
      {
         RobotQuadrant robotQuadrant = RobotQuadrant.getQuadrant(robotEnd, robotSide);
         if (contactStateA.get(robotQuadrant) != contactStateB.get(robotQuadrant))
         {
            isEqual = false;
         }
      }
      return isEqual;
   }

   private boolean isEndDoubleSupportState(RobotEnd robotEnd, QuadrantDependentList<ContactState> contactState)
   {
      boolean isDoubleSupport = true;
      for (RobotSide robotSide : RobotSide.values)
      {
         RobotQuadrant robotQuadrant = RobotQuadrant.getQuadrant(robotEnd, robotSide);
         if (contactState.get(robotQuadrant) != ContactState.IN_CONTACT)
         {
            isDoubleSupport = false;
         }
      }
      return isDoubleSupport;
   }

   private int getNumberOfEndContacts(RobotEnd robotEnd, QuadrantDependentList<ContactState> contactState)
   {
      int numberOfEndContacts = 0;
      for (RobotSide robotSide : RobotSide.values)
      {
         RobotQuadrant robotQuadrant = RobotQuadrant.getQuadrant(robotEnd, robotSide);
         if (contactState.get(robotQuadrant) == ContactState.IN_CONTACT)
         {
            numberOfEndContacts++;
         }
      }
      return numberOfEndContacts;
   }

   private double getNormalizedLeftSidePressure(RobotEnd robotEnd, QuadrantDependentList<ContactState> contactState)
   {
      RobotQuadrant leftSideQuadrant = RobotQuadrant.getQuadrant(robotEnd, RobotSide.LEFT);
      RobotQuadrant rightSideQuadrant = RobotQuadrant.getQuadrant(robotEnd, RobotSide.RIGHT);
      if ((contactState.get(leftSideQuadrant) == ContactState.IN_CONTACT) && (contactState.get(rightSideQuadrant) == ContactState.IN_CONTACT))
         return 0.5;
      else if (contactState.get(leftSideQuadrant) == ContactState.IN_CONTACT)
         return 1.0;
      else
         return 0.0;
   }

   private void addCubicPressurePolynomial(RobotEnd robotEnd, double t0, double tFinal, double z0, double zd0, double zFinal, double zdFinal)
   {
      YoTimedPolynomial polynomial = pressurePolynomialsPerEnd.get(robotEnd).get(numberOfPressurePolynomialsPerEnd.get(robotEnd).getIntegerValue());
      polynomial.setTimeInterval(t0, tFinal);
      polynomial.setCubic(0.0, tFinal - t0, z0, zd0, zFinal, zdFinal);
      numberOfPressurePolynomialsPerEnd.get(robotEnd).increment();
   }

   private void addConstantPressurePolynomial(RobotEnd robotEnd, double t0, double tFinal, double z)
   {
      YoTimedPolynomial polynomial = pressurePolynomialsPerEnd.get(robotEnd).get(numberOfPressurePolynomialsPerEnd.get(robotEnd).getIntegerValue());
      polynomial.setTimeInterval(t0, tFinal);
      polynomial.setConstant(z);
      numberOfPressurePolynomialsPerEnd.get(robotEnd).increment();
   }
}
