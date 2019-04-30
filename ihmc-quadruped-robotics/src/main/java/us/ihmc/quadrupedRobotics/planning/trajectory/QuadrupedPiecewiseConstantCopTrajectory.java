package us.ihmc.quadrupedRobotics.planning.trajectory;

import org.apache.commons.lang3.mutable.MutableDouble;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedBasics.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.quadrupedRobotics.planning.QuadrupedCenterOfPressureTools;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedContactSequence;
import us.ihmc.quadrupedRobotics.planning.WeightDistributionCalculator;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

import java.util.ArrayList;
import java.util.List;

public class QuadrupedPiecewiseConstantCopTrajectory
{
   private static final double stanceWidthCoPShiftFactor = 0.2;
   private static final double stanceLengthCoPShiftFactor = 0.0;
   private static final double maxStanceWidthCoPShift = 0.1;
   private static final double maxStanceLengthCoPShift = 0.1;
   private static final double stepWidthCoPShiftFactor = 0.1;
   private static final double stepLengthCoPShiftFactor = 0.2;
   private static final double maxStepWidthCoPShift = 0.1;
   private static final double maxStepLengthCoPShift = 0.1;

   private static final double safeDistanceFromSupportPolygonEdges = 0.04;

   private final QuadrantDependentList<ContactState> initialContactState;
   private boolean initialized;

   private final FramePoint3D copPositionAtCurrentTime;
   private int numberOfIntervals;
   private final ArrayList<MutableDouble> timeAtStartOfInterval;
   private final ArrayList<YoFramePoint3D> copPositionsAtStartOfInterval;
   private final ArrayList<QuadrantDependentList<MutableDouble>> normalizedPressureAtStartOfInterval;

   private final WeightDistributionCalculator weightDistributionCalculator;

   public QuadrupedPiecewiseConstantCopTrajectory(int maxIntervals, WeightDistributionCalculator weightDistributionCalculator, YoVariableRegistry registry)
   {
      this.weightDistributionCalculator = weightDistributionCalculator;

      initialContactState = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         initialContactState.set(robotQuadrant, ContactState.IN_CONTACT);
      }
      initialized = false;

      copPositionAtCurrentTime = new FramePoint3D();
      numberOfIntervals = 0;
      timeAtStartOfInterval = new ArrayList<>(maxIntervals);
      copPositionsAtStartOfInterval = new ArrayList<>(maxIntervals);
      normalizedPressureAtStartOfInterval = new ArrayList<>(maxIntervals);
      for (int i = 0; i < maxIntervals; i++)
      {
         timeAtStartOfInterval.add(i, new MutableDouble(0.0));
         copPositionsAtStartOfInterval.add(i, new YoFramePoint3D("copPositionWaypoint" + i, ReferenceFrame.getWorldFrame(), registry));

         normalizedPressureAtStartOfInterval.add(i, new QuadrantDependentList<MutableDouble>());
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            normalizedPressureAtStartOfInterval.get(i).set(robotQuadrant, new MutableDouble(0.0));
         }
      }
      resetVariables();
   }

   public void resetVariables()
   {
      for (int i = 0; i < copPositionsAtStartOfInterval.size(); i++)
         copPositionsAtStartOfInterval.get(i).setToNaN();
   }

   public void setupVisualizers(YoGraphicsList graphicsList, ArtifactList artifactList, double pointSize)
   {
      for (int i = 0; i < copPositionsAtStartOfInterval.size(); i++)
      {
         YoFramePoint3D copLocation = copPositionsAtStartOfInterval.get(i);
         YoGraphicPosition yoGraphicPosition = new YoGraphicPosition(copLocation.getNamePrefix(), copLocation, pointSize, YoAppearance.Green(),
                                                                     YoGraphicPosition.GraphicType.BALL_WITH_CROSS);
         graphicsList.add(yoGraphicPosition);
         artifactList.add(yoGraphicPosition.createArtifact());
      }
   }

   private final FrameVector3D copOffsetFromStance = new FrameVector3D();
   private final FrameVector3D copOffsetFromSteps = new FrameVector3D();

   /**
    * compute piecewise constant center of pressure plan given the upcoming contact states
    * @param timedContactSequence contact sequence (input)
    */
   public void initializeTrajectory(double currentTime, QuadrupedTimedContactSequence timedContactSequence, List<QuadrupedTimedStep> stepSequence)
   {
      if (timedContactSequence.size() < 1)
      {
         throw new RuntimeException("Input contact sequence must have at least one time interval.");
      }

      resetVariables();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         initialContactState.set(robotQuadrant, timedContactSequence.get(0).getContactState().get(robotQuadrant));
      }

      numberOfIntervals = timedContactSequence.size();
      for (int interval = 0; interval < numberOfIntervals; interval++)
      {
         QuadrantDependentList<FramePoint3D> solePosition = timedContactSequence.get(interval).getSolePosition();
         QuadrantDependentList<ContactState> contactState = timedContactSequence.get(interval).getContactState();
         FixedFramePoint3DBasics copPositionAtStartOfInterval = copPositionsAtStartOfInterval.get(interval);

         QuadrupedCenterOfPressureTools.computeNominalNormalizedContactPressure(normalizedPressureAtStartOfInterval.get(interval), contactState, solePosition,
                                                                                weightDistributionCalculator);
         QuadrupedCenterOfPressureTools.computeCenterOfPressure(copPositionAtStartOfInterval, solePosition, normalizedPressureAtStartOfInterval.get(interval));

         computeCoPOffsetFromStance(contactState, solePosition, copOffsetFromStance);
         computeCoPOffsetFromSteps(currentTime, contactState, solePosition, stepSequence, copOffsetFromSteps);

         copPositionAtStartOfInterval.add(copOffsetFromStance);
         copPositionAtStartOfInterval.add(copOffsetFromSteps);

         constrainToPolygon(contactState, solePosition, copPositionAtStartOfInterval);

         timeAtStartOfInterval.get(interval).setValue(timedContactSequence.get(interval).getTimeInterval().getStartTime());
      }

      initialized = true;
      computeTrajectory(timeAtStartOfInterval.get(0).doubleValue());
   }

   public void computeTrajectory(double currentTime)
   {
      if (!initialized)
      {
         throw new RuntimeException("trajectory must be initialized before calling computeTrajectory");
      }

      for (int interval = numberOfIntervals - 1; interval >= 0; interval--)
      {
         if (currentTime >= timeAtStartOfInterval.get(interval).doubleValue())
         {
            copPositionAtCurrentTime.setIncludingFrame(copPositionsAtStartOfInterval.get(interval));
            return;
         }
      }
      copPositionAtCurrentTime.setIncludingFrame(copPositionsAtStartOfInterval.get(0));
   }

   public void getPosition(FramePoint3D copPositionAtCurrentTime)
   {
      copPositionAtCurrentTime.setIncludingFrame(this.copPositionAtCurrentTime);
   }

   public int getNumberOfIntervals()
   {
      return numberOfIntervals;
   }

   public double getTimeAtStartOfInterval(int interval)
   {
      return timeAtStartOfInterval.get(interval).doubleValue();
   }

   public FixedFramePoint3DBasics getCopPositionAtStartOfInterval(int interval)
   {
      return copPositionsAtStartOfInterval.get(interval);
   }

   public ArrayList<MutableDouble> getTimeAtStartOfInterval()
   {
      return timeAtStartOfInterval;
   }

   public ArrayList<? extends FixedFramePoint3DBasics> getCopPositionsAtStartOfInterval()
   {
      return copPositionsAtStartOfInterval;
   }

   private final Quaternion nominalOrientation = new Quaternion();

   private final FramePoint3D smallSideContact = new FramePoint3D();
   private final FramePoint3D smallEndContact = new FramePoint3D();
   private final FramePoint3D tempBigSideContact = new FramePoint3D();
   private final FramePoint3D tempBigEndContact = new FramePoint3D();

   private final PoseReferenceFrame stepFrame = new PoseReferenceFrame("stepFrame", ReferenceFrame.getWorldFrame());


   private void computeCoPOffsetFromStance(QuadrantDependentList<ContactState> contactState, QuadrantDependentList<FramePoint3D> solePositions,
                                           FrameVector3D copOffsetToPack)
   {
      int numberOfLeftFeetInContact = 0;
      int numberOfRightFeetInContact = 0;
      int numberOfFrontFeetInContact = 0;
      int numberOfHindFeetInContact = 0;
      int numberOfFeetInContact = 0;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (contactState.get(robotQuadrant) == ContactState.IN_CONTACT)
         {
            numberOfFeetInContact++;
            if (robotQuadrant.isQuadrantInFront())
               numberOfFrontFeetInContact++;
            else
               numberOfHindFeetInContact++;
            if (robotQuadrant.isQuadrantOnLeftSide())
               numberOfLeftFeetInContact++;
            else
               numberOfRightFeetInContact++;
         }
      }

      if (numberOfFeetInContact != 3)
         return;

      RobotSide smallSide;
      RobotEnd smallEnd;

      if (numberOfLeftFeetInContact > numberOfRightFeetInContact)
         smallSide = RobotSide.RIGHT; // left side has more feet in contact
      else
         smallSide = RobotSide.LEFT;

      if (numberOfFrontFeetInContact > numberOfHindFeetInContact)
         smallEnd = RobotEnd.HIND; // front end has more feet in contact
      else
         smallEnd = RobotEnd.FRONT;

      RobotSide bigSide = smallSide.getOppositeSide();
      RobotEnd bigEnd = smallEnd.getOppositeEnd();

      RobotQuadrant smallEndQuadrant = RobotQuadrant.getQuadrant(smallEnd, bigSide);
      RobotQuadrant smallSideQuadrant = RobotQuadrant.getQuadrant(bigEnd, smallSide);

      double nominalYaw = getNominalYawForStance(contactState, solePositions);
      nominalOrientation.setToYawQuaternion(nominalYaw);

      smallSideContact.setIncludingFrame(solePositions.get(smallSideQuadrant));
      smallEndContact.setIncludingFrame(solePositions.get(smallEndQuadrant));

//      nominalOrientation.transform(smallSideContact);
//      nominalOrientation.transform(smallEndContact);

      stepFrame.setPoseAndUpdate(smallEndContact, nominalOrientation);

      double averageLength = 0.0;
      for (RobotSide robotSide : RobotSide.values)
      {
         tempBigEndContact.setIncludingFrame(solePositions.get(RobotQuadrant.getQuadrant(bigEnd, robotSide)));
         tempBigSideContact.changeFrame(stepFrame);
         averageLength += 0.5 * Math.abs(tempBigEndContact.getX());
      }

      stepFrame.setPoseAndUpdate(smallSideContact, nominalOrientation);

      double averageWidth = 0.0;
      for (RobotEnd robotEnd : RobotEnd.values)
      {
         tempBigSideContact.setIncludingFrame(solePositions.get(RobotQuadrant.getQuadrant(robotEnd, bigSide)));
         tempBigSideContact.changeFrame(stepFrame);
         averageWidth += 0.5 * Math.abs(tempBigSideContact.getY());
      }

      double lengthShift = smallEnd.negateIfHindEnd(MathTools.clamp(stanceLengthCoPShiftFactor * averageLength, maxStanceLengthCoPShift));
      double widthShift = smallSide.negateIfRightSide(MathTools.clamp(stanceWidthCoPShiftFactor * averageWidth, maxStanceWidthCoPShift));
      copOffsetToPack.set(lengthShift, widthShift, 0.0);
      nominalOrientation.inverseTransform(copOffsetToPack);
   }

   private final FramePoint3D goalPosition = new FramePoint3D();
   private final List<QuadrupedTimedStep> nextSteps = new ArrayList<>();


   private void computeCoPOffsetFromSteps(double currentTime, QuadrantDependentList<ContactState> contactState, QuadrantDependentList<FramePoint3D> solePositions,
                                          List<QuadrupedTimedStep> stepSequence, FrameVector3D copOffsetToPack)
   {
      int numberOfFeetInContact = 0;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (contactState.get(robotQuadrant).isLoadBearing())
            numberOfFeetInContact++;
      }

      if ( numberOfFeetInContact < 3)
         return;

      double nextStepTime = Double.NaN;
      nextSteps.clear();
      for (int i = 0; i < stepSequence.size(); i++)
      {
         QuadrupedTimedStep step = stepSequence.get(i);
         if (step.getTimeInterval().intervalContains(currentTime))
            continue;

         if (step.getTimeInterval().getStartTime() > currentTime)
         {
            if (Double.isNaN(nextStepTime))
            {
               nextSteps.add(step);
               nextStepTime = step.getTimeInterval().getStartTime();
            }
            else if (MathTools.epsilonEquals(nextStepTime, step.getTimeInterval().getStartTime(), 1e-4))
            {
               nextSteps.add(step);
            }
            else
            {
               break;
            }
         }
      }

      if (nextSteps.isEmpty())
         return;

      double averageLength = 0.0;
      double averageWidth = 0.0;
      double nominalYaw = getNominalYawForStance(contactState, solePositions);
      nominalOrientation.setToYawQuaternion(nominalYaw);

      for (int i = 0; i < nextSteps.size(); i++)
      {
         QuadrupedTimedStep nextStep = nextSteps.get(i);

         stepFrame.setPoseAndUpdate(solePositions.get(nextStep.getRobotQuadrant()), nominalOrientation);

         goalPosition.setIncludingFrame(ReferenceFrame.getWorldFrame(), nextStep.getGoalPosition());
         goalPosition.changeFrame(stepFrame);

         averageLength += goalPosition.getX();
         averageWidth += goalPosition.getY();
      }

      averageLength /= nextSteps.size();
      averageWidth /= nextSteps.size();

      double lengthShift = MathTools.clamp(stepLengthCoPShiftFactor * averageLength, maxStepLengthCoPShift);
      double widthShift = MathTools.clamp(stepWidthCoPShiftFactor * averageWidth, maxStepWidthCoPShift);

      copOffsetToPack.setToZero(stepFrame);
      copOffsetToPack.set(lengthShift, widthShift, 0.0);
      copOffsetToPack.changeFrame(ReferenceFrame.getWorldFrame());
   }

   private final FramePoint2D tempFramePoint2D = new FramePoint2D();
   private final ConvexPolygon2D constraintPolygon = new ConvexPolygon2D();
   private final ConvexPolygon2D scaledConstraintPolygon = new ConvexPolygon2D();
   private final ConvexPolygonScaler polygonScaler = new ConvexPolygonScaler();

   private void constrainToPolygon(QuadrantDependentList<ContactState> contactState, QuadrantDependentList<FramePoint3D> solePositions,
                                   FixedFramePoint3DBasics copToConstraint)
   {
      constraintPolygon.clear();
      int numberOfFeetInContact = 0;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (contactState.get(robotQuadrant).isLoadBearing())
         {
            numberOfFeetInContact++;
            constraintPolygon.addVertex(solePositions.get(robotQuadrant));
         }
      }
      constraintPolygon.update();

      tempFramePoint2D.set(copToConstraint);

      // don't need to do anything if it's already inside
      if (constraintPolygon.signedDistance(tempFramePoint2D) <= -safeDistanceFromSupportPolygonEdges)
         return;

      if (numberOfFeetInContact > 2)
      {
         polygonScaler.scaleConvexPolygon(constraintPolygon, safeDistanceFromSupportPolygonEdges, scaledConstraintPolygon);
         scaledConstraintPolygon.orthogonalProjection(tempFramePoint2D);
         copToConstraint.set(tempFramePoint2D);
      }
      else
      {
         constraintPolygon.orthogonalProjection(tempFramePoint2D);
         copToConstraint.set(tempFramePoint2D);
      }
   }

   private final QuadrantDependentList<FramePoint3DReadOnly> tempList = new QuadrantDependentList<>();

   private double getNominalYawForStance(QuadrantDependentList<ContactState> contactState, QuadrantDependentList<FramePoint3D> solePositions)
   {
      tempList.clear();
      int numberOfVertices = 0;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (contactState.get(robotQuadrant).isLoadBearing())
         {
            tempList.put(robotQuadrant, solePositions.get(robotQuadrant));
            numberOfVertices++;
         }
      }
      return QuadrupedSupportPolygon.getNominalYaw(tempList, numberOfVertices);
   }
}