package us.ihmc.quadrupedRobotics.planning.trajectory;

import java.awt.*;
import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.mutable.MutableBoolean;
import org.apache.commons.lang3.mutable.MutableDouble;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.QuadrupedSupportPolygons;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
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
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

public class QuadrupedPiecewiseConstantCopTrajectory
{
   private static final double stanceWidthCoPShiftFactor = 0.1;
   private static final double stanceLengthCoPShiftFactor = 0.0;
   private static final double maxStanceWidthCoPShift = 0.1;
   private static final double maxStanceLengthCoPShift = 0.1;

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

   /**
    * compute piecewise constant center of pressure plan given the upcoming contact states
    * @param timedContactSequence contact sequence (input)
    */
   public void initializeTrajectory(QuadrupedTimedContactSequence timedContactSequence, List<QuadrupedTimedStep> stepSequence)
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

         QuadrupedCenterOfPressureTools.computeNominalNormalizedContactPressure(normalizedPressureAtStartOfInterval.get(interval), contactState, solePosition,
                                                                                weightDistributionCalculator);
         QuadrupedCenterOfPressureTools
               .computeCenterOfPressure(copPositionsAtStartOfInterval.get(interval), solePosition, normalizedPressureAtStartOfInterval.get(interval));
         computeCoPOffsetFromStance(contactState, solePosition, copOffsetFromStance);
         copPositionsAtStartOfInterval.get(interval).add(copOffsetFromStance);

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

      nominalOrientation.transform(smallSideContact);
      nominalOrientation.transform(smallEndContact);

      double averageLength = 0.0;
      for (RobotSide robotSide : RobotSide.values)
      {
         tempBigEndContact.setIncludingFrame(solePositions.get(RobotQuadrant.getQuadrant(bigEnd, robotSide)));
         nominalOrientation.transform(tempBigEndContact);
         averageLength += 0.5 * Math.abs(tempBigEndContact.getX() - smallEndContact.getX());
      }

      double averageWidth = 0.0;
      for (RobotEnd robotEnd : RobotEnd.values)
      {
         tempBigSideContact.setIncludingFrame(solePositions.get(RobotQuadrant.getQuadrant(robotEnd, bigSide)));
         nominalOrientation.transform(tempBigSideContact);
         averageWidth += 0.5 * Math.abs(tempBigSideContact.getY() - smallSideContact.getY());
      }

      double lengthShift = smallEnd.negateIfHindEnd(MathTools.clamp(stanceLengthCoPShiftFactor * averageLength, maxStanceLengthCoPShift));
      double widthShift = smallSide.negateIfRightSide(MathTools.clamp(stanceWidthCoPShiftFactor * averageWidth, maxStanceWidthCoPShift));
      copOffsetToPack.set(lengthShift, widthShift, 0.0);
      nominalOrientation.inverseTransform(copOffsetToPack);
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