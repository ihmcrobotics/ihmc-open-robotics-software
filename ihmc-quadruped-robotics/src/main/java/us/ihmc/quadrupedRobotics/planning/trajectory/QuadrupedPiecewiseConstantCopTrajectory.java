package us.ihmc.quadrupedRobotics.planning.trajectory;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.mutable.MutableBoolean;
import org.apache.commons.lang3.mutable.MutableDouble;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.quadrupedRobotics.planning.QuadrupedCenterOfPressureTools;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedContactSequence;
import us.ihmc.quadrupedRobotics.planning.WeightDistributionCalculator;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

public class QuadrupedPiecewiseConstantCopTrajectory
{
   private final QuadrantDependentList<ContactState> initialContactState;
   private final QuadrantDependentList<MutableBoolean> isInitialContactState;
   private boolean initialized;

   private final FramePoint3D copPositionAtCurrentTime;
   private int numberOfIntervals;
   private final ArrayList<MutableDouble> timeAtStartOfInterval;
   private final ArrayList<YoFramePoint3D> copPositionsAtStartOfInterval;
   private final ArrayList<QuadrantDependentList<MutableDouble>> normalizedPressureAtStartOfInterval;
   private final ArrayList<MutableDouble> normalizedPressureContributedByInitialContacts;
   private final ArrayList<MutableDouble> normalizedPressureContributedByQueuedSteps;

   private final WeightDistributionCalculator weightDistributionCalculator;

   public QuadrupedPiecewiseConstantCopTrajectory(int maxIntervals, WeightDistributionCalculator weightDistributionCalculator, YoVariableRegistry registry)
   {
      this.weightDistributionCalculator = weightDistributionCalculator;

      initialContactState = new QuadrantDependentList<>();
      isInitialContactState = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         initialContactState.set(robotQuadrant, ContactState.IN_CONTACT);
         isInitialContactState.set(robotQuadrant, new MutableBoolean(true));
      }
      initialized = false;

      copPositionAtCurrentTime = new FramePoint3D();
      numberOfIntervals = 0;
      timeAtStartOfInterval = new ArrayList<>(maxIntervals);
      copPositionsAtStartOfInterval = new ArrayList<>(maxIntervals);
      normalizedPressureContributedByInitialContacts = new ArrayList<>(maxIntervals);
      normalizedPressureContributedByQueuedSteps = new ArrayList<>(maxIntervals);
      normalizedPressureAtStartOfInterval = new ArrayList<>(maxIntervals);
      for (int i = 0; i < maxIntervals; i++)
      {
         timeAtStartOfInterval.add(i, new MutableDouble(0.0));
         copPositionsAtStartOfInterval.add(i, new YoFramePoint3D("copPositionWaypoint" + i, ReferenceFrame.getWorldFrame(), registry));

         normalizedPressureContributedByInitialContacts.add(i, new MutableDouble(0.0));
         normalizedPressureContributedByQueuedSteps.add(i, new MutableDouble(0.0));
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
      for (int i = 0; i < copPositionsAtStartOfInterval.size(); i ++)
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
         isInitialContactState.get(robotQuadrant).setValue(true);
      }

      numberOfIntervals = timedContactSequence.size();
      for (int interval = 0; interval < numberOfIntervals; interval++)
      {
         QuadrantDependentList<FramePoint3D> solePosition = timedContactSequence.get(interval).getSolePosition();
         QuadrantDependentList<ContactState> contactState = timedContactSequence.get(interval).getContactState();

         QuadrupedCenterOfPressureTools.computeNominalNormalizedContactPressure(normalizedPressureAtStartOfInterval.get(interval), contactState, solePosition, weightDistributionCalculator);
         QuadrupedCenterOfPressureTools.computeCenterOfPressure(copPositionsAtStartOfInterval.get(interval), solePosition, normalizedPressureAtStartOfInterval.get(interval));



         normalizedPressureContributedByQueuedSteps.get(interval).setValue(0.0);
         normalizedPressureContributedByInitialContacts.get(interval).setValue(0.0);
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            if (contactState.get(robotQuadrant) != initialContactState.get(robotQuadrant))
            {
               isInitialContactState.get(robotQuadrant).setValue(false);
            }
            if (isInitialContactState.get(robotQuadrant).booleanValue())
            {
               normalizedPressureContributedByInitialContacts.get(interval)
                     .add(normalizedPressureAtStartOfInterval.get(interval).get(robotQuadrant).doubleValue());
            }
            else
            {
               normalizedPressureContributedByQueuedSteps.get(interval).add(normalizedPressureAtStartOfInterval.get(interval).get(robotQuadrant).doubleValue());
            }
         }
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

   private void

   private void computeCoPOffset(QuadrantDependentList<ContactState> contactState, QuadrantDependentList<FramePoint3D> solePositions, FrameVector3D copOffsetToPack)
   {
      int numberOfLeftFeetInContact = 0;
      int numberOfRightFeetInContact = 0;
      int numberOfFrontFeetInContact = 0;
      int numberOfHindFeetInContact = 0;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (contactState.get(robotQuadrant) == ContactState.IN_CONTACT)
         {
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

      if (numberOfLeftFeetInContact > 0 && numberOfRightFeetInContact > 0)
      {
         if (numberOfLeftFeetInContact > numberOfRightFeetInContact)
         { // left side has more feet in contact, so shift a little to the right

         }
         else if (numberOfRightFeetInContact > numberOfLeftFeetInContact)
         { // right side has more feet in contact, so shift a little to the left

         }
      }
   }
}