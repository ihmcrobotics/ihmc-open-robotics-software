package us.ihmc.quadrupedRobotics.planning.icp;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.time.TimeIntervalTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

import java.util.ArrayList;
import java.util.List;

/**
 * This class computes a list of {@link ContactStateProvider} from a list of {@link QuadrupedTimedStep}. This uses transition trajectories from the
 * start CoP to the end CoP.
 */
public class QuadrupedContactSequenceUpdater
{
   private static final boolean VISUALIZE = true;
   private static final double POINT_SIZE = 0.005;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final RecyclingArrayList<QuadrupedStepTransition> stepTransitionsInAbsoluteTime = new RecyclingArrayList<>(QuadrupedStepTransition::new);
   private final List<RobotQuadrant> feetInContact = new ArrayList<>();
   private final QuadrantDependentList<FramePoint3D> solePositions = new QuadrantDependentList<>();
   private final QuadrantDependentList<MovingReferenceFrame> soleFrames;

   private final int maxCapacity;

   private final RecyclingArrayList<QuadrupedContactPhase> contactSequenceInRelativeTime;
   private final RecyclingArrayList<QuadrupedContactPhase> contactSequenceInAbsoluteTime;

   private final List<YoFramePoint3D> copKnots = new ArrayList<>();

   public QuadrupedContactSequenceUpdater(QuadrantDependentList<MovingReferenceFrame> soleFrames, int pastContactPhaseCapacity, int futureContactPhaseCapacity)
   {
      this(soleFrames, pastContactPhaseCapacity, futureContactPhaseCapacity, null, null);
   }

   public QuadrupedContactSequenceUpdater(QuadrantDependentList<MovingReferenceFrame> soleFrames, int pastContactPhaseCapacity, int futureContactPhaseCapacity,
                                          YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      contactSequenceInRelativeTime = new RecyclingArrayList<>(pastContactPhaseCapacity + futureContactPhaseCapacity + 1, QuadrupedContactPhase::new);
      contactSequenceInAbsoluteTime = new RecyclingArrayList<>(pastContactPhaseCapacity + futureContactPhaseCapacity + 1, QuadrupedContactPhase::new);
      contactSequenceInRelativeTime.clear();
      contactSequenceInAbsoluteTime.clear();

      this.soleFrames = soleFrames;

      maxCapacity = pastContactPhaseCapacity + futureContactPhaseCapacity + 1;

      if (maxCapacity < 1)
      {
         throw new RuntimeException("Sequence capacity must be greater than zero.");
      }

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         feetInContact.add(robotQuadrant);
         solePositions.set(robotQuadrant, new FramePoint3D());
      }

      String packageName = "contactPlanner";
      YoGraphicsList graphicsList = new YoGraphicsList(packageName);
      ArtifactList artifactList = new ArtifactList(packageName);

      for (int i = 0; i < maxCapacity; i++)
      {
         YoFramePoint3D copKnot = new YoFramePoint3D("copKnot" + i, worldFrame, registry);
         copKnots.add(copKnot);

         YoGraphicPosition copKnotViz = new YoGraphicPosition("CoPKnot" + i, copKnot, POINT_SIZE, YoAppearance.Green(), YoGraphicPosition.GraphicType.BALL);
         graphicsList.add(copKnotViz);
         artifactList.add(copKnotViz.createArtifact());
      }

      artifactList.setVisible(VISUALIZE);
      graphicsList.setVisible(VISUALIZE);

      if (yoGraphicsListRegistry != null)
      {
         yoGraphicsListRegistry.registerYoGraphicsList(graphicsList);
         yoGraphicsListRegistry.registerArtifactList(artifactList);
      }
   }

   public void initialize()
   {
      contactSequenceInAbsoluteTime.clear();
   }

   public List<QuadrupedContactPhase> getContactSequence()
   {
      return contactSequenceInRelativeTime;
   }

   public void update(List<? extends QuadrupedTimedStep> stepSequence, List<RobotQuadrant> currentFeetInContact, double currentTime)
   {
      // initialize contact state and sole positions
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         solePositions.get(robotQuadrant).setToZero(soleFrames.get(robotQuadrant));
         solePositions.get(robotQuadrant).changeFrame(worldFrame);
      }
      feetInContact.clear();
      for (int footIndex = 0; footIndex < currentFeetInContact.size(); footIndex++)
         feetInContact.add(currentFeetInContact.get(footIndex));

      QuadrupedContactSequenceTools.computeStepTransitionsFromStepSequence(stepTransitionsInAbsoluteTime, currentTime, stepSequence);
      QuadrupedContactSequenceTools.trimPastContactSequences(contactSequenceInAbsoluteTime, currentTime, currentFeetInContact, solePositions);

      computeContactPhasesFromStepTransitions();

      contactSequenceInRelativeTime.clear();
      for (int i = 0; i < contactSequenceInAbsoluteTime.size(); i++)
      {
         QuadrupedContactPhase contactPhase = contactSequenceInRelativeTime.add();
         contactPhase.reset();
         contactPhase.set(contactSequenceInAbsoluteTime.get(i));
      }

      // FIXME this isn't working correctly.
      //      QuadrupedContactSequenceTools.shiftContactSequencesToRelativeTime(contactSequenceInRelativeTime, currentTime);

      int currentSize = contactSequenceInRelativeTime.size();
      TimeIntervalTools.removeEndTimesLessThan(0.0, contactSequenceInRelativeTime);

      if (contactSequenceInRelativeTime.size() < currentSize)
         throw new RuntimeException("This should have already been addressed.");

      int copKnotIndex = 0;
      for (; copKnotIndex < Math.min(copKnots.size(), contactSequenceInRelativeTime.size()); copKnotIndex++)
         copKnots.get(copKnotIndex).set(contactSequenceInRelativeTime.get(copKnotIndex).getCopStartPosition());
      for (; copKnotIndex < copKnots.size(); copKnotIndex++)
         copKnots.get(copKnotIndex).setToNaN();
   }

   private void computeContactPhasesFromStepTransitions()
   {
      int numberOfTransitions = stepTransitionsInAbsoluteTime.size();
      QuadrupedContactPhase contactPhase = contactSequenceInAbsoluteTime.getLast();

      // compute transition time and center of pressure for each time interval
      for (int transitionNumber = 0; transitionNumber < numberOfTransitions; transitionNumber++)
      {
         QuadrupedStepTransition stepTransition = stepTransitionsInAbsoluteTime.get(transitionNumber);

         for (int transitioningFootNumber = 0; transitioningFootNumber < stepTransition.getNumberOfFeetInTransition(); transitioningFootNumber++)
         {
            RobotQuadrant transitionQuadrant = stepTransition.getTransitionQuadrant(transitioningFootNumber);
            switch (stepTransition.getTransitionType(transitioningFootNumber))
            {
            case LIFT_OFF:
               feetInContact.remove(transitionQuadrant);
               break;
            case TOUCH_DOWN:
               feetInContact.add(transitionQuadrant);
               solePositions.get(transitionQuadrant).setIncludingFrame(worldFrame, stepTransition.getTransitionPosition(transitionQuadrant));
               break;
            }
         }

         // end the previous phase and add a new one
         contactPhase.getTimeInterval().setEndTime(stepTransition.getTransitionTime());
         contactPhase = contactSequenceInAbsoluteTime.add();

         contactPhase.reset();
         contactPhase.setFeetInContact(feetInContact);
         contactPhase.setSolePositions(solePositions);
         contactPhase.getTimeInterval().setStartTime(stepTransition.getTransitionTime());
         contactPhase.update();

         boolean isLastContact = (transitionNumber == numberOfTransitions - 1) || (contactSequenceInAbsoluteTime.size() == maxCapacity);
         if (isLastContact)
            break;
      }

      contactPhase.getTimeInterval().setEndTime(Double.POSITIVE_INFINITY);

   }

}
