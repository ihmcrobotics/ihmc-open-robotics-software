package us.ihmc.humanoidOperatorInterface.footstep.footstepGenerator;

import java.util.ArrayList;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidOperatorInterface.dataStructures.HeightMapWithPoints;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepUtils;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.FootstepGenerator;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.PathTypeStepParameters;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class TwoSegmentFootstepGenerator implements FootstepGenerator
{
   private AbstractSimpleParametersFootstepGenerator footstepGenerator;
   private AbstractSimpleParametersFootstepGenerator footstepGenerator2;
   private final SideDependentList<RigidBodyBasics> feet;
   private final SideDependentList<ReferenceFrame> soleFrames;
   private SideDependentList<Footstep> midStanceFeet = new SideDependentList<Footstep>();
   private boolean secondSegmentManuallySpecifiesStart = false;

   public TwoSegmentFootstepGenerator(SideDependentList<RigidBodyBasics> feet, SideDependentList<ReferenceFrame> soleFrames, FramePose2D midPose, FramePose2D finalPose, PathTypeStepParameters pathType,
                                      PathTypeStepParameters pathType2)
   {
      this.feet = feet;
      this.soleFrames = soleFrames;
      footstepGenerator = new TurnStraightTurnFootstepGenerator(feet, soleFrames, midPose, pathType);
      footstepGenerator2 = new TurnStraightTurnFootstepGenerator(feet, soleFrames, finalPose, pathType2);
   }

   // TurnStraightTurn used despite the ring being set to auto-align before this to ensure that the orientation is reached even for
   // small displacement paths that could otherwise end early before fully aligning with the direction of the path in TurningThenStraightFootstepGenerator.
   public TwoSegmentFootstepGenerator(SideDependentList<RigidBodyBasics> feet, SideDependentList<ReferenceFrame> soleFrames, FramePose2D midPose, FramePoint2D finalPoint, PathTypeStepParameters pathType,
                                      PathTypeStepParameters pathType2)
   {
      this.feet = feet;
      this.soleFrames = soleFrames;
      footstepGenerator = new TurnStraightTurnFootstepGenerator(feet, soleFrames, midPose, pathType);
      footstepGenerator2 = new TurningThenStraightFootstepGenerator(feet, soleFrames, finalPoint, pathType2);
   }

   public TwoSegmentFootstepGenerator(SideDependentList<RigidBodyBasics> feet, SideDependentList<ReferenceFrame> soleFrames, FramePose2D midPose, FramePoint2D finalPoint, PathTypeStepParameters pathType,
                                      PathTypeStepParameters pathType2, RobotSide stanceStart)
   {
      this.feet = feet;
      this.soleFrames = soleFrames;
      footstepGenerator = new TurnStraightTurnFootstepGenerator(feet, soleFrames, midPose, pathType, stanceStart);
      footstepGenerator2 = new TurningThenStraightFootstepGenerator(feet, soleFrames, finalPoint, pathType2);
   }

   public TwoSegmentFootstepGenerator(SideDependentList<RigidBodyBasics> feet, SideDependentList<ReferenceFrame> soleFrames, FramePose2D midPose, FramePoint2D finalPoint, PathTypeStepParameters pathType,
                                      PathTypeStepParameters pathType2, RobotSide stanceStart, RobotSide midStanceStart)
   {
      this.feet = feet;
      this.soleFrames = soleFrames;
      footstepGenerator = new TurnStraightTurnFootstepGenerator(feet, soleFrames, midPose, pathType, stanceStart);
      secondSegmentManuallySpecifiesStart = true;
      footstepGenerator2 = new TurningThenStraightFootstepGenerator(feet, soleFrames, finalPoint, pathType2, midStanceStart);
   }

   @Override
   public ArrayList<Footstep> generateDesiredFootstepList()
   {
      ArrayList<Footstep> footstepsFirstSegment = new ArrayList<Footstep>(footstepGenerator.generateDesiredFootstepList());

      initializeStanceFeetOfSecondSegment(footstepsFirstSegment);
      if (footstepGenerator2.hasDisplacement())
      {
         ArrayList<Footstep> footstepsSecondSegment = new ArrayList<Footstep>(footstepGenerator2.generateDesiredFootstepList());

         return concatenateFootstepPaths(footstepsFirstSegment, footstepsSecondSegment);
      }
      else
         return footstepsFirstSegment;
   }

   private void initializeStanceFeetOfSecondSegment(ArrayList<Footstep> footstepsToControlRing)
   {
      int numFootsteps = footstepsToControlRing.size();
      RobotSide lastStepSide = RobotSide.RIGHT;
      if (numFootsteps >= 1)
      {
         Footstep lastFootstep = footstepsToControlRing.get(numFootsteps - 1);
         lastStepSide = lastFootstep.getRobotSide();
         setMidstep(lastFootstep);
      }
      else
      {
         System.err.println("Why are there no feet generated for the first segment of TwoSegmentFootstepGenerator?");
         setMidstep(FootstepUtils.generateStandingFootstep(lastStepSide, feet.get(lastStepSide), soleFrames.get(lastStepSide)));
      }

      if (numFootsteps >= 2)
      {
         setMidstep(footstepsToControlRing.get(numFootsteps - 2));
      }
      else
      {
         RobotSide lastStepOppositeSide = lastStepSide.getOppositeSide();
         setMidstep(FootstepUtils.generateStandingFootstep(lastStepOppositeSide, feet.get(lastStepOppositeSide), soleFrames.get(lastStepOppositeSide)));
      }

      if (!secondSegmentManuallySpecifiesStart)
         footstepGenerator2.setStanceStartPreference(lastStepSide);
      footstepGenerator2.setPriorFootposes(midStanceFeet);

      footstepGenerator2.initialize();
   }

   private void setMidstep(Footstep footstep)
   {
      midStanceFeet.set(footstep.getRobotSide(), footstep);
   }

   private ArrayList<Footstep> concatenateFootstepPaths(ArrayList<Footstep> firstSetOfSteps, ArrayList<Footstep> secondSetOfSteps)
   {
      int indexOfLastStepOfFirstSegment = firstSetOfSteps.size() - 1;
      RobotSide sideOfLastFootstepOfFirstSegment = firstSetOfSteps.get(indexOfLastStepOfFirstSegment).getRobotSide();
      RobotSide sideOfFirstFootstepOfSecondSegment = secondSetOfSteps.get(0).getRobotSide();

      if (sideOfLastFootstepOfFirstSegment == sideOfFirstFootstepOfSecondSegment)
         firstSetOfSteps.remove(indexOfLastStepOfFirstSegment);

      for (Footstep footstep : secondSetOfSteps)
      {
         firstSetOfSteps.add(footstep);
      }

      return firstSetOfSteps;
   }

   public void setPoseFinderParams(double footstepFittingBufferSize, double boundingBoxForFootstepHeightFindingSideLength)
   {
      footstepGenerator.setPoseFinderParams(footstepFittingBufferSize, boundingBoxForFootstepHeightFindingSideLength);
      footstepGenerator2.setPoseFinderParams(footstepFittingBufferSize, boundingBoxForFootstepHeightFindingSideLength);
   }

   public void setHeightMap(HeightMapWithPoints heightMap, SideDependentList<? extends ContactablePlaneBody> contactableFeet)
   {
      footstepGenerator.setHeightMap(heightMap, contactableFeet);
      footstepGenerator2.setHeightMap(heightMap, contactableFeet);
   }

   public boolean hasDisplacement()
   {
      return footstepGenerator.hasDisplacement() || footstepGenerator2.hasDisplacement();
   }
}
