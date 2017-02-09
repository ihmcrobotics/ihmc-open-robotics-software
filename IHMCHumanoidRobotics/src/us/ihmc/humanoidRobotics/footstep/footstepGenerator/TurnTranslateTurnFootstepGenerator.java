package us.ihmc.humanoidRobotics.footstep.footstepGenerator;

import java.util.ArrayList;

import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepUtils;
import us.ihmc.robotics.dataStructures.HeightMapWithPoints;
import us.ihmc.robotics.geometry.FrameOrientation2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;

public class TurnTranslateTurnFootstepGenerator implements FootstepGenerator
{
   private TurnInPlaceFootstepGenerator firstTurn;
   private TranslationFootstepGenerator translate;
   private TurnInPlaceFootstepGenerator lastTurn;
   private final SideDependentList<RigidBody> feet;
   private final SideDependentList<ReferenceFrame> soleFrames;
   private SideDependentList<Footstep> transitionStanceFeet = new SideDependentList<Footstep>();
   private FrameOrientation2d endOrientation;
   private PathTypeStepParameters pathType;

   public TurnTranslateTurnFootstepGenerator(SideDependentList<RigidBody> feet, SideDependentList<ReferenceFrame> soleFrames, FrameOrientation2d pathYaw, FramePose2d endPose,
         PathTypeStepParameters pathType, TranslationalPathParameters translationalPathType)
   {
      this.feet = feet;
      this.soleFrames = soleFrames;
      this.pathType = pathType;
      firstTurn = new TurnInPlaceFootstepGenerator(feet, soleFrames, pathYaw, pathType);
      FramePoint2d endPosition = new FramePoint2d();
      endPose.getPosition(endPosition);
      translate = new TranslationFootstepGenerator(feet, soleFrames, endPosition, translationalPathType);
      endOrientation = new FrameOrientation2d();
      endPose.getOrientation(endOrientation);
      lastTurn = new TurnInPlaceFootstepGenerator(feet, soleFrames, endOrientation, pathType);
   }

   public ArrayList<Footstep> generateDesiredFootstepList()
   {
      ArrayList<Footstep> currentSegmentFootsteps;
      ArrayList<Footstep> completePath = null;
      boolean previousDisplacement = false;
      boolean noTranslation = true;

      firstTurn.initialize();

      if (firstTurn.hasDisplacement())
      {
         previousDisplacement = true;
         currentSegmentFootsteps = new ArrayList<Footstep>(firstTurn.generateDesiredFootstepList());
         completePath = currentSegmentFootsteps;
      }

      initializeStanceFeet(completePath, translate);

      if (translate.hasDisplacement())
      {
         previousDisplacement = true;
         noTranslation = false;
         currentSegmentFootsteps = new ArrayList<Footstep>(translate.generateDesiredFootstepList());
         if (firstTurn.turnStepsTaken())
            completePath = concatenateFootstepPaths(completePath, currentSegmentFootsteps);
         else
            completePath = currentSegmentFootsteps; // Don't save turning square up steps if no turn steps taken.
      }

      initializeStanceFeet(completePath, lastTurn);

      if (lastTurn.hasDisplacement())
      {
         if (noTranslation && previousDisplacement)
         {
            // Handle the turn only case: combine both turns to a single turn so it won't stop in the middle or reverse directions between turns
            TurnInPlaceFootstepGenerator fullTurn = new TurnInPlaceFootstepGenerator(feet, soleFrames, endOrientation, pathType);
            firstTurn.initialize();
            completePath = new ArrayList<Footstep>(fullTurn.generateDesiredFootstepList());
         }
         else
         {
            // Final turn steps
            currentSegmentFootsteps = new ArrayList<Footstep>(lastTurn.generateDesiredFootstepList());
            if (lastTurn.turnStepsTaken())
               completePath = concatenateFootstepPaths(completePath, currentSegmentFootsteps);
            else
               completePath = replaceSquareUpSteps(completePath, currentSegmentFootsteps);
         }
      }
      else
      {
         if (previousDisplacement)
         {
            // Only square up steps, but need to delete last square up steps
            currentSegmentFootsteps = new ArrayList<Footstep>(lastTurn.generateDesiredFootstepList());
            completePath = replaceSquareUpSteps(completePath, currentSegmentFootsteps);
         }
         else
         {
            // Only square up steps
            currentSegmentFootsteps = new ArrayList<Footstep>(lastTurn.generateDesiredFootstepList());
            completePath = concatenateFootstepPaths(completePath, currentSegmentFootsteps);
         }
      }

      return completePath;
   }

   private void initializeStanceFeet(ArrayList<Footstep> previousFootsteps, AbstractFootstepGenerator generator)
   {
      if (previousFootsteps != null)
      {
         int numFootsteps = previousFootsteps.size();
         RobotSide lastStepSide = RobotSide.RIGHT;
         if (numFootsteps >= 1)
         {
            Footstep lastFootstep = previousFootsteps.get(numFootsteps - 1);
            lastStepSide = lastFootstep.getRobotSide();
            setTransitionStep(lastFootstep, transitionStanceFeet);
         }
         else
         {
            System.err.println("Why are there no feet generated for the first segment of TurnTranslateTurnFootstepGenerator?");
            setTransitionStep(FootstepUtils.generateStandingFootstep(lastStepSide, feet.get(lastStepSide), soleFrames.get(lastStepSide)), transitionStanceFeet);
         }

         if (numFootsteps >= 2)
         {
            setTransitionStep(previousFootsteps.get(numFootsteps - 2), transitionStanceFeet);
         }
         else
         {
            RobotSide lastStepOppositeSide = lastStepSide.getOppositeSide();
            setTransitionStep(FootstepUtils.generateStandingFootstep(lastStepOppositeSide, feet.get(lastStepOppositeSide), soleFrames.get(lastStepOppositeSide)), transitionStanceFeet);
         }
         generator.setPriorFootposes(transitionStanceFeet);
      }
      generator.initialize();
   }

   private void setTransitionStep(Footstep footstep, SideDependentList<Footstep> transitionStanceFeet)
   {
      transitionStanceFeet.set(footstep.getRobotSide(), footstep);
   }

   private ArrayList<Footstep> concatenateFootstepPaths(ArrayList<Footstep> firstSetOfSteps, ArrayList<Footstep> secondSetOfSteps)
   {
      if (firstSetOfSteps == null)
         return secondSetOfSteps;

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

   private ArrayList<Footstep> replaceSquareUpSteps(ArrayList<Footstep> firstSetOfSteps, ArrayList<Footstep> secondSetOfSteps)
   {
      if (firstSetOfSteps == null)
      {
         return secondSetOfSteps;
      }

      Footstep firstSquareUpStep = secondSetOfSteps.get(0);
      Footstep secondSquareUpStep = secondSetOfSteps.get(1);

      int indexOfLastStepOfFirstSegment = firstSetOfSteps.size() - 1;

      RobotSide sideOfLastFootstepOfFirstSegment = firstSetOfSteps.get(indexOfLastStepOfFirstSegment).getRobotSide();
      RobotSide sideOfFirstFootstepOfSecondSegment = firstSquareUpStep.getRobotSide();

      firstSetOfSteps.remove(indexOfLastStepOfFirstSegment);
      firstSetOfSteps.remove(indexOfLastStepOfFirstSegment - 1);

      if (secondSetOfSteps.size() > 2)
         System.err.println("Expected only two square up steps, but there were more!");

      if (sideOfLastFootstepOfFirstSegment == sideOfFirstFootstepOfSecondSegment)
      {
         firstSetOfSteps.add(secondSquareUpStep);
         firstSetOfSteps.add(firstSquareUpStep);
      }
      else
      {
         firstSetOfSteps.add(firstSquareUpStep);
         firstSetOfSteps.add(secondSquareUpStep);
      }
      return firstSetOfSteps;
   }

   public void setPoseFinderParams(double footstepFittingBufferSize, double boundingBoxForFootstepHeightFindingSideLength)
   {
      firstTurn.setPoseFinderParams(footstepFittingBufferSize, boundingBoxForFootstepHeightFindingSideLength);
      translate.setPoseFinderParams(footstepFittingBufferSize, boundingBoxForFootstepHeightFindingSideLength);
      lastTurn.setPoseFinderParams(footstepFittingBufferSize, boundingBoxForFootstepHeightFindingSideLength);
   }

   public void setHeightMap(HeightMapWithPoints heightMap, SideDependentList<? extends ContactablePlaneBody> contactableFeet)
   {
      firstTurn.setHeightMap(heightMap, contactableFeet);
      translate.setHeightMap(heightMap, contactableFeet);
      lastTurn.setHeightMap(heightMap, contactableFeet);
   }

   public boolean hasDisplacement()
   {
      return firstTurn.hasDisplacement() || translate.hasDisplacement() || lastTurn.hasDisplacement();
   }
}