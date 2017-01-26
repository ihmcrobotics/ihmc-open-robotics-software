package us.ihmc.humanoidRobotics.footstep.footstepGenerator;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.overheadPath.CompositeOverheadPath;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.overheadPath.OverheadPath;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;


public abstract class AbstractSimpleParametersFootstepGenerator extends AbstractFootstepGenerator
{
   protected final DoubleYoVariable straightWalkingStepLength = new DoubleYoVariable("straightWalkingStepLength", registry);
   protected final DoubleYoVariable straightWalkingStepWidth = new DoubleYoVariable("straightWalkingStepWidth", registry);
   protected final DoubleYoVariable turningWalkingClosingAngleIncrement = new DoubleYoVariable("turningWalkingInswardsAngleIncrement", registry);
   protected final DoubleYoVariable turningWalkingOpeningAngleIncrement = new DoubleYoVariable("turningWalkingAngleIncrement", registry);
   protected final DoubleYoVariable turningWalkingStepWidth = new DoubleYoVariable("turningWalkingStepWidth", registry);
   protected boolean isLeftRightPath;
   protected boolean isRightwardPath;
   protected FootstepCounter footstepCounter;

   public AbstractSimpleParametersFootstepGenerator(SideDependentList<RigidBody> feet, SideDependentList<ReferenceFrame> soleFrames, PathTypeStepParameters pathType)
   {
      super(feet, soleFrames);
      setPathParameters(pathType);
   }

   // set a start side preference for when it is not forced to a specific side.
   public AbstractSimpleParametersFootstepGenerator(SideDependentList<RigidBody> feet, SideDependentList<ReferenceFrame> soleFrames, PathTypeStepParameters pathType, RobotSide stanceStart)
   {
      super(feet, soleFrames, stanceStart);
      setPathParameters(pathType);
   }

   public void setPathParameters(PathTypeStepParameters pathType)
   {
      setStraightWalkingStepLength(pathType.getStepLength());
      setStraightWalkingStepWidth(pathType.getStepWidth());

      setTurningStepsHipOpeningStepAngle(pathType.getTurningOpenStepAngle());
      setTurningStepsHipClosingStepAngle(pathType.getTurningCloseStepAngle());
      setTurningStepsStepWidth(pathType.getTurningStepWidth());

      calculateIsLeftRightPath(pathType.getAngle());
   }

   protected void generateFootsteps(ArrayList<Footstep> ret)
   {
      double openingAngle = turningWalkingOpeningAngleIncrement.getDoubleValue();
      double closingAngle = turningWalkingClosingAngleIncrement.getDoubleValue();
      double stepLength = straightWalkingStepLength.getDoubleValue();
      double stepWidth = straightWalkingStepWidth.getDoubleValue();

      // Find currentFootstepSide (for non-standard initial feet poses:
      // 1) based on whether a side was specified,
      // 2) assuming hipOpening and there are turns (if no side preference),
      // 3a) near foot if L/R (whether or not turn steps made or side preference indicated) [independently decided start step]
      // 3b) far foot if F/B and no turn steps made and no preference specified

      RobotSide sideOfHipAngleOpeningStep = sideOfHipAngleOpeningStep(getSignedInitialTurnDirection());

      if (startStancePreferenceSpecified)
      {
         // find currentFootstepSide: 1) based on whether a side was specified,
         currentFootstepSide = stanceStartSidePreference.getOppositeSide();
      }
      else
      {
         // find currentFootstepSide: 2) assuming hipOpening and there are turns (if no side preference),
         // If there is an initial yaw between steps, choose the side that will give the larger first displacement
         if (initialDeltaFeetYaw <= (openingAngle + closingAngle) / 2 - closingAngle)
            currentFootstepSide = sideOfHipAngleOpeningStep;
         else
            currentFootstepSide = sideOfHipAngleOpeningStep.getOppositeSide();
      }

      boolean hipExtensionFirst = currentFootstepSide == sideOfHipAngleOpeningStep;

      boolean willSetFirstStraightStepToFarSideIfNoTurns;
      if (isLeftRightPath)
      {
         if ((stepLength + stepWidth - initialDeltaFeetY) > (initialDeltaFeetY - stepWidth))//Shouldn't do just largest displacement. Needs to be bounded for max not just min!
            willSetFirstStraightStepToFarSideIfNoTurns = false;    // near foot
         else
            willSetFirstStraightStepToFarSideIfNoTurns = true;    // far foot
      }
      else
      {
         if (startStancePreferenceSpecified && (stanceStartSidePreference == getFarSideFootstep())
                 && (Math.abs(initialDeltaFeetX) < stepLength - noTranslationTolerance))
            willSetFirstStraightStepToFarSideIfNoTurns = false;
         else
            willSetFirstStraightStepToFarSideIfNoTurns = true;
      }

      initializeFootstepCounter(openingAngle, closingAngle, stepLength, hipExtensionFirst, willSetFirstStraightStepToFarSideIfNoTurns, stepWidth);
      List<Double> sValues = footstepCounter.getSValues2();
      List<Double> linearToTurningRatios = footstepCounter.getlinearToTurningRatios();
      List<Integer> subPathIndices = footstepCounter.getSubPathIndices();

      int straightPathStartIndex = footstepCounter.getFirstStraightPathStepIndex();


      // find currentFootstepSide:
      // 3a) near foot if L/R (whether or not turn steps made or side preference indicated) [independently decided start step]
      int skipFoot = -1;
      if (isLeftRightPath)
      {
         RobotSide nearSideStep = isRightwardPath ? RobotSide.RIGHT : RobotSide.LEFT;

         if (straightPathStartIndex == 0)
         {
            if(willSetFirstStraightStepToFarSideIfNoTurns)
               currentFootstepSide = nearSideStep.getOppositeSide();
            else
               currentFootstepSide = nearSideStep;
         }
         else    // straightPathStartIndex > 0
         {
            RobotSide sideOfStepJustBeforeStraightStep = (straightPathStartIndex % 2 == 1) ? currentFootstepSide : currentFootstepSide.getOppositeSide();
            if (sideOfStepJustBeforeStraightStep == nearSideStep)
               skipFoot = straightPathStartIndex - 1;
         }
      }

      // find currentFootstepSide:
      // 3b) far foot if F/B and no turn steps made and no preference specified (set to preference if specified, but should have already happened)
      else if ((straightPathStartIndex == 0) && willSetFirstStraightStepToFarSideIfNoTurns)
      {
         currentFootstepSide = getFarSideFootstep();
      }

      for (int i = 0; i < sValues.size(); i++)
      {
         if (i == skipFoot)
            continue;

         double subPathS = sValues.get(i);
         double linearToTurningRatio = linearToTurningRatios.get(i);
         int subPathIndex = subPathIndices.get(i);

         generateFootstep(ret, currentFootstepSide, linearToTurningRatio, subPathS,subPathIndex);
         currentFootstepSide = currentFootstepSide.getOppositeSide();
      }

   }

   protected abstract double getSignedInitialTurnDirection();

   protected abstract void initializeFootstepCounter(double openingAngle, double closingAngle, double stepLength, boolean hipExtensionFirst,
           boolean willSetFirstStraightStepToFarSideIfNoTurns, double standardFootWidth);

   private void generateFootstep(ArrayList<Footstep> ret, RobotSide currentFootstepSide, double linearToTurningStepRatio, double compositePathS, int pathIndex)
   {
      double stepWidth = turningWalkingStepWidth.getDoubleValue() * (1 - linearToTurningStepRatio)
                         + straightWalkingStepWidth.getDoubleValue() * linearToTurningStepRatio;

      Footstep footstep = getFootstepAtS(currentFootstepSide, compositePathS, stepWidth,pathIndex);

      ret.add(footstep);
   }

   public void setStraightWalkingStepLength(double stepLength)
   {
      this.straightWalkingStepLength.set(stepLength);
   }

   public void setStraightWalkingStepWidth(double stepWidth)
   {
      this.straightWalkingStepWidth.set(stepWidth);
   }

   public void setTurningStepsHipClosingStepAngle(double stepAngle)
   {
      this.turningWalkingClosingAngleIncrement.set(stepAngle);
   }

   public void setTurningStepsHipOpeningStepAngle(double stepAngle)
   {
      this.turningWalkingOpeningAngleIncrement.set(stepAngle);
   }

   public void setTurningStepsStepWidth(double spinningRadius)
   {
      this.turningWalkingStepWidth.set(spinningRadius);
   }

   protected void calculateIsLeftRightPath(double angle)
   {
      int discreteAngleRepresentation = AngleTools.findClosestNinetyDegreeYaw(angle);
      isRightwardPath = discreteAngleRepresentation == 1;
      isLeftRightPath = discreteAngleRepresentation % 2 == 1;
   }

   protected Footstep getFootstepAtS(RobotSide currentFootstepSide, double pathSParameter, double stepWidth, int subPathIndex)
   {
      OverheadPath path = getPath();
      FramePose2d planningPose;
      if(path instanceof CompositeOverheadPath)
         planningPose = ((CompositeOverheadPath) path).getPoseAtS(pathSParameter,subPathIndex);
      else
         planningPose = path.getPoseAtS(pathSParameter);
      FramePoint2d footstepPosition2d = new FramePoint2d();
      planningPose.getPosition(footstepPosition2d);
      double footHeading = planningPose.getYaw();
      footstepPosition2d = offsetFootstepFromPath(currentFootstepSide, footstepPosition2d, footHeading, stepWidth / 2);
      FramePose2d footstepPose2d = new FramePose2d(planningPose);
      footstepPose2d.setPosition(footstepPosition2d);

      return createFootstep(currentFootstepSide, footstepPose2d);
   }

   public abstract boolean hasDisplacement();
}
