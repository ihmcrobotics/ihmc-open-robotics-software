package us.ihmc.humanoidRobotics.footstep.footstepGenerator;

import java.util.ArrayList;

import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.overheadPath.OverheadPath;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.overheadPath.StraightLineOverheadPath;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;

public class TranslationFootstepGenerator extends AbstractFootstepGenerator
{
   // private static final boolean DEBUG = false;
   private FramePoint2d endPoint;
   private StraightLineOverheadPath footstepPath;
   protected final DoubleYoVariable forwardWalkingStepLength = new DoubleYoVariable("translationalForwardWalkingStepLength", registry);
   protected final DoubleYoVariable backwardWalkingStepLength = new DoubleYoVariable("translationalBackwardStepLength", registry);
   protected final DoubleYoVariable sidewardWalkingStepLength = new DoubleYoVariable("translationalSidewardStepLength", registry);
   protected final DoubleYoVariable nominalWalkingStepWidth = new DoubleYoVariable("translationalForwardWalkingStepWidth", registry);
   protected final DoubleYoVariable minimumWalkingStepWidth = new DoubleYoVariable("translationalMinimumStepWidth", registry);
   private boolean isRightwardPath;
   private boolean isForwardPath;

   public TranslationFootstepGenerator(SideDependentList<RigidBody> feet, SideDependentList<ReferenceFrame> soleFrames, FramePoint2d endPoint, TranslationalPathParameters translationalPathType)
   {
      super(feet, soleFrames);
      setPathParameters(translationalPathType);
      this.endPoint = endPoint;
   }

   public TranslationFootstepGenerator(SideDependentList<RigidBody> feet, SideDependentList<ReferenceFrame> soleFrames, FramePoint2d endPoint,
         TranslationalPathParameters translationalPathType, RobotSide startStanceSide)
   {
      super(feet, soleFrames, startStanceSide);
      setPathParameters(translationalPathType);
      this.endPoint = endPoint;
   }

   public void setPathParameters(TranslationalPathParameters translationalPathType)
   {
      this.forwardWalkingStepLength.set(translationalPathType.getForwardStepLength());
      this.backwardWalkingStepLength.set(translationalPathType.getBackwardStepLength());
      this.sidewardWalkingStepLength.set(translationalPathType.getSidewardStepLength());
      this.nominalWalkingStepWidth.set(translationalPathType.getNominalStepWidth());
      this.minimumWalkingStepWidth.set(translationalPathType.getMinimumStepWidth());
   }

   protected void initialize(FramePose2d startPose)
   {
      this.footstepPath = new StraightLineOverheadPath(startPose, endPoint);
   }

   @Override
   protected void generateFootsteps(ArrayList<Footstep> ret)
   {
      double maxForwardStepLength = forwardWalkingStepLength.getDoubleValue();
      double maxBackwardStepLength = backwardWalkingStepLength.getDoubleValue();
      double maxSidewardStepLength = sidewardWalkingStepLength.getDoubleValue();
      double nominalStepWidth = nominalWalkingStepWidth.getDoubleValue();
      double minimumStepWidth = minimumWalkingStepWidth.getDoubleValue();
      if (minimumStepWidth > nominalStepWidth)
      {
         nominalStepWidth = minimumStepWidth;
         System.err.println("minimum translational step width is larger than nominal translational step width. nominal changed to min");
      }

      double totalDistance = footstepPath.getDistance();

      FramePose2d poseAtS0 = footstepPath.getPoseAtS(0);
      FramePoint2d position = new FramePoint2d();
      poseAtS0.getPosition(position);
      FramePoint2d position2 = new FramePoint2d();
      footstepPath.getPoseAtS(1).getPosition(position2);
      position2.sub(position);
      double pathAngle = Math.atan2(position2.getY(), position2.getX());
      double yaw = poseAtS0.getYaw();
      double relativePathAngle = AngleTools.computeAngleDifferenceMinusPiToPi(pathAngle, yaw);

      // Number of steps based on max of forward type and sidewards type path
      double totalLRDistance = totalDistance * Math.sin(relativePathAngle);
      double totalFBDistance = totalDistance * Math.cos(relativePathAngle);

      isForwardPath = (relativePathAngle >= -Math.PI / 2) && (relativePathAngle <= Math.PI / 2);
      double maxFBStepLength;
      if (isForwardPath)
         maxFBStepLength = maxForwardStepLength;
      else
         maxFBStepLength = maxBackwardStepLength;

      isRightwardPath = (relativePathAngle < 0) && (relativePathAngle > -Math.PI);
      RobotSide leftRightPathDirection = isRightwardPath ? RobotSide.RIGHT : RobotSide.LEFT;

      // Set start foot
      boolean startWithNearSideFoot = setStartFootSide(maxSidewardStepLength, minimumStepWidth, maxFBStepLength, leftRightPathDirection, relativePathAngle);

      // TODO: Adjust totalDistance based on offsets from standard initial conditions
      // TODO: Find first step distances
      double initialLRFeetOffsetAdjustment = 0;
      double initialFBFeetOffsetAdjustment = 0;
      if (startWithNearSideFoot)
      {
         // isRightwardPath
         initialLRFeetOffsetAdjustment = (minimumStepWidth - initialDeltaFeetY) / 2.0; // near foot adjustment (correct)
         initialFBFeetOffsetAdjustment = Math.abs(initialDeltaFeetX) / 2.0; // near foot adjustment (potential for negative pathS)
      }
      else
      {
         initialLRFeetOffsetAdjustment = (initialDeltaFeetY - minimumStepWidth) / 2.0; // far foot adjustment (will require extra step or one less...)
         initialFBFeetOffsetAdjustment = Math.abs(initialDeltaFeetX) / 2.0; // far foot adjustment
      }

      if (isRightwardPath)
         initialLRFeetOffsetAdjustment = -initialLRFeetOffsetAdjustment;

      // Number of steps based on max of forward type and sidewards type path
      if (Math.abs(totalLRDistance) <= Math.abs(initialLRFeetOffsetAdjustment))
         initialLRFeetOffsetAdjustment = 0;
      totalLRDistance -= initialLRFeetOffsetAdjustment;
      totalFBDistance -= initialFBFeetOffsetAdjustment;

      double stepTheta = Math.atan2(totalLRDistance / maxSidewardStepLength, totalFBDistance / 2 / maxFBStepLength);
      boolean sinThetaNotCloseToZero = (Math.abs(stepTheta) > Math.PI / 4) && (Math.abs(stepTheta) < 3.0 / 4.0 * Math.PI);
      double approxNEvenSteps = sinThetaNotCloseToZero ? 2 * totalLRDistance / (maxSidewardStepLength * Math.sin(stepTheta)) : totalFBDistance
            / (maxFBStepLength * Math.cos(stepTheta));
      int nEvenSteps = (int) Math.ceil(approxNEvenSteps);
      if (approxNEvenSteps < 1e-14)
         nEvenSteps = 0;
      if (nEvenSteps % 2 == 1)
         nEvenSteps += 1;

      // TODO: add evaluation for nOddSteps as well..., choose the one with fewer steps

      int nSteps = nEvenSteps;
      int nFB = nSteps;
      int nLR;
      int numShortForSquareSteps = 1;

      if (nSteps % 2 == 0)
      {
         if (startWithNearSideFoot)
         {
            nLR = nSteps / 2;
            if (Math.abs(totalFBDistance) > 1e-14)
               numShortForSquareSteps = 1;
            else
               numShortForSquareSteps = 2;
         }
         else
         {
            nLR = (nSteps + 1) / 2;
            numShortForSquareSteps = 1;
         }
      }
      else if (startWithNearSideFoot)
      {
         nLR = (nSteps + 1) / 2; // preferred case as it never will have restep problems for sidewards directions.
      }
      else
      {
         nLR = (nSteps - 1) / 2;
      }

      double lrStepLength = totalLRDistance / nLR;
      double fbStepLength = totalFBDistance / nFB;

      // Choose stepWidth between nominalStepWidth and minimumStepWidth such that lrStepLength + stepWidth/2 - initialStepWidth/2 >= 0
      double stepWidth;
      if (nLR == 0)
         stepWidth = nominalStepWidth;
      else
         stepWidth = MathTools.clamp(initialDeltaFeetY - Math.abs(lrStepLength) * 2, minimumStepWidth, nominalStepWidth);

      boolean isSideStep = startWithNearSideFoot;
      FramePoint2d currentPathPosition = position;

      currentPathPosition = displacePosition(currentPathPosition, yaw, initialFBFeetOffsetAdjustment, initialLRFeetOffsetAdjustment);

      for (int i = 0; i < nSteps - numShortForSquareSteps; i++)
      {
         currentPathPosition = displacePosition(currentPathPosition, yaw, fbStepLength, isSideStep ? lrStepLength : 0.0);

         addFootstep(ret, currentPathPosition, stepWidth, yaw);
         isSideStep = !isSideStep;
      }

      // Do two square up steps
      //    stepWidth = nominalStepWidth;//Changing to nominal at end can cause overstep if stepWidth is smaller than stepWidth for the rest of the path.
      FramePoint2d position3 = new FramePoint2d();
      footstepPath.getPoseAtS(1).getPosition(position3);
      addFootstep(ret, position3, stepWidth, yaw);
      addFootstep(ret, position3, stepWidth, yaw);
   }

   protected boolean setStartFootSide(double maxSidewardStepLength, double minimumStepWidth, double maxFBStepLength, RobotSide leftRightPathDirection,
         double relativePathAngle)
   {
      // Consider start foot based on initial delta x and initial delta y
      RobotSide leftRightStartSidePreference;
      if ((maxSidewardStepLength + minimumStepWidth - initialDeltaFeetY) > (initialDeltaFeetY - minimumStepWidth)) // Shouldn't do just largest displacement. Needs to be bounded for max not just min!
         leftRightStartSidePreference = leftRightPathDirection; // near foot in lr direction
      else
         leftRightStartSidePreference = leftRightPathDirection.getOppositeSide(); // far foot in lr direction

      RobotSide frontBackStartSidePreference = getFarSideFootstep();

      if (leftRightStartSidePreference == frontBackStartSidePreference)
      {
         // standard result if stopping a translation path in the middle.
         currentFootstepSide = leftRightStartSidePreference;
      }
      else
      {
         if ((Math.abs(relativePathAngle) < Math.PI / 4) || (Math.abs(relativePathAngle) > Math.PI * 3 / 4))
            currentFootstepSide = frontBackStartSidePreference;
         else
            currentFootstepSide = leftRightStartSidePreference;
      }

      // Check whether starting on near or far foot in order to know whether to start with a pure forward step.
      boolean startWithNearSideFoot = currentFootstepSide == leftRightPathDirection;

      return startWithNearSideFoot;
   }

   private void addFootstep(ArrayList<Footstep> ret, FramePoint2d currentPathPosition, double stepWidth, double yaw)
   {
      Footstep footstep = getFootstepAtPosition(currentFootstepSide, currentPathPosition, stepWidth, yaw);
      ret.add(footstep);

      currentFootstepSide = currentFootstepSide.getOppositeSide();
   }

   private Footstep getFootstepAtPosition(RobotSide currentFootstepSide, FramePoint2d footstepPosition2d, double stepWidth, double yaw)
   {
      double footHeading = yaw;
      FramePoint2d footstepPosition = offsetFootstepFromPath(currentFootstepSide, footstepPosition2d, footHeading, stepWidth / 2);
      FramePose2d footstepPose2d = new FramePose2d(WORLD_FRAME, footstepPosition.getPoint(), yaw);

      return createFootstep(currentFootstepSide, footstepPose2d);
   }

   protected FramePoint2d displacePosition(FramePoint2d footstepPosition2d, double forwardHeading, double offsetForward, double offsetLeft)
   {
      double xOffset = offsetForward * Math.cos(forwardHeading) - offsetLeft * Math.sin(forwardHeading);
      double yOffset = offsetForward * Math.sin(forwardHeading) + offsetLeft * Math.cos(forwardHeading);
      FrameVector2d offsetVector = new FrameVector2d(WORLD_FRAME, xOffset, yOffset);
      footstepPosition2d.changeFrame(WORLD_FRAME);
      footstepPosition2d.add(offsetVector);

      return footstepPosition2d;
   }

   @Override
   protected OverheadPath getPath()
   {
      return footstepPath;
   }

   public double getDistance()
   {
      return footstepPath.getDistance();
   }

   public boolean hasDisplacement()
   {
      return footstepPath.getDistance() > noTranslationTolerance;
   }

   public boolean isRightwardPath()
   {
      return isRightwardPath;
   }

   public boolean isForwardPath()
   {
      return isForwardPath;
   }
}
