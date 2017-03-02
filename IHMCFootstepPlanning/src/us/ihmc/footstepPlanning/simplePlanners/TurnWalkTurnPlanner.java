package us.ihmc.footstepPlanning.simplePlanners;

import java.util.ArrayList;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.Pose2dReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class TurnWalkTurnPlanner implements FootstepPlanner
{

   private static final String STRAIGHT_PATH_NAME = "Forward Path";
   private static final double STRAIGHT_STEP_LENGTH = 0.3; // For Steppr: 0.30;
   private static final double STRAIGHT_STEP_WIDTH = 0.3; // For Steppr: 0.35;
   private static final String REVERSE_PATH_NAME = "Reverse Path";
   private static final double REVERSE_ANGLE = Math.PI;
   private static final double REVERSE_STEP_LENGTH = 0.15;
   private static final double REVERSE_STEP_WIDTH = 0.3; // For Steppr: 0.35;
   private static final String RIGHT_SHUFFLE_PATH_NAME = "Right Shuffle Path";
   private static final String LEFT_SHUFFLE_PATH_NAME = "Left Shuffle Path";
   private static final double SHUFFLE_STEP_LENGTH = 0.25;  // For Steppr: 0.3;
   private static final double SHUFFLE_STEP_WIDTH = 0.21;
   private static final double LEFT_SHUFFLE_ANGLE = -Math.PI / 2;

   private final double epsilon = 10E-6;

   public static double maximumHipOpeningAngle = Math.toRadians(20.0);
   public static double maximumHipClosingAngle = Math.toRadians(0.0);
   public static double turningStepWidth = 0.3;

   private final FramePose2d initialStanceFootPose = new FramePose2d();
   private final FramePose2d goalPose = new FramePose2d();
   private RobotSide initialStanceSide;
   private RobotSide lastStepSide;
   private final Pose2dReferenceFrame stanceFootFrame = new Pose2dReferenceFrame("StanceFootFrame", ReferenceFrame.getWorldFrame());
   private final Pose2dReferenceFrame turningFrame = new Pose2dReferenceFrame("TurningFrame", ReferenceFrame.getWorldFrame());
   private double groundHeight = 0.0;

   @Override
   public void setInitialStanceFoot(FramePose stanceFootPose, RobotSide side)
   {
      this.initialStanceFootPose.set(FlatGroundPlanningUtils.pose2dFormPose(stanceFootPose));
      this.initialStanceFootPose.changeFrame(ReferenceFrame.getWorldFrame());
      this.initialStanceSide = side;
      this.lastStepSide = side;
      this.groundHeight = stanceFootPose.getZ();
   }

   @Override
   public void setGoal(FootstepPlannerGoal goal)
   {
      FramePose goalPose = goal.getGoalPoseBetweenFeet();
      this.goalPose.set(FlatGroundPlanningUtils.pose2dFormPose(goalPose));
      this.goalPose.changeFrame(ReferenceFrame.getWorldFrame());
   }

   private FootstepPlan footstepPlan = new FootstepPlan();

   @Override
   public FootstepPlanningResult plan()
   {
      stanceFootFrame.setPoseAndUpdate(initialStanceFootPose);

      FramePoint2d goalPoint = new FramePoint2d();
      goalPose.getPosition(goalPoint);

      ArrayList<FramePose2d> footstepList = new ArrayList<>();

      // turn
      Point2D robotOffsetFromStanceFoot = new Point2D(0.0, lastStepSide.negateIfLeftSide(turningStepWidth / 2.0));
      FramePose2d robotPose = new FramePose2d(stanceFootFrame, robotOffsetFromStanceFoot, 0.0);
      FramePose2d robotPoseInWorld = new FramePose2d(robotPose);
      robotPoseInWorld.changeFrame(ReferenceFrame.getWorldFrame());
      addTurnInPlaceToFacePoint(footstepList, robotPoseInWorld, goalPoint);

      // walk
      FramePoint2d robotPosition = new FramePoint2d();
      robotPoseInWorld.getPosition(robotPosition);
      double distanceToTravel = robotPosition.distance(goalPoint);
      addStraightWalk(footstepList, robotPosition, distanceToTravel);

      // turn
      FramePose2d stanceFootPose = new FramePose2d(stanceFootFrame);
      stanceFootPose.changeFrame(goalPose.getReferenceFrame());
      double turningAngle = AngleTools.trimAngleMinusPiToPi(goalPose.getYaw() - stanceFootPose.getYaw());
      FramePoint2d pointToTurnAbout = new FramePoint2d(stanceFootFrame, new Point2D(0.0, lastStepSide.negateIfLeftSide(STRAIGHT_STEP_WIDTH / 2.0)));
      addTurnInPlace(footstepList, turningAngle, pointToTurnAbout);

      // square up
      addSquareUp(footstepList, pointToTurnAbout);

      footstepPlan.clear();
      RobotSide robotSide = initialStanceSide;
      for (FramePose2d footstepPose2d : footstepList)
      {
         robotSide = robotSide.getOppositeSide();
         footstepPlan.addFootstep(robotSide, FlatGroundPlanningUtils.poseFormPose2d(footstepPose2d, groundHeight));
      }
      return FootstepPlanningResult.OPTIMAL_SOLUTION;
   }

   private void addSquareUp(ArrayList<FramePose2d> footstepList, FramePoint2d robotPosition)
   {
      robotPosition.changeFrame(stanceFootFrame);
      if (Math.abs(robotPosition.getX()) > 0.001)
         throw new RuntimeException("Can not square up for given position.");

      robotPosition.changeFrame(stanceFootFrame);
      FramePose2d footstepPose = new FramePose2d(stanceFootFrame);
      footstepPose.setY(2.0*robotPosition.getY());

      if(lastStepSide.equals(RobotSide.LEFT) && footstepPose.getY() > 0)
         throw new RuntimeException("Left foot can not be placed on right side of right foot");


      if (lastStepSide.equals(RobotSide.RIGHT) && footstepPose.getY() < 0)
         throw new RuntimeException("Right foot can not be placed on left side of left foot");

      footstepPose.changeFrame(ReferenceFrame.getWorldFrame());

      footstepList.add(footstepPose);
      stanceFootFrame.setPoseAndUpdate(footstepPose);
      lastStepSide = lastStepSide.getOppositeSide();
   }

   private void addStraightWalk(ArrayList<FramePose2d> footstepList, FramePoint2d startingPoint, double distanceToTravel)
   {

      if(distanceToTravel<epsilon)
         return;

      double straightSteps = Math.ceil(distanceToTravel / STRAIGHT_STEP_LENGTH);
      double stepLength = distanceToTravel / straightSteps;
      FramePoint2d startingPointInWorld = new FramePoint2d(startingPoint);
      startingPointInWorld.changeFrame(ReferenceFrame.getWorldFrame());

      for (int i = 0; i < straightSteps; i++)
      {
         startingPoint.setIncludingFrame(startingPointInWorld);
         startingPoint.changeFrame(stanceFootFrame);

         FramePose2d nextFootStep = new FramePose2d(stanceFootFrame);
         nextFootStep.setX(stepLength);
         nextFootStep.setY(startingPoint.getY() + lastStepSide.negateIfLeftSide(STRAIGHT_STEP_WIDTH / 2.0));


         if(lastStepSide.equals(RobotSide.LEFT) && nextFootStep.getY() > 0)
            throw new RuntimeException("Left foot can not be placed on right side of right foot");


         if (lastStepSide.equals(RobotSide.RIGHT) && nextFootStep.getY() < 0)
            throw new RuntimeException("Right foot can not be placed on left side of left foot");

         nextFootStep.changeFrame(ReferenceFrame.getWorldFrame());
         footstepList.add(nextFootStep);
         stanceFootFrame.setPoseAndUpdate(nextFootStep);
         lastStepSide = lastStepSide.getOppositeSide();
      }
   }

   private void addTurnInPlaceToFacePoint(ArrayList<FramePose2d> footstepList, FramePose2d robotPose, FramePoint2d goalPoint)
   {
      double turningAngle = AngleTools.calculateHeading(robotPose, goalPoint, -robotPose.getYaw(), 0.0);
      FramePoint2d pointToTurnAbout = new FramePoint2d();
      robotPose.getPosition(pointToTurnAbout);
      addTurnInPlace(footstepList, turningAngle, pointToTurnAbout);
   }

   private void addTurnInPlace(ArrayList<FramePose2d> footstepList, double turningAngle, FramePoint2d pointToTurnAbout)
   {
      if(Math.abs(turningAngle)<epsilon)
         return;

      FramePoint2d pointToTurnAboutInWorld = new FramePoint2d(pointToTurnAbout);
      pointToTurnAboutInWorld.changeFrame(ReferenceFrame.getWorldFrame());

      pointToTurnAbout.changeFrame(stanceFootFrame);
      if (Math.abs(pointToTurnAbout.getX()) > 0.001)
         throw new RuntimeException("Can not turn in place around given point.");

      RobotSide sideToTurnTo = turningAngle >= 0.0 ? RobotSide.LEFT : RobotSide.RIGHT;

      double twoStepTurnAngle = maximumHipClosingAngle + maximumHipOpeningAngle;
      double requiredDoubleSteps = Math.abs(turningAngle / twoStepTurnAngle);

      double turningSteps = 2.0 * Math.ceil(requiredDoubleSteps);
      double maxTurningAngle = Math.ceil(requiredDoubleSteps) * twoStepTurnAngle;
      boolean firstStepClosing = sideToTurnTo.equals(lastStepSide);
      if (firstStepClosing)
      {
         if (Math.floor(requiredDoubleSteps) * twoStepTurnAngle + maximumHipClosingAngle >= Math.abs(turningAngle))
         {
            turningSteps--;
            maxTurningAngle -= maximumHipOpeningAngle;
         }
      }
      else
      {
         if (Math.floor(requiredDoubleSteps) * twoStepTurnAngle + maximumHipOpeningAngle >= Math.abs(turningAngle))
         {
            turningSteps--;
            maxTurningAngle -= maximumHipClosingAngle;
         }
      }
      double scaleTurningAngle = Math.abs(turningAngle) / maxTurningAngle;

      for (int i = 0; i < turningSteps; i++)
      {
         FramePose2d turningFramePose = new FramePose2d(stanceFootFrame);
         pointToTurnAbout.setIncludingFrame(pointToTurnAboutInWorld);
         pointToTurnAbout.changeFrame(stanceFootFrame);
         turningFramePose.setY(pointToTurnAbout.getY());

         if (sideToTurnTo.equals(lastStepSide))
         {
            turningFramePose.setYaw(sideToTurnTo.negateIfRightSide(maximumHipClosingAngle * scaleTurningAngle));
         }
         else
         {
            turningFramePose.setYaw(sideToTurnTo.negateIfRightSide(maximumHipOpeningAngle * scaleTurningAngle));
         }
         turningFramePose.changeFrame(ReferenceFrame.getWorldFrame());
         turningFrame.setPoseAndUpdate(turningFramePose);

         FramePose2d nextFootstep = new FramePose2d(turningFrame);
         nextFootstep.setY(lastStepSide.negateIfLeftSide(turningStepWidth / 2.0));

         if(lastStepSide.equals(RobotSide.LEFT) && nextFootstep.getY() > 0)
            throw new RuntimeException("Left foot can not be placed on right side of right foot");


         if (lastStepSide.equals(RobotSide.RIGHT) && nextFootstep.getY() < 0)
            throw new RuntimeException("Right foot can not be placed on left side of left foot");

         nextFootstep.changeFrame(ReferenceFrame.getWorldFrame());


         footstepList.add(nextFootstep);
         stanceFootFrame.setPoseAndUpdate(nextFootstep);
         lastStepSide = lastStepSide.getOppositeSide();

      }
      pointToTurnAbout.setIncludingFrame(pointToTurnAboutInWorld);
      pointToTurnAbout.changeFrame(stanceFootFrame);
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
   }

   @Override
   public FootstepPlan getPlan()
   {
      return footstepPlan;
   }
}
