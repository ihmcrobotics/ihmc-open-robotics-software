package us.ihmc.footstepPlanning.simplePlanners;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.referenceFrames.Pose2dReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.List;

public class TurnWalkTurnPlanner
{
   private static final boolean debug = false;
   private static final RobotSide defaultStartNodeSide = RobotSide.LEFT;
   private static final double epsilon = 10E-6;

   private final FramePose2D initialStanceFootPose = new FramePose2D();
   private final FramePose2D robotStartPose = new FramePose2D();
   private final FramePose2D goalPose = new FramePose2D();
   private RobotSide initialStanceSide;
   private RobotSide lastStepSide;
   private final Pose2dReferenceFrame robotStartFrame = new Pose2dReferenceFrame("RobotStartFrame", ReferenceFrame.getWorldFrame());
   private final Pose2dReferenceFrame planStanceFootFrame = new Pose2dReferenceFrame("StanceFootFrame", ReferenceFrame.getWorldFrame());
   private final Pose2dReferenceFrame turningFrame = new Pose2dReferenceFrame("TurningFrame", ReferenceFrame.getWorldFrame());
   private double groundHeight = 0.0;

   private final DefaultFootstepPlannerParametersReadOnly parameters;

   public TurnWalkTurnPlanner()
   {
      this(new DefaultTurnWalkTurnPlannerParameters());
   }

   public TurnWalkTurnPlanner(DefaultFootstepPlannerParametersReadOnly parameters)
   {
      this.parameters = parameters;
   }

   public void setInitialStanceFoot(FramePose3DReadOnly stanceFootPose, RobotSide side)
   {
      if (side == null)
      {
         if (debug)
            LogTools.info("Start node needs a side, but trying to set it to null. Setting it to " + defaultStartNodeSide);

         side = defaultStartNodeSide;
      }

      this.initialStanceFootPose.set(stanceFootPose);
      this.initialStanceFootPose.changeFrame(ReferenceFrame.getWorldFrame());
      this.initialStanceSide = side;
      this.lastStepSide = side;
      this.groundHeight = stanceFootPose.getZ();

      planStanceFootFrame.setPoseAndUpdate(initialStanceFootPose);

      robotStartPose.setToZero(planStanceFootFrame);
      robotStartPose.setY(lastStepSide.negateIfLeftSide(parameters.getIdealFootstepWidth() / 2.0));
      robotStartPose.changeFrame(ReferenceFrame.getWorldFrame());
      robotStartFrame.setPoseAndUpdate(robotStartPose);
   }

   public void setGoal(FootstepPlannerGoal goal)
   {
      setGoal(goal.getGoalPoseBetweenFeet());
   }

   public void setGoal(FramePose3DReadOnly goal)
   {
      this.goalPose.set(goal);
      this.goalPose.changeFrame(ReferenceFrame.getWorldFrame());
   }

   private final FootstepPlan footstepPlan = new FootstepPlan();

   public FootstepPlan getPlan()
   {
      return footstepPlan;
   }

   public FootstepPlanningResult plan()
   {
      lastStepSide = initialStanceSide;
      planStanceFootFrame.setPoseAndUpdate(initialStanceFootPose);

      FramePoint2D goalPoint = new FramePoint2D(goalPose.getPosition());

      List<FramePose2DReadOnly> footstepList = new ArrayList<>();

      if (isGoalOutOfReach())
      {
         // turn
         double headingTurnAngle = AngleTools.calculateHeading(robotStartPose, goalPoint, -robotStartPose.getYaw(), 0.0);
         double minTurn = AngleTools.computeAngleDifferenceMinusPiToPi(initialStanceFootPose.getYaw(), goalPose.getYaw());
         boolean walkingBackwards = Math.abs(headingTurnAngle) > Math.PI / 2.0 && Math.abs(minTurn) < Math.PI / 2.0;
         if (walkingBackwards)
            headingTurnAngle = headingTurnAngle > 0.0 ? headingTurnAngle - Math.PI : headingTurnAngle + Math.PI;

         addTurnInPlace(footstepList, headingTurnAngle, robotStartPose.getPosition());

         // walk
         double distanceToTravel = walkingBackwards ? -robotStartPose.getPositionDistance(goalPoint) : robotStartPose.getPositionDistance(goalPoint);
         double stepLength = walkingBackwards ? -Math.abs(parameters.getIdealBackStepLength()) : parameters.getIdealFootstepLength();
         addStraightWalk(footstepList, robotStartPose.getPosition(), distanceToTravel, stepLength);
      }
      else
      {
         addStepsAtGoal(footstepList);
      }

      // turn to desired final heading
      FramePose2D stanceFootPose = new FramePose2D(planStanceFootFrame);
      stanceFootPose.changeFrame(goalPose.getReferenceFrame());
      double headingTurnAngle = AngleTools.trimAngleMinusPiToPi(goalPose.getYaw() - stanceFootPose.getYaw());
      FramePoint2D pointToTurnAbout = new FramePoint2D(planStanceFootFrame, 0.0, lastStepSide.negateIfLeftSide(parameters.getIdealFootstepWidth() / 2.0));
      pointToTurnAbout.changeFrame(ReferenceFrame.getWorldFrame());
      addTurnInPlace(footstepList, headingTurnAngle, pointToTurnAbout);

      // square up
      addSquareUp(footstepList, pointToTurnAbout);

      footstepPlan.clear();
      RobotSide robotSide = initialStanceSide;
      for (FramePose2DReadOnly footstepPose2d : footstepList)
      {
         robotSide = robotSide.getOppositeSide();
         footstepPlan.addFootstep(robotSide, FlatGroundPlanningUtils.poseFormPose2d(footstepPose2d, groundHeight));
      }
      return FootstepPlanningResult.FOUND_SOLUTION;
   }

   private void addSquareUp(List<FramePose2DReadOnly> footstepListToPack, FramePoint2DReadOnly robotPosition)
   {
      FramePoint2D positionInStance = new FramePoint2D(robotPosition);
      positionInStance.changeFrame(planStanceFootFrame);
      if (Math.abs(positionInStance.getX()) > 0.001)
         throw new RuntimeException("Can not square up for given position.");

      FramePose2D footstepPose = new FramePose2D(planStanceFootFrame);
      footstepPose.setY(2.0 * positionInStance.getY());

      if (lastStepSide.equals(RobotSide.LEFT) && footstepPose.getY() > 0)
         throw new RuntimeException("Left foot can not be placed on right side of right foot");

      if (lastStepSide.equals(RobotSide.RIGHT) && footstepPose.getY() < 0)
         throw new RuntimeException("Right foot can not be placed on left side of left foot");

      footstepPose.changeFrame(ReferenceFrame.getWorldFrame());

      footstepListToPack.add(footstepPose);
      planStanceFootFrame.setPoseAndUpdate(footstepPose);
      lastStepSide = lastStepSide.getOppositeSide();
   }

   private void addStraightWalk(List<FramePose2DReadOnly> footstepListToPack,
                                FramePoint2DReadOnly startingPoint,
                                double distanceToTravel,
                                double nominalStepLength)
   {
      if (Math.abs(distanceToTravel) < epsilon)
         return;

      double straightSteps = Math.ceil(distanceToTravel / nominalStepLength);
      double stepLength = distanceToTravel / straightSteps;
      FramePoint2D planStart = new FramePoint2D();

      for (int i = 0; i < straightSteps; i++)
      {
         planStart.setIncludingFrame(startingPoint);
         planStart.changeFrame(planStanceFootFrame);

         FramePose2D nextFootStep = new FramePose2D(planStanceFootFrame);
         nextFootStep.setX(stepLength);
         nextFootStep.setY(lastStepSide.negateIfLeftSide(parameters.getIdealFootstepWidth()));

         if (lastStepSide.equals(RobotSide.LEFT) && nextFootStep.getY() > 0)
            throw new RuntimeException("Left foot can not be placed on right side of right foot");

         if (lastStepSide.equals(RobotSide.RIGHT) && nextFootStep.getY() < 0)
            throw new RuntimeException("Right foot can not be placed on left side of left foot");

         nextFootStep.changeFrame(ReferenceFrame.getWorldFrame());
         footstepListToPack.add(nextFootStep);
         planStanceFootFrame.setPoseAndUpdate(nextFootStep);
         lastStepSide = lastStepSide.getOppositeSide();
      }
   }

   private void addTurnInPlace(List<FramePose2DReadOnly> footstepListToPack, double turningAngle, FramePoint2DReadOnly pointToTurnAbout)
   {
      if (Math.abs(turningAngle) < epsilon)
         return;

      FramePoint2D pointToPlanFrom = new FramePoint2D(pointToTurnAbout);

      pointToPlanFrom.changeFrame(planStanceFootFrame);
      if (Math.abs(pointToPlanFrom.getX()) > 0.001)
         throw new RuntimeException("Can not turn in place around given point.");

      RobotSide sideToTurnTo = turningAngle >= 0.0 ? RobotSide.LEFT : RobotSide.RIGHT;

      double twoStepTurnAngle = -parameters.getMinStepYaw() + parameters.getMaxStepYaw();
      double requiredDoubleSteps = Math.abs(turningAngle / twoStepTurnAngle);

      int turningSteps = 2 * (int) Math.ceil(requiredDoubleSteps);
      double maxTurningAngle = Math.ceil(requiredDoubleSteps) * twoStepTurnAngle;
      boolean firstStepClosing = sideToTurnTo.equals(lastStepSide);
      if (firstStepClosing)
      {
         if (Math.floor(requiredDoubleSteps) * twoStepTurnAngle - parameters.getMinStepYaw() >= Math.abs(turningAngle))
         {
            turningSteps--;
            maxTurningAngle -= parameters.getMaxStepYaw();
         }
      }
      else
      {
         if (Math.floor(requiredDoubleSteps) * twoStepTurnAngle + parameters.getMaxStepYaw() >= Math.abs(turningAngle))
         {
            turningSteps--;
            maxTurningAngle += parameters.getMinStepYaw();
         }
      }
      double scaleTurningAngle = Math.abs(turningAngle) / maxTurningAngle;

      for (int i = 0; i < turningSteps; i++)
      {
         FramePose2D turningFramePose = new FramePose2D(planStanceFootFrame);
         pointToPlanFrom.setIncludingFrame(pointToTurnAbout);
         pointToPlanFrom.changeFrame(planStanceFootFrame);
         turningFramePose.setY(pointToPlanFrom.getY());

         if (sideToTurnTo.equals(lastStepSide))
         {
            turningFramePose.setYaw(sideToTurnTo.negateIfRightSide(-parameters.getMinStepYaw() * scaleTurningAngle));
         }
         else
         {
            turningFramePose.setYaw(sideToTurnTo.negateIfRightSide(parameters.getMaxStepYaw() * scaleTurningAngle));
         }
         turningFramePose.changeFrame(ReferenceFrame.getWorldFrame());
         turningFrame.setPoseAndUpdate(turningFramePose);

         FramePose2D nextFootstep = new FramePose2D(turningFrame);
         nextFootstep.setY(lastStepSide.negateIfLeftSide(parameters.getIdealFootstepWidth() / 2.0));

         if (lastStepSide.equals(RobotSide.LEFT) && nextFootstep.getY() > 0)
            throw new RuntimeException("Left foot can not be placed on right side of right foot");

         if (lastStepSide.equals(RobotSide.RIGHT) && nextFootstep.getY() < 0)
            throw new RuntimeException("Right foot can not be placed on left side of left foot");

         nextFootstep.changeFrame(ReferenceFrame.getWorldFrame());

         footstepListToPack.add(nextFootstep);
         planStanceFootFrame.setPoseAndUpdate(nextFootstep);
         lastStepSide = lastStepSide.getOppositeSide();
      }
   }

   private void addStepsAtGoal(List<FramePose2DReadOnly> footstepListToPack)
   {
      // step directly to the goal position, in the current yaw
      FramePose2D firstStepPose = new FramePose2D(goalPose);
      firstStepPose.changeFrame(planStanceFootFrame);
      firstStepPose.getOrientation().setToZero();
      firstStepPose.getPosition().addY(lastStepSide.negateIfLeftSide(parameters.getIdealFootstepWidth() / 2.0));
      firstStepPose.changeFrame(ReferenceFrame.getWorldFrame());

      footstepListToPack.add(firstStepPose);

      planStanceFootFrame.setPoseAndUpdate(firstStepPose);
      lastStepSide = lastStepSide.getOppositeSide();

      FramePose2D secondStepPose = new FramePose2D(planStanceFootFrame);
      secondStepPose.getPosition().setY(lastStepSide.negateIfLeftSide(parameters.getIdealFootstepWidth()));
      secondStepPose.changeFrame(ReferenceFrame.getWorldFrame());

      footstepListToPack.add(secondStepPose);

      planStanceFootFrame.setPoseAndUpdate(secondStepPose);
      lastStepSide = lastStepSide.getOppositeSide();
   }

   private boolean isGoalOutOfReach()
   {
      FramePose2D goalRelativeToStart = new FramePose2D(goalPose);
      goalRelativeToStart.changeFrame(robotStartFrame);
      boolean steppingLeft = goalRelativeToStart.getY() > 0.0;
      double xIdealDistance = goalRelativeToStart.getX() > 0.0 ? parameters.getIdealFootstepLength() : parameters.getIdealBackStepLength();
      double yIdealDistance = steppingLeft && lastStepSide == RobotSide.RIGHT ? parameters.getMaxStepWidth() : parameters.getMinStepWidth();
      double normalizedDistance = MathTools.square(goalRelativeToStart.getX() / xIdealDistance) + MathTools.square(goalRelativeToStart.getY() / yIdealDistance);

      if (normalizedDistance > 1.0)
         return true;

      if (goalRelativeToStart.getPosition().distanceFromOriginSquared() > parameters.getMaxStepReach())
         return true;

      return false;
   }

   private static class DefaultTurnWalkTurnPlannerParameters extends DefaultFootstepPlannerParameters
   {
      public DefaultTurnWalkTurnPlannerParameters()
      {
         setIdealFootstepWidth(0.3);
         setIdealFootstepLength(0.45);
         setMaxStepYaw(Math.toRadians(20.0));
         setMinStepYaw(Math.toRadians(0.0));
         setMaxStepReach(0.45);
         setMinStepWidth(0.0);
         setMaxStepZ(0.0);
         setMaxStepWidth(0.3);
      }
   }

}
