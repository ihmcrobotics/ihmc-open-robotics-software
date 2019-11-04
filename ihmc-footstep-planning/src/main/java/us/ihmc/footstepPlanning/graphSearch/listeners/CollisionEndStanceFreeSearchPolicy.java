package us.ihmc.footstepPlanning.graphSearch.listeners;

import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.BodyCollisionNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class CollisionEndStanceFreeSearchPolicy implements PlannerHeuristicNodeSearchPolicy, StartAndGoalListener
{
   private final static double distanceFromStartToConsiderSearching = 1.0;
   private final static double distanceFromGoalToConsiderSearching = 1.0;

   private final static double rotationIncrement = Math.toRadians(10.0);
   private final static double minimumRotation = Math.toRadians(30.0);

   private final static double biasTowardsEnd = 0.8;

   private final static double maxRotation = Math.PI / 2.0;

   private final List<PlannerHeuristicNodeActionPolicy> actionPolicies = new ArrayList<>();

   private final FramePose3D initialPose = new FramePose3D();
   private final FramePose3D finalPose = new FramePose3D();

   private final AtomicReference<FootstepNode> validNode = new AtomicReference<>();
   private final AtomicReference<FootstepNode> parentOfValidNode = new AtomicReference<>();

   private final FootstepPlannerParametersReadOnly parameters;
   private final BodyCollisionNodeChecker collisionNodeChecker;
   private final FootstepNodeSnapper snapper;

   public CollisionEndStanceFreeSearchPolicy(BodyCollisionNodeChecker collisionNodeChecker, FootstepNodeSnapper snapper, FootstepPlannerParametersReadOnly parameters)
   {
      this.collisionNodeChecker = collisionNodeChecker;
      this.parameters = parameters;
      this.snapper = snapper;
   }

   @Override
   public void attachActionPolicy(PlannerHeuristicNodeActionPolicy actionPolicy)
   {
      this.actionPolicies.add(actionPolicy);
   }

   @Override
   public void setInitialPose(FramePose3D initialPose)
   {
      this.initialPose.set(initialPose);
   }

   @Override
   public void setGoalPose(FramePose3D finalPose)
   {
      this.finalPose.set(finalPose);
   }

   private final Point2D tempPoint2D = new Point2D();

   private boolean checkFarEnoughFromStart(FootstepNode rejectedNode)
   {
      tempPoint2D.set(rejectedNode.getX(), rejectedNode.getY());

      return initialPose.getPosition().distanceXY(tempPoint2D) > distanceFromStartToConsiderSearching;
   }

   private boolean checkFarEnoughFromEnd(FootstepNode rejectedNode)
   {
      tempPoint2D.set(rejectedNode.getX(), rejectedNode.getY());

      return finalPose.getPosition().distanceXY(tempPoint2D) > distanceFromGoalToConsiderSearching;
   }

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final PoseReferenceFrame midStanceFrame = new PoseReferenceFrame("midStanceFrame", worldFrame);

   @Override
   public boolean performSearchForValidNode(FootstepNode rejectedNode, FootstepNode parentNode)
   {
      if (!parameters.performHeuristicSearchPolicies() || !checkFarEnoughFromStart(rejectedNode) || !checkFarEnoughFromEnd(rejectedNode))
         return false;

      double currentRotation = 0.0;
      boolean newNodeIsValid = false;
      validNode.set(null);
      parentOfValidNode.set(null);

      double newX = InterpolationTools.linearInterpolate(parentNode.getX(), rejectedNode.getX(), biasTowardsEnd);
      double newY = InterpolationTools.linearInterpolate(parentNode.getY(), rejectedNode.getY(), biasTowardsEnd);
      double averageYaw = AngleTools.interpolateAngle(parentNode.getYaw(), rejectedNode.getYaw(), biasTowardsEnd);

      FramePose3D footPose = new FramePose3D();
      footPose.setPosition(newX, newY, 0.0);
      footPose.setOrientationYawPitchRoll(averageYaw, 0.0, 0.0);

      PoseReferenceFrame poseFrame = new PoseReferenceFrame("poseFrame", worldFrame);
      poseFrame.setPoseAndUpdate(footPose);

      FramePose3D midPose = new FramePose3D(poseFrame);
      midPose.setY(0.5 * rejectedNode.getRobotSide().negateIfLeftSide(parameters.getIdealFootstepWidth()));
      midPose.changeFrame(worldFrame);

      FramePose3D leftFoot = new FramePose3D();
      FramePose3D rightFoot = new FramePose3D();

      FootstepNode newNode = null;
      FootstepNode newParentNode = null;

      currentRotation = 0.0;
      double rotationDirection = rejectedNode.getRobotSide().negateIfLeftSide(1.0);

      while (MathTools.intervalContains(currentRotation, maxRotation) && !newNodeIsValid)
      {
         currentRotation += rotationDirection * rotationIncrement;

         midPose.setOrientationYawPitchRoll(averageYaw + currentRotation, 0.0, 0.0);
         midStanceFrame.setPoseAndUpdate(midPose);

         leftFoot.setToZero(midStanceFrame);
         rightFoot.setToZero(midStanceFrame);

         leftFoot.setY(0.5 * parameters.getIdealFootstepWidth());
         rightFoot.setY(-0.5 * parameters.getIdealFootstepWidth());

         leftFoot.changeFrame(worldFrame);
         rightFoot.changeFrame(worldFrame);

         if (rejectedNode.getRobotSide().equals(RobotSide.LEFT))
         {
            newNode = new FootstepNode(leftFoot.getX(), leftFoot.getY(), leftFoot.getYaw(), RobotSide.LEFT);
            newParentNode = new FootstepNode(rightFoot.getX(), rightFoot.getY(), rightFoot.getYaw(), RobotSide.RIGHT);
         }
         else
         {
            newNode = new FootstepNode(rightFoot.getX(), rightFoot.getY(), rightFoot.getYaw(), RobotSide.RIGHT);
            newParentNode = new FootstepNode(leftFoot.getX(), leftFoot.getY(), leftFoot.getYaw(), RobotSide.LEFT);
         }

         snapper.snapFootstepNode(newNode);
         snapper.snapFootstepNode(newParentNode);

         boolean newNodeSnapIsValid = !snapper.snapFootstepNode(newNode).getSnapTransform().containsNaN();
         boolean newParentNodeSnapIsValid = !snapper.snapFootstepNode(newParentNode).getSnapTransform().containsNaN();
         newNodeIsValid = newNodeSnapIsValid && newParentNodeSnapIsValid && collisionNodeChecker.isNodeValid(newNode, newParentNode);
      }

      if (newNodeIsValid && Math.abs(currentRotation) > minimumRotation)
      {
         validNode.set(newNode);
         parentOfValidNode.set(newParentNode);
         return true;
      }

      currentRotation = 0.0;
      rotationDirection = -rotationDirection;

      while (MathTools.intervalContains(currentRotation, maxRotation)  && !newNodeIsValid)
      {
         currentRotation += rotationDirection * rotationIncrement;
         midPose.setOrientationYawPitchRoll(averageYaw + currentRotation, 0.0, 0.0);
         midStanceFrame.setPoseAndUpdate(midPose);

         leftFoot.setToZero(midStanceFrame);
         rightFoot.setToZero(midStanceFrame);

         leftFoot.setY(0.5 * parameters.getIdealFootstepWidth());
         rightFoot.setY(-0.5 * parameters.getIdealFootstepWidth());

         leftFoot.changeFrame(worldFrame);
         rightFoot.changeFrame(worldFrame);

         if (rejectedNode.getRobotSide().equals(RobotSide.LEFT))
         {
            newNode = new FootstepNode(leftFoot.getX(), leftFoot.getY(), leftFoot.getYaw(), RobotSide.LEFT);
            newParentNode = new FootstepNode(rightFoot.getX(), rightFoot.getY(), rightFoot.getYaw(), RobotSide.RIGHT);
         }
         else
         {
            newNode = new FootstepNode(rightFoot.getX(), rightFoot.getY(), rightFoot.getYaw(), RobotSide.RIGHT);
            newParentNode = new FootstepNode(leftFoot.getX(), leftFoot.getY(), leftFoot.getYaw(), RobotSide.LEFT);
         }

         boolean newNodeSnapIsValid = !snapper.snapFootstepNode(newNode).getSnapTransform().containsNaN();
         boolean newParentNodeSnapIsValid = !snapper.snapFootstepNode(newParentNode).getSnapTransform().containsNaN();
         newNodeIsValid = newNodeSnapIsValid && newParentNodeSnapIsValid && collisionNodeChecker.isNodeValid(newNode, newParentNode);
      }

      if (newNodeIsValid && Math.abs(currentRotation) > minimumRotation)
      {
         validNode.set(newNode);
         parentOfValidNode.set(newParentNode);
         return true;
      }

      return false;
   }

   @Override
   public FootstepNode pollNewValidNode()
   {
      return validNode.getAndSet(null);
   }

   @Override
   public FootstepNode pollNewValidParentNode()
   {
      return parentOfValidNode.getAndSet(null);
   }

   @Override
   public void executeActionPoliciesForNewValidNode()
   {
      FootstepNode newNode = pollNewValidNode();
      FootstepNode newParentNode = pollNewValidParentNode();

      for (PlannerHeuristicNodeActionPolicy actionPolicy : actionPolicies)
         actionPolicy.performActionForNewValidNode(newNode, newParentNode);
   }
}
