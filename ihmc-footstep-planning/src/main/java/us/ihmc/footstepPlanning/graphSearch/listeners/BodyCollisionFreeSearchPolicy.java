package us.ihmc.footstepPlanning.graphSearch.listeners;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.BodyCollisionNodeChecker;

import java.util.ArrayList;
import java.util.List;

public class BodyCollisionFreeSearchPolicy implements FootstepPlannerHeuristicSearchPolicy, StartAndGoalListener
{
   private final static double distanceFromStartToConsiderSearching = 1.0;
   private final static double distanceFromGoalToConsiderSearching = 1.0;

   private final static double rotationIncrement = Math.toRadians(10.0);

   private final List<FootstepPlannerHeuristicActionPolicy> actionPolicies = new ArrayList<>();

   private final FramePose3D initialPose = new FramePose3D();
   private final FramePose3D finalPose = new FramePose3D();

   private FootstepNode validNode = null;
   private FootstepNode parentOfValidNode = null;
   private boolean hasNewValidNode = false;

   private final BodyCollisionNodeChecker collisionNodeChecker;
   private double currentRotation = 0.0;

   public BodyCollisionFreeSearchPolicy(BodyCollisionNodeChecker collisionNodeChecker)
   {
      this.collisionNodeChecker = collisionNodeChecker;
   }

   @Override
   public void attachActionPolicy(FootstepPlannerHeuristicActionPolicy actionPolicy)
   {
      this.actionPolicies.add(actionPolicy);
   }

   @Override
   public void setStartPose(FramePose3D initialPose)
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

   @Override
   public boolean performSearchForNewNode(FootstepNode rejectedNode, FootstepNode parentNode)
   {
      if (!checkFarEnoughFromStart(rejectedNode) || !checkFarEnoughFromEnd(rejectedNode))
         return false;

      currentRotation = 0.0;
      boolean newNodeIsValid = false;
      hasNewValidNode = false;
      validNode = null;
      parentOfValidNode = null;
      FootstepNode newNode = null;
      while ((Math.PI / 2 > currentRotation) && !newNodeIsValid)
      {
         currentRotation += rotationIncrement;
         newNode = new FootstepNode(rejectedNode.getX(), rejectedNode.getY(), rejectedNode.getYaw() + currentRotation, rejectedNode.getRobotSide());
         newNodeIsValid = collisionNodeChecker.isNodeValidInternal(newNode, parentNode);
      }

      if (newNodeIsValid && Math.abs(currentRotation) > Math.toRadians(30))
      {
         hasNewValidNode = true;
         validNode = newNode;
         parentOfValidNode = parentNode;
         return true;
      }

      currentRotation = 0.0;
      while ((currentRotation > -Math.PI / 2) && !newNodeIsValid)
      {
         currentRotation -= rotationIncrement;
         newNode = new FootstepNode(rejectedNode.getX(), rejectedNode.getY(), rejectedNode.getYaw() + currentRotation, rejectedNode.getRobotSide());
         newNodeIsValid = collisionNodeChecker.isNodeValidInternal(newNode, parentNode);
      }

      if (newNodeIsValid && Math.abs(currentRotation) > Math.toRadians(30))
      {
         hasNewValidNode = true;
         validNode = newNode;
         parentOfValidNode = parentNode;
         return true;
      }

      return false;
   }

   @Override
   public FootstepNode pollNewValidNode()
   {
      FootstepNode nodeToReturn;
      if (hasNewValidNode)
      {
         hasNewValidNode = false;
         nodeToReturn = validNode;
      }
      else
      {
         nodeToReturn = null;
      }

      return nodeToReturn;
   }

   @Override
   public void performActionPoliciesForNewNode()
   {
      for (FootstepPlannerHeuristicActionPolicy actionPolicy : actionPolicies)
         actionPolicy.performActionFromNewNode(pollNewValidNode(), parentOfValidNode);
   }
}
