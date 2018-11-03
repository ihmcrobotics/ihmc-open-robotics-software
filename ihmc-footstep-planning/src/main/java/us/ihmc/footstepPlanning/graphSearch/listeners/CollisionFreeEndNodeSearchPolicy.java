package us.ihmc.footstepPlanning.graphSearch.listeners;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.BodyCollisionNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class CollisionFreeEndNodeSearchPolicy implements PlannerHeuristicNodeSearchPolicy, StartAndGoalListener
{
   private final static double distanceFromStartToConsiderSearching = 1.0;
   private final static double distanceFromGoalToConsiderSearching = 1.0;

   private final static double rotationIncrement = Math.toRadians(10.0);
   private final static double minimumRotation = Math.toRadians(30.0);

   private final static double maxRotation = Math.PI / 2.0;

   private final List<PlannerHeuristicNodeActionPolicy> actionPolicies = new ArrayList<>();

   private final FramePose3D initialPose = new FramePose3D();
   private final FramePose3D finalPose = new FramePose3D();

   private final AtomicReference<FootstepNode> validNode = new AtomicReference<>();
   private final AtomicReference<FootstepNode> parentOfValidNode = new AtomicReference<>();

   private final FootstepPlannerParameters parameters;
   private final BodyCollisionNodeChecker collisionNodeChecker;
   private double currentRotation = 0.0;

   public CollisionFreeEndNodeSearchPolicy(BodyCollisionNodeChecker collisionNodeChecker, FootstepPlannerParameters parameters)
   {
      this.collisionNodeChecker = collisionNodeChecker;
      this.parameters = parameters;
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

   @Override
   public boolean performSearchForValidNode(FootstepNode rejectedNode, FootstepNode parentNode)
   {
      if (!checkFarEnoughFromStart(rejectedNode) || !checkFarEnoughFromEnd(rejectedNode))
         return false;

      currentRotation = 0.0;
      boolean newNodeIsValid = false;
      validNode.set(null);
      parentOfValidNode.set(null);
      FootstepNode newNode = null;
      while ((maxRotation > currentRotation) && !newNodeIsValid)
      {
         currentRotation += rotationIncrement;
         newNode = new FootstepNode(rejectedNode.getX(), rejectedNode.getY(), rejectedNode.getYaw() + currentRotation, rejectedNode.getRobotSide());
         newNodeIsValid = collisionNodeChecker.isNodeValidInternal(newNode, parentNode, 1.0);
      }

      if (newNodeIsValid && Math.abs(currentRotation) > minimumRotation)
      {
         validNode.set(newNode);
         parentOfValidNode.set(parentNode);
         return true;
      }

      currentRotation = 0.0;
      while ((currentRotation > -maxRotation) && !newNodeIsValid)
      {
         currentRotation -= rotationIncrement;
         newNode = new FootstepNode(rejectedNode.getX(), rejectedNode.getY(), rejectedNode.getYaw() + currentRotation, rejectedNode.getRobotSide());
         newNodeIsValid = collisionNodeChecker.isNodeValidInternal(newNode, parentNode, 1.0);
      }

      if (newNodeIsValid && Math.abs(currentRotation) > minimumRotation)
      {
         validNode.set(newNode);
         parentOfValidNode.set(parentNode);
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
      actionPolicies.parallelStream().forEach(actionPolicy -> actionPolicy.performActionForNewValidNode(pollNewValidNode(), pollNewValidParentNode()));
   }
}
