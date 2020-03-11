package us.ihmc.valkyrie.planner;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.collision.BodyCollisionData;
import us.ihmc.footstepPlanning.graphSearch.collision.FootstepNodeBodyCollisionDetector;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.ObstacleBetweenNodesChecker;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.PlanarRegionBaseOfCliffAvoider;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.TransformReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.valkyrie.planner.log.ValkyriePlannerEdgeData;

import java.util.List;
import java.util.function.UnaryOperator;

public class ValkyrieFootstepValidityChecker
{
   private final ValkyriePlannerEdgeData edgeData;
   private final FootstepNodeSnapper snapper;
   private final SideDependentList<ConvexPolygon2D> footPolygons;
   private final ValkyrieAStarFootstepPlannerParameters parameters;
   private final PlanarRegionBaseOfCliffAvoider cliffAvoider;
   private final ObstacleBetweenNodesChecker obstacleBetweenNodesChecker;
   private final FootstepNodeBodyCollisionDetector collisionDetector;

   private final TransformReferenceFrame startOfSwingFrame = new TransformReferenceFrame("startOfSwingFrame", ReferenceFrame.getWorldFrame());
   private final TransformReferenceFrame stanceFootFrame = new TransformReferenceFrame("stanceFootFrame", ReferenceFrame.getWorldFrame());
   private final TransformReferenceFrame candidateFootFrame = new TransformReferenceFrame("candidateFootFrame", ReferenceFrame.getWorldFrame());
   private final ZUpFrame startOfSwingZUpFrame = new ZUpFrame(ReferenceFrame.getWorldFrame(), startOfSwingFrame, "startOfSwingZUpFrame");
   private final ZUpFrame stanceFootZUpFrame = new ZUpFrame(ReferenceFrame.getWorldFrame(), stanceFootFrame, "stanceFootZUpFrame");
   private final FramePoint3D candidateFootPosition = new FramePoint3D();

   private UnaryOperator<FootstepNode> parentNodeSupplier = null;
   private PlanarRegionsList planarRegionsList = null;

   // Variables to log
   private double stepWidth;
   private double stepLength;
   private double stepHeight;
   private double stepReach;
   private double footAreaPercentage;
   private final FootstepNodeSnapData candidateNodeSnapData = FootstepNodeSnapData.identityData();

   public ValkyrieFootstepValidityChecker(ValkyrieAStarFootstepPlannerParameters parameters,
                                          SideDependentList<ConvexPolygon2D> footPolygons,
                                          FootstepNodeSnapper snapper,
                                          ValkyriePlannerEdgeData edgeData)
   {
      this.edgeData = edgeData;
      this.parameters = parameters;
      this.footPolygons = footPolygons;
      this.snapper = snapper;
      this.cliffAvoider = new PlanarRegionBaseOfCliffAvoider(snapper,
                                                             footPolygons,
                                                             parameters::getMinimumDistanceFromCliffBottoms,
                                                             parameters::getCliffHeightToAvoid);
      this.obstacleBetweenNodesChecker = new ObstacleBetweenNodesChecker(snapper,
                                                                         parameters::getCheckForPathCollisions,
                                                                         parameters::getIdealFootstepWidth,
                                                                         parameters.getBodyBoxOffset()::getZ,
                                                                         parameters.getBodyBoxDimensions()::getZ);
      this.collisionDetector = new FootstepNodeBodyCollisionDetector(parameters.getBodyBoxDimensions()::getX,
                                                                     parameters.getBodyBoxDimensions()::getY,
                                                                     parameters.getBodyBoxDimensions()::getZ,
                                                                     () -> 0.0,
                                                                     parameters.getBodyBoxOffset()::getX,
                                                                     parameters.getBodyBoxOffset()::getY,
                                                                     parameters.getBodyBoxOffset()::getZ);
   }

   public boolean checkFootstep(FootstepNode candidateNode, FootstepNode stanceNode)
   {
      clearLoggedVariables();
      StepRejectionReason rejectionReason = checkFootstepInternal(candidateNode, stanceNode);

      if(edgeData.getStanceNode() == null)
         logVariables(candidateNode, stanceNode, rejectionReason);

      return rejectionReason == null;
   }

   private StepRejectionReason checkFootstepInternal(FootstepNode candidateNode, FootstepNode stanceNode)
   {
      if(planarRegionsList == null)
      {
         return null;
      }

      candidateNodeSnapData.set(snapper.snapFootstepNode(candidateNode));

      // Check valid snap
      if (candidateNodeSnapData.getSnapTransform().containsNaN())
      {
         return StepRejectionReason.COULD_NOT_SNAP;
      }

      // Check incline
      RigidBodyTransform snappedSoleTransform = candidateNodeSnapData.getOrComputeSnappedNodeTransform(candidateNode);
      double minimumSurfaceNormalZ = Math.cos(parameters.getMaximumSurfanceInclineRadians());
      if (snappedSoleTransform.getM22() < minimumSurfaceNormalZ)
      {
         return StepRejectionReason.INCLINE_TOO_STEEP;
      }

      // Check snap area
      ConvexPolygon2D footholdAfterSnap = candidateNodeSnapData.getCroppedFoothold();
      double croppedFootArea = footholdAfterSnap.getArea();
      double fullFootArea = footPolygons.get(candidateNode.getRobotSide()).getArea();
      footAreaPercentage = croppedFootArea / fullFootArea;

      double epsilonAreaPercentage = 1e-4;
      if (!footholdAfterSnap.isEmpty() && footAreaPercentage < (parameters.getMinimumFootholdPercent() - epsilonAreaPercentage))
      {
         return StepRejectionReason.AREA_TOO_SMALL;
      }

      if(stanceNode == null)
      {
         return null;
      }

      FootstepNodeSnapData stanceNodeSnapData = snapper.snapFootstepNode(stanceNode);
      candidateFootFrame.setTransformAndUpdate(candidateNodeSnapData.getOrComputeSnappedNodeTransform(candidateNode));
      stanceFootFrame.setTransformAndUpdate(stanceNodeSnapData.getOrComputeSnappedNodeTransform(stanceNode));
      stanceFootZUpFrame.update();

      candidateFootPosition.setToZero(candidateFootFrame);
      candidateFootPosition.changeFrame(stanceFootZUpFrame);

      stepLength = candidateFootPosition.getX();
      stepWidth = candidateNode.getRobotSide().negateIfRightSide(candidateFootPosition.getY());
      stepReach = EuclidGeometryTools.pythagorasGetHypotenuse(Math.abs(candidateFootPosition.getX()),
                                                                     Math.abs(stepWidth - parameters.getIdealFootstepWidth()));
      stepHeight = candidateFootPosition.getZ();

      // Check step height
      if (Math.abs(stepHeight) > parameters.getMaximumStepZ())
      {
         return StepRejectionReason.HEIGHT_TOO_LOW_OR_HIGH;
      }

      // Check step width and reach
      boolean flatGround = MathTools.intervalContains(stepHeight,
                                                      parameters.getFlatGroundLowerThreshold(),
                                                      parameters.getFlatGroundUpperThreshold());
      if (flatGround)
      {
         if (stepReach > parameters.getMaximumStepReach())
         {
            return StepRejectionReason.REACH_EXCEEDED;
         }
         else if (!MathTools.intervalContains(stepWidth, parameters.getMinimumStepWidth(), parameters.getMaximumStepWidth()))
         {
            return StepRejectionReason.WIDTH_TOO_SMALL_OR_LARGE;
         }
      }
      // Stepping up
      else if (stepHeight >= parameters.getFlatGroundUpperThreshold())
      {
         if (stepReach > parameters.getMaximumStepReachWhenSteppingUp())
         {
            return StepRejectionReason.REACH_EXCEEDED;
         }
         else if (!MathTools.intervalContains(stepWidth, parameters.getMinimumStepWidth(), parameters.getMaximumStepWidthWhenSteppingUp()))
         {
            return StepRejectionReason.WIDTH_TOO_SMALL_OR_LARGE;
         }
      }
      // Stepping down
      else
      {
         if (stepReach > parameters.getMaximumStepReachWhenSteppingDown())
         {
            return StepRejectionReason.REACH_EXCEEDED;
         }
         else if (!MathTools.intervalContains(stepWidth, parameters.getMinimumStepWidth(), parameters.getMaximumStepWidthWhenSteppingDown()))
         {
            return StepRejectionReason.WIDTH_TOO_SMALL_OR_LARGE;
         }
      }

      // Check reach from start of swing
      FootstepNode grandParentNode;
      FootstepNodeSnapData grandparentNodeSnapData;
      double alpha = parameters.getTranslationScaleFromGrandparentNode();
      if (alpha > 0.0 && parentNodeSupplier != null && (grandParentNode = parentNodeSupplier.apply(stanceNode)) != null
          && (grandparentNodeSnapData = snapper.snapFootstepNode(grandParentNode)) != null)
      {
         startOfSwingFrame.setTransformAndUpdate(grandparentNodeSnapData.getOrComputeSnappedNodeTransform(grandParentNode));
         startOfSwingZUpFrame.update();
         candidateFootPosition.changeFrame(startOfSwingZUpFrame);

         double swingReach = EuclidGeometryTools.pythagorasGetHypotenuse(Math.abs(candidateFootPosition.getX()), Math.abs(candidateFootPosition.getY()));

         if (stepHeight > parameters.getFlatGroundUpperThreshold())
         {
            if (swingReach > alpha * parameters.getMaximumStepReachWhenSteppingUp())
            {
               return StepRejectionReason.SWING_REACH_EXCEEDED;
            }
         }
         else if (stepHeight < parameters.getFlatGroundLowerThreshold())
         {
            if (swingReach > alpha * parameters.getMaximumStepReachWhenSteppingDown())
            {
               return StepRejectionReason.SWING_REACH_EXCEEDED;
            }
         }
      }

      // Check for ankle collision
      cliffAvoider.setPlanarRegionsList(planarRegionsList);
      if(!cliffAvoider.isNodeValid(candidateNode))
      {
         return StepRejectionReason.TOO_CLOSE_TO_LEDGE;
      }

      // Check for obstacle collisions (vertically extruded line between steps)
      if (parameters.getCheckForPathCollisions())
      {
         obstacleBetweenNodesChecker.setPlanarRegions(planarRegionsList);
         try
         {
            if (!obstacleBetweenNodesChecker.isNodeValid(candidateNode, stanceNode))
            {
               return StepRejectionReason.OBSTACLE_BETWEEN_STEPS;
            }
         }
         catch(Exception e)
         {
            return StepRejectionReason.OBSTACLE_BETWEEN_STEPS;
         }
      }

      // Check for bounding box collision
      if (parameters.getCheckForBodyBoxCollisions())
      {
         collisionDetector.setPlanarRegionsList(planarRegionsList);
         List<BodyCollisionData> bodyCollisionData = collisionDetector.checkForCollision(candidateNode,
                                                                                         stanceNode,
                                                                                         candidateNodeSnapData.getSnapTransform().getTranslationZ(),
                                                                                         stanceNodeSnapData.getSnapTransform().getTranslationZ(),
                                                                                         parameters.getNumberOfBoundingBoxChecks());
         for (int i = 0; i < bodyCollisionData.size(); i++)
         {
            if (bodyCollisionData.get(i).isCollisionDetected())
            {
               return StepRejectionReason.BOUNDING_BOX_COLLISION;
            }
         }
      }

      return null;
   }

   private void logVariables(FootstepNode candidateNode, FootstepNode stanceNode, StepRejectionReason rejectionReason)
   {
      edgeData.setStanceNode(stanceNode);
      edgeData.setCandidateNode(candidateNode);
      edgeData.setCandidateNodeSnapData(candidateNodeSnapData);

      edgeData.setStepWidth(stepWidth);
      edgeData.setStepLength(stepLength);
      edgeData.setStepHeight(stepHeight);
      edgeData.setStepReach(stepReach);
      edgeData.setFootAreaPercentage(footAreaPercentage);
      edgeData.setRejectionReason(rejectionReason);
   }

   private void clearLoggedVariables()
   {
      stepWidth = Double.NaN;
      stepLength = Double.NaN;
      stepHeight = Double.NaN;
      stepReach = Double.NaN;
      footAreaPercentage = Double.NaN;
      candidateNodeSnapData.clear();
   }

   public enum StepRejectionReason
   {
      COULD_NOT_SNAP, INCLINE_TOO_STEEP, AREA_TOO_SMALL, HEIGHT_TOO_LOW_OR_HIGH, REACH_EXCEEDED, WIDTH_TOO_SMALL_OR_LARGE, SWING_REACH_EXCEEDED, TOO_CLOSE_TO_LEDGE, OBSTACLE_BETWEEN_STEPS, BOUNDING_BOX_COLLISION
   }

   public void setPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;
   }

   public void setParentNodeSupplier(UnaryOperator<FootstepNode> parentNodeSupplier)
   {
      this.parentNodeSupplier = parentNodeSupplier;
   }
}
