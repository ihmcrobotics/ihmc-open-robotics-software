package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeExpansion;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.CliffDetectionTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.HashSet;

public class VariableResolutionNodeExpansion extends ParameterBasedNodeExpansion
{
   private final FootstepNodeSnapper snapper;

   public VariableResolutionNodeExpansion(FootstepPlannerParameters parameters, QuadrupedXGaitSettingsReadOnly xGaitSettings, FootstepNodeSnapper snapper)
   {
      super(parameters, xGaitSettings);
      this.snapper = snapper;
   }

   @Override
   public HashSet<FootstepNode> expandNode(FootstepNode node)
   {
      HashSet<FootstepNode> expansion = new HashSet<>();
      double resolution = getExpansionResolution(node);
      addDefaultFootsteps(node, expansion, resolution);

      return expansion;
   }

   private double getExpansionResolution(FootstepNode node)
   {
      if (!snapper.hasPlanarRegions())
         return  2 * FootstepNode.gridSizeXY;

      RobotQuadrant movingQuadrant = node.getMovingQuadrant();
      int xIndex = node.getXIndex(movingQuadrant);
      int yIndex = node.getYIndex(movingQuadrant);
      RigidBodyTransform footTransformToWorld = new RigidBodyTransform();
      FootstepNodeTools.getSnappedNodeTransformToWorld(xIndex, yIndex, snapper.snapFootstepNode(xIndex, yIndex).getSnapTransform(), footTransformToWorld);

      Point3D footInWorld = new Point3D();
      footTransformToWorld.transform(footInWorld);

      boolean isMovingFront = movingQuadrant.isQuadrantInFront();
      boolean isMovingLeft = movingQuadrant.isQuadrantOnLeftSide();

      double forward = isMovingFront ? parameters.getMaximumFrontStepLength() : parameters.getMaximumHindStepLength();
      double backward = isMovingFront ? parameters.getMinimumFrontStepLength() : parameters.getMinimumHindStepLength();
      double left = isMovingLeft ? parameters.getMaximumStepOutward() : -parameters.getMaximumStepInward();
      double right = isMovingLeft ? parameters.getMaximumStepInward() : -parameters.getMaximumStepOutward();

      if (CliffDetectionTools.isNearCliff(snapper.getPlanarRegionsList(), footInWorld, node.getStepYaw(), parameters.getCliffHeightToAvoid(), forward,
                                          backward, left, right))
         return FootstepNode.gridSizeXY;

      return 2 * FootstepNode.gridSizeXY;
   }


}
