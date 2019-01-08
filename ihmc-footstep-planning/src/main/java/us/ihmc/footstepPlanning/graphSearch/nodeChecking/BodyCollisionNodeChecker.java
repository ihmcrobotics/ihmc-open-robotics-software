package us.ihmc.footstepPlanning.graphSearch.nodeChecking;

import java.util.HashMap;
import java.util.List;

import gnu.trove.map.TIntObjectMap;
import gnu.trove.map.hash.TIntObjectHashMap;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.shape.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.geometry.polytope.ConvexPolytope;
import us.ihmc.geometry.polytope.GilbertJohnsonKeerthiCollisionDetector;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.TranslationReferenceFrame;

public class BodyCollisionNodeChecker extends FootstepNodeChecker
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double planarRegionsFilterRadius = 2.0;

   private final PoseReferenceFrame midStanceFrame = new PoseReferenceFrame("midStanceFrame", worldFrame);
   private final TranslationReferenceFrame bodyCollisionFrame = new TranslationReferenceFrame("bodyCollisionFrame", midStanceFrame);
   private final Box3D bodyCollisionBox;
   private final ConvexPolytope bodyCollisionPolytope = new ConvexPolytope();
   private final FootstepNodeSnapperReadOnly snapper;

   private final TIntObjectMap<List<PlanarRegion>> nearbyPlanarRegions = new TIntObjectHashMap<>();

   private final GilbertJohnsonKeerthiCollisionDetector collisionDetector = new GilbertJohnsonKeerthiCollisionDetector();

   private final HashMap<PlanarRegion, ConvexPolytope> planarRegionPolytopes = new HashMap<>();

   private final FootstepPlannerParameters parameters;
   private final Vector3D bodyBoxDimensions = new Vector3D();

   private final Point2D tempPoint2d1 = new Point2D();
   private final Point2D tempPoint2d2 = new Point2D();
   private final Point3D tempPoint1 = new Point3D();
   private final Point3D tempPoint2 = new Point3D();
   private final FramePoint3D tempFramePoint = new FramePoint3D();
   private final FrameVector3D tempVector = new FrameVector3D();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();

   public BodyCollisionNodeChecker(FootstepPlannerParameters parameters, FootstepNodeSnapperReadOnly snapper)
   {
      this.parameters = parameters;
      this.snapper = snapper;

      bodyCollisionBox = new Box3D();
      bodyCollisionBox.setSize(parameters.getBodyBoxDepth(), parameters.getBodyBoxWidth(), parameters.getBodyBoxHeight());

      bodyCollisionFrame.updateTranslation(
            new Vector3D(parameters.getBodyBoxBaseX(), parameters.getBodyBoxBaseY(), parameters.getBodyBoxBaseZ() + 0.5 * parameters.getBodyBoxHeight()));
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegions)
   {
      super.setPlanarRegions(planarRegions);
      if (!hasPlanarRegions())
         return;

      planarRegionPolytopes.clear();

      for (PlanarRegion planarRegion : planarRegions.getPlanarRegionsAsList())
      {
         ConvexPolytope planarRegionPolytope = new ConvexPolytope();
         List<? extends Point2DReadOnly> pointsInPlanarRegion = planarRegion.getConvexHull().getVertexBufferView();
         planarRegion.getTransformToWorld(tempTransform);
         for (Point2DReadOnly point : pointsInPlanarRegion)
         {
            tempFramePoint.setToZero();
            tempFramePoint.set(point.getX(), point.getY(), 0.0);
            tempFramePoint.applyTransform(tempTransform);
            planarRegionPolytope.addVertex(tempFramePoint.getX(), tempFramePoint.getY(), tempFramePoint.getZ());
         }

         planarRegionPolytopes.put(planarRegion, planarRegionPolytope);
      }
   }

   @Override
   public boolean isNodeValid(FootstepNode node, FootstepNode previousNode)
   {
      if (!hasPlanarRegions() || previousNode == null || !parameters.checkForBodyBoxCollisions())
      {
         return true;
      }

      if (!isNodeValidInternal(node, previousNode, 1.0))
      {
         rejectNode(node, previousNode, BipedalFootstepPlannerNodeRejectionReason.OBSTACLE_HITTING_BODY);
         return false;
      }

      return true;
   }

   @Override
   public void addStartNode(FootstepNode startNode, RigidBodyTransform startNodeTransform)
   {

   }

   public boolean isNodeValidInternal(FootstepNode node, FootstepNode previousNode, double scaleFactor)
   {
      setBoundingBoxFrame(node, previousNode);
      updateBodyCollisionDimensions(node, previousNode, scaleFactor);

      bodyBoxDimensions.set(parameters.getBodyBoxBaseX(), parameters.getBodyBoxBaseY(), parameters.getBodyBoxBaseZ() + 0.5 * parameters.getBodyBoxHeight());
      bodyCollisionFrame.updateTranslation(bodyBoxDimensions);
      bodyCollisionPolytope.getVertices().clear();
      for (Point3D vertex : bodyCollisionBox.getVertices())
      {
         tempFramePoint.setIncludingFrame(bodyCollisionFrame, vertex);
         tempFramePoint.changeFrame(worldFrame);
         bodyCollisionPolytope.addVertex(tempFramePoint.getX(), tempFramePoint.getY(), tempFramePoint.getZ());
      }

      tempPoint2d1.set(node.getX(), node.getY());
      tempPoint2d2.set(previousNode.getX(), previousNode.getY());
      tempPoint2d1.interpolate(tempPoint2d1, tempPoint2d2, 0.5);

      List<PlanarRegion> nearbyRegions = PlanarRegionTools
              .filterPlanarRegionsWithBoundingCircle(tempPoint2d1, planarRegionsFilterRadius, planarRegionsList.getPlanarRegionsAsList());
      nearbyPlanarRegions.put(node.hashCode(), nearbyRegions);

      for (int i = 0; i < planarRegionsList.getNumberOfPlanarRegions(); i++)
      {
         ConvexPolytope planarRegionPolytope = planarRegionPolytopes.get(planarRegionsList.getPlanarRegion(i));
         if (collisionDetector.arePolytopesColliding(bodyCollisionPolytope, planarRegionPolytope, tempPoint1, tempPoint2))
         {
            return false;
         }
      }

      return true;
   }

   private final FramePose3D midPose = new FramePose3D();

   private void setBoundingBoxFrame(FootstepNode node, FootstepNode previousNode)
   {
      FootstepNodeTools.getSnappedNodeTransform(node, snapper.getSnapData(node).getSnapTransform(), tempTransform);
      tempTransform.getTranslation(tempPoint1);

      FootstepNodeTools.getSnappedNodeTransform(previousNode, snapper.getSnapData(previousNode).getSnapTransform(), tempTransform);
      tempTransform.getTranslation(tempPoint2);

      tempPoint1.interpolate(tempPoint2, 0.5);
      midPose.setPosition(tempPoint1);

      double angleAverage = AngleTools.computeAngleAverage(node.getYaw(), previousNode.getYaw());
      midPose.setOrientationYawPitchRoll(angleAverage, 0.0, 0.0);

      midStanceFrame.setPoseAndUpdate(midPose);
      bodyCollisionFrame.update();
   }

   private void updateBodyCollisionDimensions(FootstepNode node, FootstepNode previousNode, double scaleFactor)
   {
      tempVector.setIncludingFrame(worldFrame, node.getX(), node.getY(), 0.0);
      tempVector.sub(previousNode.getX(), previousNode.getY(), 0.0);
      tempVector.changeFrame(midStanceFrame);

      double stepTranslationMidFootFrameX = parameters.getStepTranslationBoundingBoxScaleFactor() * Math.abs(tempVector.getX());
      bodyCollisionBox.setSize(parameters.getBodyBoxDepth() + stepTranslationMidFootFrameX, parameters.getBodyBoxWidth(), parameters.getBodyBoxHeight());

      if (!MathTools.epsilonEquals(1.0, scaleFactor, 1e-5))
         bodyCollisionBox.scale(scaleFactor);
   }
}
