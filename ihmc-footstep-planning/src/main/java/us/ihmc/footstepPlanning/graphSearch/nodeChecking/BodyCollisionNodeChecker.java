package us.ihmc.footstepPlanning.graphSearch.nodeChecking;

import java.util.HashMap;
import java.util.List;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
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
   private final PoseReferenceFrame midStanceFrame = new PoseReferenceFrame("midStanceFrame", worldFrame);
   private final TranslationReferenceFrame bodyCollisionFrame = new TranslationReferenceFrame("bodyCollisionFrame", midStanceFrame);
   private final Box3D bodyCollisionBox;
   private final ConvexPolytope bodyCollisionPolytope = new ConvexPolytope();

   private final GilbertJohnsonKeerthiCollisionDetector collisionDetector = new GilbertJohnsonKeerthiCollisionDetector();

   private final HashMap<PlanarRegion, ConvexPolytope> planarRegionPolytopes = new HashMap<>();

   private final FootstepPlannerParameters parameters;
   private final FootstepNodeSnapper snapper;
   private final Vector3D bodyBoxDimensions = new Vector3D();

   private final FramePoint3D tempPoint = new FramePoint3D();
   private final FrameVector3D tempVector = new FrameVector3D();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();

   public BodyCollisionNodeChecker(FootstepPlannerParameters parameters, FootstepNodeSnapper snapper)
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
            tempPoint.setToZero();
            tempPoint.set(point.getX(), point.getY(), 0.0);
            tempPoint.applyTransform(tempTransform);
            planarRegionPolytope.addVertex(tempPoint.getX(), tempPoint.getY(), tempPoint.getZ());
         }

         planarRegionPolytopes.put(planarRegion, planarRegionPolytope);
      }
   }

   private final Point3D pointToThrowAway1 = new Point3D();
   private final Point3D pointToThrowAway2 = new Point3D();

   @Override
   public boolean isNodeValidInternal(FootstepNode node, FootstepNode previousNode)
   {
      if (previousNode == null || !parameters.checkForBodyBoxCollisions())
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
      if (!findMidStanceFrame(node, previousNode))
         return true;

      tempPoint.setToZero(midStanceFrame);
      tempPoint.changeFrame(worldFrame);
      double roundedX = FootstepNode.round(tempPoint.getX());
      double roundedY = FootstepNode.round(tempPoint.getY());
      List<PlanarRegion> planarRegionList = snapper.getOrCreateBodyCollisionRegions(roundedX, roundedY, tempPoint.getZ());

      if (planarRegionList.size() == 0)
         return true;

      updateBodyCollisionBox(node, previousNode, scaleFactor);

      bodyBoxDimensions.set(parameters.getBodyBoxBaseX(), parameters.getBodyBoxBaseY(), parameters.getBodyBoxBaseZ() + 0.5 * parameters.getBodyBoxHeight());
      bodyCollisionFrame.updateTranslation(bodyBoxDimensions);
      bodyCollisionPolytope.getVertices().clear();
      for (Point3D vertex : bodyCollisionBox.getVertices())
      {
         tempPoint.setIncludingFrame(bodyCollisionFrame, vertex);
         tempPoint.changeFrame(worldFrame);
         bodyCollisionPolytope.addVertex(tempPoint.getX(), tempPoint.getY(), tempPoint.getZ());
      }

      for (PlanarRegion planarRegion : planarRegionList)
      {
         ConvexPolytope planarRegionPolytope = planarRegionPolytopes.get(planarRegion);
         if (collisionDetector.arePolytopesColliding(bodyCollisionPolytope, planarRegionPolytope, pointToThrowAway1, pointToThrowAway2))
         {
            return false;
         }
      }

      return true;
   }

   private void updateBodyCollisionBox(FootstepNode node, FootstepNode previousNode, double scaleFactor)
   {
      tempVector.setIncludingFrame(worldFrame, node.getX(), node.getY(), 0.0);
      tempVector.sub(previousNode.getX(), previousNode.getY(), 0.0);
      tempVector.changeFrame(midStanceFrame);

      double stepTranslationMidFootFrameX = parameters.getStepTranslationBoundingBoxScaleFactor() * Math.abs(tempVector.getX());
      bodyCollisionBox.setSize(parameters.getBodyBoxDepth() + stepTranslationMidFootFrameX, parameters.getBodyBoxWidth(), parameters.getBodyBoxHeight());

      if (!MathTools.epsilonEquals(1.0, scaleFactor, 1e-5))
         bodyCollisionBox.scale(scaleFactor);
   }

   private final FramePose3D midPose = new FramePose3D();

   private boolean findMidStanceFrame(FootstepNode node, FootstepNode previousNode)
   {
      Point3D midPoint = getMidPoint(node, previousNode);
      if (midPoint == null)
         return false;

      double angleAverage = AngleTools.computeAngleAverage(node.getYaw(), previousNode.getYaw());

      midPose.setPosition(midPoint);
      midPose.setOrientationYawPitchRoll(angleAverage, 0.0, 0.0);

      midStanceFrame.setPoseAndUpdate(midPose);
      bodyCollisionFrame.update();

      return true;
   }

   private Point3D getMidPoint(FootstepNode node, FootstepNode previousNode)
   {
      List<PlanarRegion> nodePlanes = snapper.getOrCreateSteppableRegions(node.getRoundedX(), node.getRoundedY());
      if (nodePlanes.isEmpty())
         return null;

      tempPoint.set(node.getRoundedX(), node.getRoundedY(), 0.0);
      Point3DReadOnly projectedPoint = PlanarRegionTools.projectPointToPlanesVertically(tempPoint, nodePlanes);

      if (projectedPoint == null)
         return null;

      List<PlanarRegion> previousNodePlanes = snapper.getOrCreateSteppableRegions(previousNode.getRoundedX(), previousNode.getRoundedY());
      if (previousNodePlanes.isEmpty())
         return null;

      tempPoint.set(previousNode.getRoundedX(), previousNode.getRoundedY(), 0.0);

      Point3DReadOnly previousProjectedPoint = PlanarRegionTools.projectPointToPlanesVertically(tempPoint, previousNodePlanes);

      if (previousProjectedPoint == null)
         return null;

      Point3D midPoint = new Point3D();
      midPoint.interpolate(projectedPoint, previousProjectedPoint, 0.5);

      return midPoint;
   }
}
