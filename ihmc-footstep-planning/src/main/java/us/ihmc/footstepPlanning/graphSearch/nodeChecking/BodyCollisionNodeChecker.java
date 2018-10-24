package us.ihmc.footstepPlanning.graphSearch.nodeChecking;

import us.ihmc.euclid.geometry.Box3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerListener;
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

import java.util.HashMap;
import java.util.List;

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

   private BipedalFootstepPlannerListener listener;


   public BodyCollisionNodeChecker(FootstepPlannerParameters parameters, FootstepNodeSnapper snapper)
   {
      this.parameters = parameters;
      this.snapper = snapper;

      bodyCollisionBox = new Box3D();
      bodyCollisionBox.setSize(parameters.getBodyBoxDepth(), parameters.getBodyBoxWidth(), parameters.getBodyBoxHeight());

      bodyCollisionFrame.updateTranslation(new Vector3D(parameters.getBodyBoxBaseX(), parameters.getBodyBoxBaseY(), parameters.getBodyBoxBaseZ() + 0.5 * parameters.getBodyBoxHeight()));
   }

   public void addPlannerListener(BipedalFootstepPlannerListener listener)
   {
      this.listener = listener;
   }


   private final FramePoint3D tempPoint = new FramePoint3D();
   private final RigidBodyTransform tempTransform= new RigidBodyTransform();

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegions)
   {
      super.setPlanarRegions(planarRegions);
      if(!hasPlanarRegions())
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
   public boolean isNodeValid(FootstepNode node, FootstepNode previousNode)
   {
      if (!hasPlanarRegions() || previousNode == null || !parameters.checkForBodyBoxCollisions())
      {
         return true;
      }

      if (!findMidStanceFrame(node, previousNode))
         return true;

      tempPoint.setToZero(midStanceFrame);
      tempPoint.changeFrame(worldFrame);
      double roundedX = FootstepNode.round(tempPoint.getX());
      double roundedY = FootstepNode.round(tempPoint.getY());
      List<PlanarRegion> planarRegionList = snapper.getOrCreateBodyCollisionRegions(roundedX, roundedY, tempPoint.getZ());

      if (planarRegionList.size() == 0)
         return true;

      bodyCollisionBox.setSize(parameters.getBodyBoxDepth(), parameters.getBodyBoxWidth(), parameters.getBodyBoxHeight());
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
            notifyPlannerListenerThatNodeIsRejected(node, BipedalFootstepPlannerNodeRejectionReason.OBSTACLE_HITTING_BODY);
            return false;
         }
      }

      return true;
   }

   @Override
   public void addStartNode(FootstepNode startNode, RigidBodyTransform startNodeTransform)
   {

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

   private void notifyPlannerListenerThatNodeIsRejected(FootstepNode node, BipedalFootstepPlannerNodeRejectionReason rejectionReason)
   {
      if(listener != null)
         listener.nodeUnderConsiderationWasRejected(node, rejectionReason);
   }
}
