package us.ihmc.footstepPlanning.graphSearch.nodeChecking;

import us.ihmc.commons.PrintTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.Box3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.geometry.polytope.ConvexPolytope;
import us.ihmc.geometry.polytope.GilbertJohnsonKeerthiCollisionDetector;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.TranslationReferenceFrame;

import java.util.List;

public class BodyCollisionNodeChecker implements FootstepNodeChecker
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private PlanarRegionsList planarRegionsList;
   private final PoseReferenceFrame midStanceFrame = new PoseReferenceFrame("midStanceFrame", worldFrame);
   private final TranslationReferenceFrame bodyCollisionFrame = new TranslationReferenceFrame("bodyCollisionFrame", midStanceFrame);
   private final Box3D bodyCollisionBox;
   private final ConvexPolytope bodyCollisionPolytope = new ConvexPolytope();

   private final GilbertJohnsonKeerthiCollisionDetector collisionDetector = new GilbertJohnsonKeerthiCollisionDetector();

   private final RecyclingArrayList<ConvexPolytope> planarRegionPolytopes = new RecyclingArrayList<>(ConvexPolytope.class);

   public BodyCollisionNodeChecker(FootstepPlannerParameters parameters)
   {
      bodyCollisionBox = new Box3D();
      bodyCollisionBox.setSize(parameters.getBodyBoxDepth(), parameters.getBodyBoxWidth(), parameters.getBodyBoxHeight());

      bodyCollisionFrame.updateTranslation(new Vector3D(0.0, 0.0, parameters.getBodyBoxCenterHeight()));
   }

   private final FramePoint3D tempPoint = new FramePoint3D();
   private final RigidBodyTransform tempTransform= new RigidBodyTransform();

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegions)
   {
      this.planarRegionsList = planarRegions;
      planarRegionPolytopes.clear();

      for (PlanarRegion planarRegion : planarRegions.getPlanarRegionsAsList())
      {
         ConvexPolytope planarRegionPolytope = planarRegionPolytopes.add();
         List<? extends Point2DReadOnly> pointsInPlanarRegion = planarRegion.getConvexHull().getVertexBufferView();
         planarRegion.getTransformToWorld(tempTransform);
         for (Point2DReadOnly point : pointsInPlanarRegion)
         {
            tempPoint.setToZero();
            tempPoint.set(point.getX(), point.getY(), 0.0);
            tempPoint.applyTransform(tempTransform);
            planarRegionPolytope.addVertex(tempPoint.getX(), tempPoint.getY(), tempPoint.getZ());
         }
      }
   }

   private final Point3D pointToThrowAway1 = new Point3D();
   private final Point3D pointToThrowAway2 = new Point3D();

   @Override
   public boolean isNodeValid(FootstepNode node, FootstepNode previousNode)
   {
      if (previousNode == null)
      {
         PrintTools.info("previousNode is null");
         return true;
      }

      if (!findMidStanceFrame(node, previousNode))
         return false;

      bodyCollisionPolytope.getVertices().clear();
      for (Point3D vertex : bodyCollisionBox.getVertices())
      {
         tempPoint.setIncludingFrame(bodyCollisionFrame, vertex);
         tempPoint.changeFrame(worldFrame);
         bodyCollisionPolytope.addVertex(tempPoint.getX(), tempPoint.getY(), tempPoint.getZ());
      }

      for (ConvexPolytope planarRegionPolytope : planarRegionPolytopes)
      {
         if (collisionDetector.arePolytopesColliding(bodyCollisionPolytope, planarRegionPolytope, pointToThrowAway1, pointToThrowAway2))
            return false;
      }

      return true;
   }

   @Override
   public void addStartNode(FootstepNode startNode, RigidBodyTransform startNodeTransform)
   {

   }

   private final FramePoint3D point = new FramePoint3D();
   private final FramePoint3D previousPoint = new FramePoint3D();
   private final FramePoint3D midPoint = new FramePoint3D();
   private final FramePose3D midPose = new FramePose3D();

   private boolean findMidStanceFrame(FootstepNode node, FootstepNode previousNode)
   {

      List<PlanarRegion> nodePlanes = planarRegionsList.findPlanarRegionsContainingPointByProjectionOntoXYPlane(node.getX(), node.getY());
      if (nodePlanes.isEmpty())
      {
         return false;
      }

      List<PlanarRegion> previousNodePlanes = planarRegionsList.findPlanarRegionsContainingPointByProjectionOntoXYPlane(previousNode.getX(), previousNode.getY());

      double nodeZ = -Double.MAX_VALUE;
      double previousNodeZ = -Double.MAX_VALUE;
      for (PlanarRegion nodePlane : nodePlanes)
      {
         double height = nodePlane.getPlaneZGivenXY(node.getX(), node.getY());
         nodeZ = Math.max(nodeZ, height);
      }
      for (PlanarRegion previousNodePlane : previousNodePlanes)
      {
         double height = previousNodePlane.getPlaneZGivenXY(previousNode.getX(), previousNode.getY());
         previousNodeZ = Math.max(height, previousNodeZ);
      }

      point.set(node.getX(), node.getY(), nodeZ);
      previousPoint.set(previousNode.getX(), previousNode.getY(), previousNodeZ);
      midPoint.interpolate(point, previousPoint, 0.5);

      double angleAverage = AngleTools.computeAngleAverage(node.getYaw(), previousNode.getYaw());

      midPose.setPosition(midPoint);
      midPose.setOrientationYawPitchRoll(angleAverage, 0.0, 0.0);

      midStanceFrame.setPoseAndUpdate(midPose);
      bodyCollisionFrame.update();

      return true;
   }
}
