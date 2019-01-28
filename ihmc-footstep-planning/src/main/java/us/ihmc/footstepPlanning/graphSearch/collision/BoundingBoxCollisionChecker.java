package us.ihmc.footstepPlanning.graphSearch.collision;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.shape.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.HashMap;

public class BoundingBoxCollisionChecker
{
   private final FootstepNodeSnapperReadOnly snapDataHolder;
   private final FootstepPlannerParameters parameters;
   private PlanarRegionsList planarRegionsList;
   private final HashMap<LatticeNode, BodyCollisionData> collisionDataHolder = new HashMap<>();

   private double bodyPoseX = Double.NaN;
   private double bodyPoseY = Double.NaN;
   private double bodyPoseYaw = Double.NaN;

   private final Box3D bodyBox;
   private final BoundingBox3D boundingBox = new BoundingBox3D();

   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final Point3D tempPoint = new Point3D();

   public BoundingBoxCollisionChecker(FootstepPlannerParameters parameters, FootstepNodeSnapperReadOnly snapDataHolder)
   {
      this.parameters = parameters;
      this.snapDataHolder = snapDataHolder;
      this.bodyBox = new Box3D(0.5 * parameters.getMaximumBodyBoxDepthToPenalize(), 0.5 * parameters.getMaximumBodyBoxWidthToPenalize(),
                               0.5 * parameters.getBodyBoxHeight());
   }

   public void setPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      collisionDataHolder.clear();
      this.planarRegionsList = planarRegionsList;
   }

   public void reset()
   {
      collisionDataHolder.clear();
   }

   public void setBoxPose(double bodyPoseX, double bodyPoseY, double bodyPoseYaw)
   {
      this.bodyPoseX = bodyPoseX;
      this.bodyPoseY = bodyPoseY;
      this.bodyPoseYaw = bodyPoseYaw;
   }

   public BodyCollisionData checkForCollision()
   {
      checkInputs();

      LatticeNode node = new LatticeNode(bodyPoseX, bodyPoseY, bodyPoseYaw);
      if(collisionDataHolder.containsKey(node))
         return collisionDataHolder.get(node);

      setBoundingBox();
      BodyCollisionData collisionData = new BodyCollisionData();

      for(PlanarRegion planarRegion : planarRegionsList.getPlanarRegionsAsList())
      {
         if(planarRegion.getBoundingBox3dInWorld().intersectsExclusive(boundingBox))
         {
            collisionData.setCollisionDetected(true);

            ConvexPolygon2D convexHull = planarRegion.getConvexHull();
            planarRegion.getTransformToWorld(tempTransform);

            for (int i = 0; i < convexHull.getNumberOfVertices(); i++)
            {
               tempPoint.set(convexHull.getVertex(i));
               tempPoint.setZ(0.0);
               tempTransform.transform(tempPoint);

               if(boundingBox.isInsideInclusive(tempPoint))
               {
                  tempTransform.setTranslation(bodyBox.getPositionX(), bodyBox.getPositionY(), bodyBox.getPositionZ());
                  tempTransform.setRotation(bodyBox.getOrientation());
                  tempTransform.invert();
                  tempTransform.transform(tempPoint);

                  double xBodyFrame = tempPoint.getX();
                  double yBodyFrame = tempPoint.getY();

                  if(closerToBody(xBodyFrame, yBodyFrame, collisionData))
                     collisionData.setCollisionPointInBodyFrame(xBodyFrame, yBodyFrame);
               }
            }
         }
      }

      collisionDataHolder.put(node, collisionData);
      return collisionData;
   }

   private boolean closerToBody(double x, double y, BodyCollisionData collisionData)
   {
      Point2DReadOnly collisionPointInBodyFrame = collisionData.getCollisionPointInBodyFrame();
      if(collisionPointInBodyFrame.containsNaN())
         return true;

      double minX = parameters.getBodyBoxDepth();
      double maxX = parameters.getMaximumBodyBoxDepthToPenalize();
      double minY = parameters.getBodyBoxWidth();
      double maxY = parameters.getMaximumBodyBoxWidthToPenalize();

      double oldProximityRatio = Math.min((collisionPointInBodyFrame.getX() - minX) / (maxX - minX), (collisionPointInBodyFrame.getY() - minY) / (maxY - minY));
      double newProximityRatio = Math.min((x - minX) / (maxX - minX), (y - minY) / (maxY - minY));
      return newProximityRatio < oldProximityRatio;
   }

   private void setBoundingBox()
   {
      FootstepNodeSnapData snapData = snapDataHolder.getSnapData(new FootstepNode(bodyPoseX, bodyPoseY, bodyPoseYaw, RobotSide.LEFT));
      if(snapData == null)
         snapData = snapDataHolder.getSnapData(new FootstepNode(bodyPoseX, bodyPoseY, bodyPoseYaw, RobotSide.RIGHT));

      double groundHeight = 0.0;
      if(snapData != null)
         groundHeight = snapData.getSnapTransform().getTranslationZ();

      bodyBox.setPosition(bodyPoseX, bodyPoseY, groundHeight + parameters.getBodyBoxBaseZ() + 0.5 * parameters.getBodyBoxHeight());
      bodyBox.setOrientationYawPitchRoll(bodyPoseYaw, 0.0, 0.0);
      bodyBox.getBoundingBox3D(boundingBox);
   }

   private void checkInputs()
   {
      if(Double.isNaN(bodyPoseX) || Double.isNaN(bodyPoseY) || Double.isNaN(bodyPoseYaw))
         throw new RuntimeException("Bounding box position has not been set");
   }
}
