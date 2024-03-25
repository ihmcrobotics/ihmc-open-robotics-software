package us.ihmc.perception.sceneGraph.rigidBody;

import gnu.trove.map.TLongObjectMap;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

/**
 * A scene object that is a rigid body whose shape and appearance is
 * known beforehand.
 * <p>
 * Rigid bodies as in a door panel, chair, can of soup, etc.
 * <p>
 * This class also provides support for remembering the parent frame
 * and initial transform to parent, allowing an operator to manually
 * adjust it and also reset it.
 * <p>
 * TODO:
 *   - Add collision information
 */
public class PredefinedRigidBodySceneNode extends RigidBodySceneNode
{
   private final String visualModelFilePath;
   private final RigidBodyTransform visualModelToNodeFrameTransform;
   private final ReferenceFrame planarRegionCentroidZUp;
   private final RigidBodyTransform planarRegionCentroidZUpTransformToWorld = new RigidBodyTransform();

   public PredefinedRigidBodySceneNode(long id,
                                       String name,
                                       TLongObjectMap<SceneNode> sceneGraphIDToNodeMap,
                                       long initialParentNodeID,
                                       RigidBodyTransformReadOnly initialTransformToParent,
                                       String visualModelFilePath,
                                       RigidBodyTransform visualModelToNodeFrameTransform)
   {
      super(id, name, sceneGraphIDToNodeMap, initialParentNodeID, initialTransformToParent);
      this.visualModelFilePath = visualModelFilePath;
      this.visualModelToNodeFrameTransform = visualModelToNodeFrameTransform;

      planarRegionCentroidZUp = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(),
                                                                                                       planarRegionCentroidZUpTransformToWorld);
   }

   public void updatePlanarRegions(PlanarRegionsList planarRegionsList)
   {
      if (getName().toLowerCase().contains("door"))
      {
         if (!planarRegionsList.isEmpty())
         {
            Point3D doorLeverPointInWorld = new Point3D(getNodeFrame().getTransformToWorldFrame().getTranslation());

            float epsilon = 0.15f;

            PlanarRegion doorPlanarRegion = null;

            for (PlanarRegion closePlanarRegion : planarRegionsList.getPlanarRegionsAsList())
            {
               if (closePlanarRegion.distance(doorLeverPointInWorld) > epsilon)
               {
                  continue;
               }

               if (doorPlanarRegion == null)
               {
                  doorPlanarRegion = closePlanarRegion;
                  continue;
               }

               if (closePlanarRegion.distance(doorLeverPointInWorld) < doorPlanarRegion.distance(doorLeverPointInWorld))
               {
                  doorPlanarRegion = closePlanarRegion;
               }
            }

            if (doorPlanarRegion != null)
            {
               System.out.println("doorPlanarRegion ID " + doorPlanarRegion.getRegionId());

               planarRegionCentroidZUpTransformToWorld.getTranslation().set(doorPlanarRegion.getTransformToWorld().getTranslation());
               planarRegionCentroidZUpTransformToWorld.getRotation().setYawPitchRoll(doorPlanarRegion.getTransformToWorld().getRotation().getYaw(), 0.0, 0.0);

               planarRegionCentroidZUp.update();

               FramePoint3D handleFramePoint = new FramePoint3D();
               handleFramePoint.setToZero(getNodeFrame());
               handleFramePoint.changeFrame(planarRegionCentroidZUp);

               System.out.println(handleFramePoint);

               // Update yaw wrt door normal
               //               getNodeToParentFrameTransform().getRotation().setToYawOrientation((Math.PI / 2));
               //               getNodeFrame().update();

               //               Point3D centerOfDoor = new Point3D();
               //               doorPlanarRegion.getBoundingBox3dInWorld().getCenterPoint(centerOfDoor);
               //
               //               Vector3D normal = new Vector3D(Axis3D.Z);
               //               doorPlanarRegion.getTransformToWorld().getRotation().transform(normal);
               //
               //               System.out.println(normal);
               //
               //               Line2D doorLineNormal = new Line2D(centerOfDoor.getX(), centerOfDoor.getY(), normal.getX(), normal.getY());
               //               Point2D doorLeverPointInWorld2D = new Point2D(doorLeverPointInWorld);
               //
               //               RobotSide doorSide = doorLineNormal.isPointOnLeftSideOfLine(doorLeverPointInWorld2D) ? RobotSide.RIGHT : RobotSide.LEFT;
               //
               //               System.out.println(doorSide);
            }
         }
      }
   }

   public String getVisualModelFilePath()
   {
      return visualModelFilePath;
   }

   public RigidBodyTransform getVisualModelToNodeFrameTransform()
   {
      return visualModelToNodeFrameTransform;
   }
}
