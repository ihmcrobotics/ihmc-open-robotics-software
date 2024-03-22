package us.ihmc.perception.sceneGraph.rigidBody;

import gnu.trove.map.TLongObjectMap;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;

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
   }

   public void updatePlanarRegions(PlanarRegionsList planarRegionsList)
   {
      if (getName().toLowerCase().contains("door"))
      {
         if (!planarRegionsList.isEmpty())
         {
            Point3D pointInWorld = new Point3D(getNodeFrame().getTransformToWorldFrame().getTranslation());

            float epsilon = 0.15f;

            PlanarRegion doorPlanarRegion = null;

            for (PlanarRegion closePlanarRegion : planarRegionsList.getPlanarRegionsAsList())
            {
               if (closePlanarRegion.distance(pointInWorld) > epsilon)
               {
                  continue;
               }

               if (doorPlanarRegion == null)
               {
                  doorPlanarRegion = closePlanarRegion;
                  continue;
               }

               if (closePlanarRegion.distance(pointInWorld) < doorPlanarRegion.distance(pointInWorld))
               {
                  doorPlanarRegion = closePlanarRegion;
               }
            }

            if (doorPlanarRegion != null)
            {
               System.out.println(doorPlanarRegion.getRegionId());

               RobotSide sideOfDoor;
               Quaternion orientation = new Quaternion();
               EuclidGeometryTools.orientation3DFromFirstToSecondVector3D(getNodeToParentFrameTransform().getTranslation(),
                                                                          doorPlanarRegion.getNormal(),
                                                                          orientation);

               Point3D centerOfDoor = new Point3D();
               doorPlanarRegion.getBoundingBox3dInWorld().getCenterPoint(centerOfDoor);

               Vector3D centerOfDoorToDoorHandle = new Vector3D();

//               double normalAngle = doorPlanarRegion.getNormal().angle(getNodeFrame().getTransformToWorldFrame().getTranslation());

               sideOfDoor = MathTools.sign(orientation.getYaw()) == -1 ? RobotSide.LEFT : RobotSide.RIGHT;

               System.out.println(sideOfDoor);

               double doorYawInWorld = doorPlanarRegion.getTransformToWorld().getRotation().getYaw();
               getNodeToParentFrameTransform().getRotation().setYawPitchRoll(doorYawInWorld + Math.PI / 2, Math.PI, 0);
               getNodeFrame().update();
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
