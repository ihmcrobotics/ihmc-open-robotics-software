package us.ihmc.jMonkeyEngineToolkit.jme;

import com.jme3.collision.Collidable;
import com.jme3.collision.CollisionResult;
import com.jme3.collision.CollisionResults;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.jMonkeyEngineToolkit.jme.util.JMEDataTypeUtils;
import us.ihmc.jMonkeyEngineToolkit.jme.util.JMEGeometryUtils;
import us.ihmc.robotics.geometry.Ray3d;

public class JMERayCollisionAdapter
{
   private static final boolean DEBUG = false;
   private Collidable collidable;
   private CollisionResults results = new CollisionResults();
   private Node rootNode;

   public JMERayCollisionAdapter(Node rootNode)
   {
      this.rootNode = rootNode;

      if (DEBUG)
      {
         System.out.println("Adding root node with " + rootNode.getChildren().size() + " children.");
         
         for (Spatial child : rootNode.getChildren())
         {
            System.out.println("Child: " + child.getName());
         }
      }
   }

   public double getPickDistance()
   {
      return getPickDistance(rootNode);
   }

   public Node cloneRootNode()
   {
      return rootNode.clone(false);
   }

   public double getPickDistance(Node rootNode)
   {
      return getPickDistance(rootNode, null, null);
   }

   public double getPickDistance(Node rootNode, Vector3D normalToPack, Point3D closestPoint)
   {
      results.clear();
      rootNode.collideWith(collidable, results);

      for (CollisionResult collisionResult : results)
      {
         Geometry geometry = collisionResult.getGeometry();
         String userDataRayCastOpacity = JMERayCollisionAdapter.searchForUserData(geometry.getParent(), JMERayCastOpacity.USER_DATA_FIELD);

         if (DEBUG)
            System.out.println("JMERayCollisionAdapter: userDataRayCastOpacity = " + userDataRayCastOpacity);

         if (JMERayCastOpacity.OPAQUE.toString().equals(userDataRayCastOpacity))
         {
            if (normalToPack != null)
            {
               packInSCSCoordinates(collisionResult.getContactNormal(), normalToPack);
            }

            if (closestPoint != null)
            {
               packInSCSCoordinates(collisionResult.getContactPoint(), closestPoint);
            }

            return collisionResult.getDistance();
         }
      }

      return Double.NaN;
   }

   private static String searchForUserData(Node startingNode, String userDataToSearch)
   {
      Node node = startingNode;
      do
      {
         String userData = node.getUserData(userDataToSearch);
         if (userData != null)
            return userData;
         node = node.getParent();
      }
      while (node != null);

      return null;
   }

   private void packInSCSCoordinates(Vector3f vector3f, Tuple3DBasics tuple3d)
   {
      JMEGeometryUtils.transformFromJMECoordinatesToZup(vector3f);
      JMEDataTypeUtils.packJMEVector3fInVecMathTuple3d(vector3f, tuple3d);
   }

   public void setPickingGeometry(Ray3d ray3d)
   {
      Point3D rayOrigin = ray3d.getPoint();
      Vector3D rayDirection = ray3d.getVector();

      Vector3f rayOrigin3f = new Vector3f();
      Vector3f rayDirection3f = new Vector3f();

      JMEDataTypeUtils.packVecMathTuple3dInJMEVector3f(rayOrigin, rayOrigin3f);
      JMEDataTypeUtils.packVecMathTuple3dInJMEVector3f(rayDirection, rayDirection3f);

      JMEGeometryUtils.transformFromZupToJMECoordinates(rayOrigin3f);
      JMEGeometryUtils.transformFromZupToJMECoordinates(rayDirection3f);

      com.jme3.math.Ray jmeRay = new com.jme3.math.Ray(rayOrigin3f, rayDirection3f);

      collidable = jmeRay;
   }
}
