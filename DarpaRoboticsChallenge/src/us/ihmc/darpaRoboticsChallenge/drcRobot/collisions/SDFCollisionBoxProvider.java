package us.ihmc.darpaRoboticsChallenge.drcRobot.collisions;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import us.ihmc.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.SdfLoader.JaxbSDFLoader;
import us.ihmc.SdfLoader.SDFConversionsHelper;
import us.ihmc.SdfLoader.SDFJointHolder;
import us.ihmc.SdfLoader.SDFLinkHolder;
import us.ihmc.SdfLoader.xmlDescription.Collision;
import us.ihmc.SdfLoader.xmlDescription.SDFGeometry;
import us.ihmc.ihmcPerception.depthData.CollisionBoxProvider;
import us.ihmc.ihmcPerception.depthData.CollisionDescription;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;

import com.jme3.scene.Mesh;
import com.jme3.scene.shape.Box;
import com.jme3.scene.shape.Cylinder;
import com.jme3.scene.shape.Sphere;

public class SDFCollisionBoxProvider implements CollisionBoxProvider
{
   private final HashMap<String, List<CollisionDescription>> collissionMeshes = new HashMap<>();

   protected final float extent = 0.03f;

   public SDFCollisionBoxProvider(JaxbSDFLoader loader, String modelName)
   {
      GeneralizedSDFRobotModel model = loader.getGeneralizedSDFRobotModel(modelName);
      SDFLinkHolder rootLink = model.getRootLinks().get(0);
      recursivelyAddLinks(rootLink.getName(), rootLink);
   }
   
   public void addMesh(String jointName, Mesh mesh, RigidBodyTransform pose)
   {
      List<CollisionDescription> meshes = collissionMeshes.get(jointName);
      if (meshes == null)
      {
         meshes = new ArrayList<CollisionDescription>();
         collissionMeshes.put(jointName, meshes);
      }
      
      meshes.add(new CollisionDescription(mesh, pose));
   }

   private void recursivelyAddLinks(String jointName, SDFLinkHolder holder)
   {
      ArrayList<CollisionDescription> meshes = new ArrayList<>();

      for (Collision collision : holder.getCollisions())
      {
         SDFGeometry collisionGeometry = collision.getGeometry();
         RigidBodyTransform visualPose = SDFConversionsHelper.poseToTransform(collision.getPose());

         Mesh mesh;
         if (collisionGeometry.getBox() != null)
         {
            String[] boxDimensions = collisionGeometry.getBox().getSize().split(" ");
            float bx = Float.parseFloat(boxDimensions[0]) / 2.0f + extent;
            float by = Float.parseFloat(boxDimensions[1]) / 2.0f + extent;
            float bz = Float.parseFloat(boxDimensions[2]) / 2.0f + extent;

            mesh = new Box(bx, by, bz);
         }
         else if (collisionGeometry.getCylinder() != null)
         {

            float length = Float.parseFloat(collisionGeometry.getCylinder().getLength()) + 2.0f * extent;
            float radius = Float.parseFloat(collisionGeometry.getCylinder().getRadius()) + 2.0f * extent;

            mesh = new Cylinder(18, 18, radius, length, true);
         }
         else if (collisionGeometry.getSphere() != null)
         {
            float radius = Float.parseFloat(collisionGeometry.getSphere().getRadius()) * 2.0f * extent;
            mesh = new Sphere(18, 18, radius);
         }
         else
         {
            System.err.println("Cannot create collision box for " + holder);
            continue;
         }
         mesh.updateBound();
         CollisionDescription description = new CollisionDescription(mesh, visualPose);
         meshes.add(description);

      }
      collissionMeshes.put(jointName, meshes);

      for (SDFJointHolder joint : holder.getChildren())
      {
         recursivelyAddLinks(joint.getName(), joint.getChildLinkHolder());
      }
   }

   @Override
   public List<CollisionDescription> getCollisionMesh(String jointName)
   {
      return collissionMeshes.get(jointName);
   }

}
