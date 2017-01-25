package us.ihmc.avatar.drcRobot.collisions;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import us.ihmc.modelFileLoaders.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.modelFileLoaders.SdfLoader.JaxbSDFLoader;
import us.ihmc.modelFileLoaders.SdfLoader.SDFConversionsHelper;
import us.ihmc.modelFileLoaders.SdfLoader.SDFJointHolder;
import us.ihmc.modelFileLoaders.SdfLoader.SDFLinkHolder;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.Collision;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFGeometry;
import us.ihmc.ihmcPerception.depthData.CollisionBoxProvider;
import us.ihmc.ihmcPerception.depthData.collisionShapes.CollisionBox;
import us.ihmc.ihmcPerception.depthData.collisionShapes.CollisionCylinder;
import us.ihmc.ihmcPerception.depthData.collisionShapes.CollisionShape;
import us.ihmc.ihmcPerception.depthData.collisionShapes.CollisionSphere;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.tools.io.printing.PrintTools;

public class SDFCollisionBoxProvider implements CollisionBoxProvider
{
   private final HashMap<String, List<CollisionShape>> collissionMeshes = new HashMap<>();

   protected final float extent = 0.04f;

   public SDFCollisionBoxProvider(JaxbSDFLoader loader, String modelName)
   {
      GeneralizedSDFRobotModel model = loader.getGeneralizedSDFRobotModel(modelName);
      SDFLinkHolder rootLink = model.getRootLinks().get(0);
      recursivelyAddLinks(rootLink.getName(), rootLink);
   }
   
   public void addCollisionShape(String jointName, CollisionShape mesh)
   {
      List<CollisionShape> meshes = collissionMeshes.get(jointName);
      if (meshes == null)
      {
         meshes = new ArrayList<CollisionShape>();
         collissionMeshes.put(jointName, meshes);
      }
      
      meshes.add(mesh);
   }

   private void recursivelyAddLinks(String jointName, SDFLinkHolder holder)
   {
      ArrayList<CollisionShape> meshes = new ArrayList<>();

      for (Collision collision : holder.getCollisions())
      {
         SDFGeometry collisionGeometry = collision.getGeometry();
         RigidBodyTransform visualPose = SDFConversionsHelper.poseToTransform(collision.getPose());

         CollisionShape mesh;
         if (collisionGeometry.getBox() != null)
         {
            String[] boxDimensions = collisionGeometry.getBox().getSize().split(" ");
            float bx = Float.parseFloat(boxDimensions[0]) / 2.0f + extent;
            float by = Float.parseFloat(boxDimensions[1]) / 2.0f + extent;
            float bz = Float.parseFloat(boxDimensions[2]) / 2.0f + extent;

            mesh = new CollisionBox(visualPose, bx, by, bz);
         }
         else if (collisionGeometry.getCylinder() != null)
         {

            float length = Float.parseFloat(collisionGeometry.getCylinder().getLength()) + 2.0f * extent;
            float radius = Float.parseFloat(collisionGeometry.getCylinder().getRadius()) + 2.0f * extent;

            mesh = new CollisionCylinder(visualPose, radius, length);
         }
         else if (collisionGeometry.getSphere() != null)
         {
            float radius = Float.parseFloat(collisionGeometry.getSphere().getRadius()) * 2.0f * extent;
            mesh = new CollisionSphere(visualPose, radius);
         }
         else
         {
            PrintTools.error("Cannot create collision box for " + holder);
            continue;
         }
         meshes.add(mesh);

      }
      collissionMeshes.put(jointName, meshes);

      for (SDFJointHolder joint : holder.getChildren())
      {
         recursivelyAddLinks(joint.getName(), joint.getChildLinkHolder());
      }
   }

   @Override
   public List<CollisionShape> getCollisionMesh(String jointName)
   {
      return collissionMeshes.get(jointName);
   }

}
