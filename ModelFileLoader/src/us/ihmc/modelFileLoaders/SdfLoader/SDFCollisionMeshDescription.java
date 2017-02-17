package us.ihmc.modelFileLoaders.SdfLoader;

import java.util.List;

import org.apache.commons.lang3.builder.ToStringBuilder;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.modelFileLoaders.ModelFileLoaderConversionsHelper;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.AbstractSDFMesh;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFGeometry;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFGeometry.Mesh;
import us.ihmc.robotics.robotDescription.CollisionMeshDescription;


public class SDFCollisionMeshDescription extends CollisionMeshDescription
{
   public SDFCollisionMeshDescription(List<? extends AbstractSDFMesh> sdfVisuals)
   {
      this(sdfVisuals, new RigidBodyTransform());
   }

   public SDFCollisionMeshDescription(List<? extends AbstractSDFMesh> sdfVisuals, RigidBodyTransform graphicsTransform)
   {
      RotationMatrix rotation = new RotationMatrix();
      Vector3D offset = new Vector3D();
      graphicsTransform.get(rotation, offset);

      if(sdfVisuals != null)
      {
         for(AbstractSDFMesh sdfVisual : sdfVisuals)
         {
            identity();
            translate(offset);
            rotate(rotation);

            RigidBodyTransform visualPose = ModelFileLoaderConversionsHelper.poseToTransform(sdfVisual.getPose());
            Vector3D modelOffset = new Vector3D();
            RotationMatrix modelRotation = new RotationMatrix();
            visualPose.get(modelRotation, modelOffset);

            translate(modelOffset);
            rotate(modelRotation);

            SDFGeometry geometry = sdfVisual.getGeometry();
            Mesh mesh = geometry.getMesh();
            if(mesh != null)
            {
               System.err.println("Meshes not implemented yet for CollisionMeshes!! Skipping " + mesh.getUri());
            }
            else if(geometry.getCylinder() != null)
            {
               double length = Double.parseDouble(geometry.getCylinder().getLength());
               double radius = Double.parseDouble(geometry.getCylinder().getRadius());
//               translate(0.0, 0.0, -length/2.0);

               addCylinderReferencedAtCenter(length, radius);
            }
            else if(geometry.getBox() != null)
            {
               String[] boxDimensions = geometry.getBox().getSize().split(" ");
               double bx = Double.parseDouble(boxDimensions[0]);
               double by = Double.parseDouble(boxDimensions[1]);
               double bz = Double.parseDouble(boxDimensions[2]);
//               translate(0.0, 0.0, -bz/2.0);
               addCubeReferencedAtCenter(bx, by, bz);
            }
            else if(geometry.getSphere() != null)
            {
               double radius = Double.parseDouble(geometry.getSphere().getRadius());
               addSphere(radius);
            }
            else if(geometry.getPlane() != null)
            {
               throw new RuntimeException("Planes not implemented yet for CollisionMeshes!!");

//               Vector3d normal = ModelFileLoaderConversionsHelper.stringToNormalizedVector3d(geometry.getPlane().getNormal());
//               Vector2d size = ModelFileLoaderConversionsHelper.stringToVector2d(geometry.getPlane().getSize());
//
//               AxisAngle4d planeRotation = GeometryTools.getRotationBasedOnNormal(normal);
//               rotate(planeRotation);
//               addCube(size.getX(), size.getY(), 0.005, getDefaultAppearanceIfNull(appearance));
            }
            else if(geometry.getHeightMap() != null)
            {
               throw new RuntimeException("Height Map not implemented for CollisionMeshes!!");
            }
            else
            {
               System.err.println("Visual for " + sdfVisual.getName() + " not implemented yet");
               System.err.println("Defined visual" + ToStringBuilder.reflectionToString(geometry));

            }
         }
      }
   }
}
