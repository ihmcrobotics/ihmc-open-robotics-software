package us.ihmc.modelFileLoaders.SdfLoader;

import java.util.List;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import org.apache.commons.lang3.builder.ToStringBuilder;

import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.AbstractSDFMesh;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFGeometry;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFGeometry.Mesh;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotDescription.CollisionMeshDescription;


public class SDFCollisionMeshDescription extends CollisionMeshDescription
{
   public SDFCollisionMeshDescription(List<? extends AbstractSDFMesh> sdfVisuals)
   {
      this(sdfVisuals, new RigidBodyTransform());
   }

   public SDFCollisionMeshDescription(List<? extends AbstractSDFMesh> sdfVisuals, RigidBodyTransform graphicsTransform)
   {
      Matrix3d rotation = new Matrix3d();
      Vector3d offset = new Vector3d();
      graphicsTransform.get(rotation, offset);

      if(sdfVisuals != null)
      {
         for(AbstractSDFMesh sdfVisual : sdfVisuals)
         {
            identity();
            translate(offset);
            rotate(rotation);

            RigidBodyTransform visualPose = SDFConversionsHelper.poseToTransform(sdfVisual.getPose());
            Vector3d modelOffset = new Vector3d();
            Matrix3d modelRotation = new Matrix3d();
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

               System.out.println("Adding Cylinder with height = " + length + " and radius + " + radius);
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

//               Vector3d normal = SDFConversionsHelper.stringToNormalizedVector3d(geometry.getPlane().getNormal());
//               Vector2d size = SDFConversionsHelper.stringToVector2d(geometry.getPlane().getSize());
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
