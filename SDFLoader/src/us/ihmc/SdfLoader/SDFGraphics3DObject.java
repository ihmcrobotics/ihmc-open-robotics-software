package us.ihmc.SdfLoader;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.List;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.xmlDescription.SDFVisual;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.ModelFileType;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;


public class SDFGraphics3DObject extends Graphics3DObject
{
   private static final boolean SHOW_COORDINATE_SYSTEMS = false;

   public SDFGraphics3DObject(Matrix3d rotation, List<SDFVisual> sdfVisuals, String resourceDirectory)
   {
      rotate(rotation);
      
      for(SDFVisual sdfVisual : sdfVisuals)
      {
         Transform3D visualPose = SDFConversionsHelper.poseToTransform(sdfVisual.getPose());
         Vector3d modelOffset = new Vector3d();
         Matrix3d modelRotation = new Matrix3d();
         visualPose.get(modelRotation, modelOffset);
         
         if (SHOW_COORDINATE_SYSTEMS)
         {
            addCoordinateSystem(0.1);         
         }
         translate(modelOffset);
         rotate(modelRotation);       
         
         if(sdfVisual.getGeometry().getMesh() != null)
         {
            String uri = convertToFullPath(resourceDirectory, sdfVisual.getGeometry().getMesh().getUri());
            addMesh(uri, visualPose);
         }
         else if(sdfVisual.getGeometry().getCylinder() != null)
         {
            double length = Double.parseDouble(sdfVisual.getGeometry().getCylinder().getLength());
            double radius = Double.parseDouble(sdfVisual.getGeometry().getCylinder().getRadius()); 
            addCylinder(length, radius, YoAppearance.White());
         }
         else if(sdfVisual.getGeometry().getBox() != null)
         {
            String[] boxDimensions = sdfVisual.getGeometry().getBox().getSize().split(" ");            
            double bx = Double.parseDouble(boxDimensions[0]);
            double by = Double.parseDouble(boxDimensions[1]);
            double bz = Double.parseDouble(boxDimensions[2]);
            addCube(bx, by, bz, YoAppearance.AluminumMaterial());
         }
         else if(sdfVisual.getGeometry().getSphere() != null)
         {
            double radius = Double.parseDouble(sdfVisual.getGeometry().getSphere().getRadius());
            addSphere(radius, YoAppearance.White());
         }
         else
         {
            System.err.println("Visual for " + sdfVisual.getName() + " not implemented yet");
         }

         identity();
      }
      identity();
   }
   
   private void addMesh(String mesh, Transform3D visualPose)
   {
      

      AppearanceDefinition appearance = null;
      if (ModelFileType.getFileType(mesh) == ModelFileType._STL)
      {
         appearance = YoAppearance.BlackMetalMaterial(); // Otherwise it becomes a white blob
      }
      

      addModelFile(mesh, appearance);

   }
   
   private String convertToFullPath(String resourceDirectory, String meshPath)
   {
      try
      {
         URI meshURI = new URI(meshPath);
         return resourceDirectory + meshURI.getAuthority() + meshURI.getPath();

      }
      catch (URISyntaxException e)
      {
         return meshPath;
      }

   }

}
