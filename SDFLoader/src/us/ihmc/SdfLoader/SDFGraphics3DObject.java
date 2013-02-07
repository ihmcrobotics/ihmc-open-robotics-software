package us.ihmc.SdfLoader;

import java.io.File;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.List;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.xmlDescription.SDFVisual;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.ModelFileType;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.SDFAppearance;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;


public class SDFGraphics3DObject extends Graphics3DObject
{
   private static final boolean SHOW_COORDINATE_SYSTEMS = false;
   private static final AppearanceDefinition DEFAULT_APPEARANCE = YoAppearance.White();

   public SDFGraphics3DObject(List<SDFVisual> sdfVisuals, ArrayList<String> resourceDirectories)
   {
      
      for(SDFVisual sdfVisual : sdfVisuals)
      {
         identity();
         
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
         
         AppearanceDefinition appearance = null;
         if(sdfVisual.getMaterial() != null)
         {
            if(sdfVisual.getMaterial().getScript() != null)
            {
               System.out.println("Loading SDF Material");
               ArrayList<String> paths = new ArrayList<String>();
               
               if(sdfVisual.getMaterial().getScript().getUri() != null)
               {
                  for(String uri : sdfVisual.getMaterial().getScript().getUri())
                  {
                     String fullPath = convertToFullPath(resourceDirectories, uri);
                     paths.add(fullPath);
                  }
               }
               
               String name = sdfVisual.getMaterial().getScript().getName();
               
               appearance = new SDFAppearance(paths, name);
            }
         }
         
         if(sdfVisual.getGeometry().getMesh() != null)
         {
            String uri = convertToFullPath(resourceDirectories, sdfVisual.getGeometry().getMesh().getUri());
            addMesh(uri, visualPose, appearance);
         }
         else if(sdfVisual.getGeometry().getCylinder() != null)
         {
            double length = Double.parseDouble(sdfVisual.getGeometry().getCylinder().getLength());
            double radius = Double.parseDouble(sdfVisual.getGeometry().getCylinder().getRadius()); 
            translate(0.0, 0.0, -length/2.0);
            addCylinder(length, radius, getDefaultAppearanceIfNull(appearance));
         }
         else if(sdfVisual.getGeometry().getBox() != null)
         {
            String[] boxDimensions = sdfVisual.getGeometry().getBox().getSize().split(" ");            
            double bx = Double.parseDouble(boxDimensions[0]);
            double by = Double.parseDouble(boxDimensions[1]);
            double bz = Double.parseDouble(boxDimensions[2]);
            translate(0.0, 0.0, -bz/2.0);
            addCube(bx, by, bz, getDefaultAppearanceIfNull(appearance));
         }
         else if(sdfVisual.getGeometry().getSphere() != null)
         {
            double radius = Double.parseDouble(sdfVisual.getGeometry().getSphere().getRadius());
            addSphere(radius, getDefaultAppearanceIfNull(appearance));
         }
         else
         {
            System.err.println("Visual for " + sdfVisual.getName() + " not implemented yet");
         }

         
      }
   }
   
   private static AppearanceDefinition getDefaultAppearanceIfNull(AppearanceDefinition appearance)
   {
      if(appearance == null)
      {
         return DEFAULT_APPEARANCE;
      }
      else
      {
         return appearance;
      }
   }
   
   private void addMesh(String mesh, Transform3D visualPose, AppearanceDefinition appearance)
   {

      // STL files do not have appearances
      if (ModelFileType.getFileType(mesh) == ModelFileType._STL)
      {
         appearance = getDefaultAppearanceIfNull(appearance);
      }
      

      addModelFile(mesh, appearance);

   }

   private String convertToFullPath(ArrayList<String> resourceDirectories, String meshPath)
   {
      for (String resourceDirectory : resourceDirectories)
      {
         String fullPath;
         try
         {
            URI meshURI = new URI(meshPath);
            fullPath = resourceDirectory + File.separator + meshURI.getAuthority() + meshURI.getPath();
         }
         catch (URISyntaxException e)
         {
            fullPath = meshPath;
         }
         
         File testFile = new File(fullPath);
         if(testFile.exists())
         {
            return fullPath;
         }
      }
      System.out.println(meshPath);
      throw new RuntimeException("Resource not found");
   }

}
