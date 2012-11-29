package us.ihmc.SdfLoader;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.List;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.xmlDescription.SDFVisual;
import us.ihmc.graphics3DAdapter.graphics.LinkGraphics;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearanceDefinition;

import com.yobotics.simulationconstructionset.ModelFileType;

public class SDFLinkGraphics extends LinkGraphics
{
   private static final boolean SHOW_COORDINATE_SYSTEMS = false;

   public SDFLinkGraphics(Matrix3d rotation, List<SDFVisual> sdfVisuals, String resourceDirectory)
   {
      rotate(rotation);
      
      for(SDFVisual sdfVisual : sdfVisuals)
      {
         Transform3D visualPose = SDFConversionsHelper.poseToTransform(sdfVisual.getPose());
         
         if(sdfVisual.getGeometry().getMesh() != null)
         {
            String uri = convertToFullPath(resourceDirectory, sdfVisual.getGeometry().getMesh().getUri());
            addMesh(uri, visualPose);
         }
         else
         {
            System.err.println("Visual for " + sdfVisual.getName() + " not implemented yet");
         }
      }
   }
   
   private void addMesh(String mesh, Transform3D visualPose)
   {
      Vector3d offset = new Vector3d();
      Matrix3d rotation = new Matrix3d();
      visualPose.get(rotation, offset);

      YoAppearanceDefinition appearance = null;
      if (ModelFileType.getFileType(mesh) == ModelFileType._STL)
      {
         appearance = YoAppearance.BlackMetalMaterial(); // Otherwise it becomes a white blob
      }
      if (SHOW_COORDINATE_SYSTEMS)
      {
         addCoordinateSystem(0.1);         
      }

      translate(offset);
      rotate(rotation);

      addModelFile(mesh, appearance);

      identity();
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
