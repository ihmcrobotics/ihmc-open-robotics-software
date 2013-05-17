package us.ihmc.SdfLoader;

import java.io.File;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.List;

import javax.media.j3d.Transform3D;
import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import org.apache.commons.lang.builder.ToStringBuilder;

import us.ihmc.SdfLoader.xmlDescription.SDFGeometry.HeightMap.Blend;
import us.ihmc.SdfLoader.xmlDescription.SDFGeometry.HeightMap.Texture;
import us.ihmc.SdfLoader.xmlDescription.SDFVisual;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.ModelFileType;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.HeightBasedTerrainBlend;
import us.ihmc.graphics3DAdapter.graphics.appearances.SDFAppearance;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.utilities.math.geometry.GeometryTools;


public class SDFGraphics3DObject extends Graphics3DObject
{
   private static final boolean SHOW_COORDINATE_SYSTEMS = false;
   private static final AppearanceDefinition DEFAULT_APPEARANCE = YoAppearance.White();
   
   public SDFGraphics3DObject(List<SDFVisual> sdfVisuals, ArrayList<String> resourceDirectories)
   {
      this(sdfVisuals, resourceDirectories, new Transform3D());
   }
   
   public SDFGraphics3DObject(List<SDFVisual> sdfVisuals, ArrayList<String> resourceDirectories, Transform3D graphicsTransform)
   {
      Matrix3d rotation = new Matrix3d();
      Vector3d offset = new Vector3d();
      graphicsTransform.get(rotation, offset);
      
      for(SDFVisual sdfVisual : sdfVisuals)
      {
         identity();
         translate(offset);
         rotate(rotation);
         
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
               
               appearance = new SDFAppearance(paths, name, resourceDirectories);
            }
         }
         
         if(sdfVisual.getGeometry().getMesh() != null)
         {
            String uri = convertToFullPath(resourceDirectories, sdfVisual.getGeometry().getMesh().getUri());
            if(sdfVisual.getGeometry().getMesh().getScale() != null)
            {
               Vector3d scale = SDFConversionsHelper.stringToVector3d(sdfVisual.getGeometry().getMesh().getScale());
               scale(scale);
            }
            String submesh = null;
            boolean centerSubmesh = false;
            if(sdfVisual.getGeometry().getMesh().getSubmesh() != null)
            {
               submesh = sdfVisual.getGeometry().getMesh().getSubmesh().getName().trim();
               centerSubmesh = sdfVisual.getGeometry().getMesh().getSubmesh().getCenter().trim().equals("1");
            }
            addMesh(uri, submesh, centerSubmesh, visualPose, appearance, resourceDirectories);
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
         else if(sdfVisual.getGeometry().getPlane() != null)
         {
            Vector3d normal = SDFConversionsHelper.stringToNormalizedVector3d(sdfVisual.getGeometry().getPlane().getNormal());
            Vector2d size = SDFConversionsHelper.stringToVector2d(sdfVisual.getGeometry().getPlane().getSize());
            
            AxisAngle4d planeRotation = GeometryTools.getRotationBasedOnNormal(normal);
            rotate(planeRotation);
            addCube(size.x, size.y, 0.005, getDefaultAppearanceIfNull(appearance));
         }
         else if(sdfVisual.getGeometry().getHeightMap() != null)
         {
            String URI = convertToFullPath(resourceDirectories, sdfVisual.getGeometry().getHeightMap().getUri());
            SDFHeightMap heightMap = new SDFHeightMap(URI, sdfVisual.getGeometry().getHeightMap());
            
            
            AppearanceDefinition app = DEFAULT_APPEARANCE;
            if(sdfVisual.getGeometry().getHeightMap().getTextures() != null)
            {
               double width = heightMap.getXMax() - heightMap.getXMin();
               HeightBasedTerrainBlend sdfTerrainBlend = new HeightBasedTerrainBlend(heightMap);
               for(Texture text : sdfVisual.getGeometry().getHeightMap().getTextures())
               {
                  double size = Double.parseDouble(text.getSize());
                  double scale = width/size;
                  sdfTerrainBlend.addTexture(scale, convertToFullPath(resourceDirectories, text.getDiffuse()),
                        convertToFullPath(resourceDirectories, text.getNormal()));
               }
               
               for(Blend blend : sdfVisual.getGeometry().getHeightMap().getBlends())
               {
                  sdfTerrainBlend.addBlend(Double.parseDouble(blend.getMinHeight()), Double.parseDouble(blend.getFadeDist()));
               }
               
               app = sdfTerrainBlend;
            }
            translate(heightMap.getOffset());
            addHeightMap(heightMap, 1000, 1000, app);
         }
         else
         {
            System.err.println("Visual for " + sdfVisual.getName() + " not implemented yet");
            System.err.println("Defined visual" + ToStringBuilder.reflectionToString(sdfVisual.getGeometry()));
            
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
   
   private void addMesh(String mesh, String submesh, boolean centerSubmesh, Transform3D visualPose, AppearanceDefinition appearance, ArrayList<String> resourceDirectories)
   {

      // STL files do not have appearances
      if (ModelFileType.getFileType(mesh) == ModelFileType._STL)
      {
         appearance = getDefaultAppearanceIfNull(appearance);
      }
      

      addModelFile(mesh, submesh, centerSubmesh, resourceDirectories, appearance);

   }

   private String convertToFullPath(ArrayList<String> resourceDirectories, String meshPath)
   {
      if(meshPath.equals("__default__"))
      {
         meshPath = "file://media/materials/scripts/gazebo.material";
      }
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
