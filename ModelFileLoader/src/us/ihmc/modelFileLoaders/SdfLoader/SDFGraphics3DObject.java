package us.ihmc.modelFileLoaders.SdfLoader;

import java.io.File;
import java.net.URI;
import java.net.URISyntaxException;
import java.net.URL;
import java.util.ArrayList;
import java.util.List;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import org.apache.commons.lang3.builder.ToStringBuilder;

import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.modelFileLoaders.ModelFileLoaderConversionsHelper;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.AbstractSDFMesh;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFGeometry;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFGeometry.HeightMap.Blend;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFGeometry.HeightMap.Texture;
import us.ihmc.graphicsDescription.ModelFileType;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.HeightBasedTerrainBlend;
import us.ihmc.graphicsDescription.appearance.SDFAppearance;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.appearance.YoAppearanceMaterial;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFGeometry.Mesh;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;


public class SDFGraphics3DObject extends LinkGraphicsDescription
{
   private static final boolean SHOW_COORDINATE_SYSTEMS = false;
   private static final AppearanceDefinition DEFAULT_APPEARANCE = YoAppearance.Orange();
   static
   {
      YoAppearance.makeTransparent(DEFAULT_APPEARANCE, 0.4);
   }

   public SDFGraphics3DObject(List<? extends AbstractSDFMesh> sdfVisuals, List<String> resourceDirectories)
   {
      this(sdfVisuals, resourceDirectories, new RigidBodyTransform());
   }

   public SDFGraphics3DObject(List<? extends AbstractSDFMesh> sdfVisuals, List<String> resourceDirectories, RigidBodyTransform graphicsTransform)
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

            RigidBodyTransform visualPose = ModelFileLoaderConversionsHelper.poseToTransform(sdfVisual.getPose());
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
                  ArrayList<String> resourceUrls = new ArrayList<>();

                  if(sdfVisual.getMaterial().getScript().getUri() != null)
                  {
                     for(String uri : sdfVisual.getMaterial().getScript().getUri())
                     {
                        if(uri.equals("__default__"))
                        {
                           resourceUrls.add("/scripts/gazebo.material");
                        }
                        else
                        {
                           String id = convertToResourceIdentifier(resourceDirectories, uri);
                           resourceUrls.add(id);
                        }
                     }
                  }

                  String name = sdfVisual.getMaterial().getScript().getName();

                  appearance = new SDFAppearance(resourceUrls, name, resourceDirectories);
               }
               else
               {
                  YoAppearanceMaterial mat = new YoAppearanceMaterial();

                  mat.setAmbientColor(ModelFileLoaderConversionsHelper.stringToColor(sdfVisual.getMaterial().getAmbient()));
                  mat.setDiffuseColor(ModelFileLoaderConversionsHelper.stringToColor(sdfVisual.getMaterial().getDiffuse()));
                  mat.setSpecularColor(ModelFileLoaderConversionsHelper.stringToColor(sdfVisual.getMaterial().getSpecular()));

                  appearance = mat;
               }
            }

            if (sdfVisual.getTransparency() != null)
            {
               // An appearance must exist in order to set a transparency value, whether a material was defined above or not.
               if (appearance == null)
               {
                  appearance = new YoAppearanceRGBColor(DEFAULT_APPEARANCE.getColor(), 0.0);
               }
               appearance.setTransparency(Double.parseDouble(sdfVisual.getTransparency()));
            }

            SDFGeometry geometry = sdfVisual.getGeometry();
            Mesh mesh = geometry.getMesh();
            if(mesh != null)
            {
               String resourceUrl = convertToResourceIdentifier(resourceDirectories, mesh.getUri());
               if(mesh.getScale() != null)
               {
                  Vector3d scale = ModelFileLoaderConversionsHelper.stringToVector3d(mesh.getScale());
                  scale(scale);
               }
               String submesh = null;
               boolean centerSubmesh = false;
               if(mesh.getSubmesh() != null)
               {
                  submesh = mesh.getSubmesh().getName().trim();
                  centerSubmesh = mesh.getSubmesh().getCenter().trim().equals("1") || mesh.getSubmesh().getCenter().trim().equals("true");
               }
               addMesh(resourceUrl, submesh, centerSubmesh, visualPose, appearance, resourceDirectories);
            }
            else if(geometry.getCylinder() != null)
            {
               double length = Double.parseDouble(geometry.getCylinder().getLength());
               double radius = Double.parseDouble(geometry.getCylinder().getRadius());
               translate(0.0, 0.0, -length/2.0);
               addCylinder(length, radius, getDefaultAppearanceIfNull(appearance));
            }
            else if(geometry.getBox() != null)
            {
               String[] boxDimensions = geometry.getBox().getSize().split(" ");
               double bx = Double.parseDouble(boxDimensions[0]);
               double by = Double.parseDouble(boxDimensions[1]);
               double bz = Double.parseDouble(boxDimensions[2]);
               translate(0.0, 0.0, -bz/2.0);
               addCube(bx, by, bz, getDefaultAppearanceIfNull(appearance));
            }
            else if(geometry.getSphere() != null)
            {
               double radius = Double.parseDouble(geometry.getSphere().getRadius());
               addSphere(radius, getDefaultAppearanceIfNull(appearance));
            }
            else if(geometry.getPlane() != null)
            {
               Vector3d normal = ModelFileLoaderConversionsHelper.stringToNormalizedVector3d(geometry.getPlane().getNormal());
               Vector2d size = ModelFileLoaderConversionsHelper.stringToVector2d(geometry.getPlane().getSize());

               AxisAngle4d planeRotation = GeometryTools.getAxisAngleFromZUpToVector(normal);
               rotate(planeRotation);
               addCube(size.getX(), size.getY(), 0.005, getDefaultAppearanceIfNull(appearance));
            }
            else if(geometry.getHeightMap() != null)
            {
               String id = convertToResourceIdentifier(resourceDirectories, geometry.getHeightMap().getUri());
               SDFHeightMap heightMap = new SDFHeightMap(id, geometry.getHeightMap());


               AppearanceDefinition app = DEFAULT_APPEARANCE;
               if(geometry.getHeightMap().getTextures() != null)
               {
                  double width = heightMap.getBoundingBox().getXMax() - heightMap.getBoundingBox().getXMin();
                  HeightBasedTerrainBlend sdfTerrainBlend = new HeightBasedTerrainBlend(heightMap);
                  for(Texture text : geometry.getHeightMap().getTextures())
                  {
                     double size = Double.parseDouble(text.getSize());
                     double scale = width/size;
                     sdfTerrainBlend.addTexture(scale, convertToResourceIdentifier(resourceDirectories, text.getDiffuse()),
                           convertToResourceIdentifier(resourceDirectories, text.getNormal()));
                  }

                  for(Blend blend : geometry.getHeightMap().getBlends())
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
               System.err.println("Defined visual" + ToStringBuilder.reflectionToString(geometry));

            }


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

   private void addMesh(String mesh, String submesh, boolean centerSubmesh, RigidBodyTransform visualPose, AppearanceDefinition appearance, List<String> resourceDirectories)
   {

      // STL files do not have appearances
      if (ModelFileType.getModelTypeFromId(mesh) == ModelFileType._STL)
      {
         appearance = getDefaultAppearanceIfNull(appearance);
      }
      addModelFile(mesh, submesh, centerSubmesh, resourceDirectories, appearance);
   }

   private String convertToResourceIdentifier(List<String> resourceDirectories, String meshPath)
   {
      if(meshPath.equals("__default__"))
      {
         meshPath = "file://media/materials/scripts/gazebo.material";
      }

      if (meshPath.contains("models/silverspine_roll.obj"))
         System.out.println();

      if (resourceDirectories.size() == 0)
      {
         String id = tryConversion(meshPath, "");
         if (id != null) return id;
      }

      for (String resourceDirectory : resourceDirectories)
      {
         String id = tryConversion(meshPath, resourceDirectory);
         if (id != null) return id;
      }

      System.out.println(meshPath);
      throw new RuntimeException("Resource not found: " + meshPath);
   }

   private String tryConversion(String meshPath, String resourceDirectory)
   {
      try
      {
         URI meshURI = new URI(meshPath);

         String authority = meshURI.getAuthority() == null ? "" : meshURI.getAuthority();
         String id = resourceDirectory + authority + meshURI.getPath();
//            System.out.println("PATH: " + meshURI.getPath());
//            System.out.println("AUTH: " + meshURI.getAuthority());
//            System.out.println("ID: " + id);
         URL resource = getClass().getClassLoader().getResource(id);
         // Path relative to class root
         if (resource != null)
         {
            return id;
         }
         // Absolute path
    	   File f = new File(id);
         if (f.exists())
         {
        	 return id;
         }
      }
      catch (URISyntaxException e)
      {
         System.err.println("Malformed resource path in .SDF file for path: " + meshPath);
      }

      return null;
   }
}
