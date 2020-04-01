package us.ihmc.modelFileLoaders.SdfLoader;

import java.io.File;
import java.net.URI;
import java.net.URISyntaxException;
import java.net.URL;
import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.builder.ToStringBuilder;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.ModelFileType;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.HeightBasedTerrainBlend;
import us.ihmc.graphicsDescription.appearance.SDFAppearance;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.appearance.YoAppearanceMaterial;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.modelFileLoaders.ModelFileLoaderConversionsHelper;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.AbstractSDFMesh;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFGeometry;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFGeometry.HeightMap.Blend;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFGeometry.HeightMap.Texture;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFGeometry.Mesh;
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
      this(sdfVisuals, resourceDirectories, (ClassLoader) null);
   }

   public SDFGraphics3DObject(List<? extends AbstractSDFMesh> sdfVisuals, List<String> resourceDirectories, ClassLoader resourceClassLoader)
   {
      this(sdfVisuals, resourceDirectories, resourceClassLoader, new RigidBodyTransform());
   }

   public SDFGraphics3DObject(List<? extends AbstractSDFMesh> sdfVisuals, List<String> resourceDirectories, RigidBodyTransform graphicsTransform)
   {
      this(sdfVisuals, resourceDirectories, null, graphicsTransform);
   }

   public SDFGraphics3DObject(List<? extends AbstractSDFMesh> sdfVisuals, List<String> resourceDirectories, ClassLoader resourceClassLoader,
                              RigidBodyTransform graphicsTransform)
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
                           String id = convertToResourceIdentifier(resourceDirectories, resourceClassLoader, uri);
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
               String resourceUrl = convertToResourceIdentifier(resourceDirectories, resourceClassLoader, mesh.getUri());
               if(mesh.getScale() != null)
               {
                  Vector3D scale = ModelFileLoaderConversionsHelper.stringToVector3d(mesh.getScale());
                  scale(scale);
               }
               String submesh = null;
               boolean centerSubmesh = false;
               if(mesh.getSubmesh() != null)
               {
                  submesh = mesh.getSubmesh().getName().trim();
                  centerSubmesh = mesh.getSubmesh().getCenter().trim().equals("1") || mesh.getSubmesh().getCenter().trim().equals("true");
               }
               addMesh(resourceUrl, submesh, centerSubmesh, visualPose, appearance, resourceDirectories, resourceClassLoader);
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
               Vector3D normal = ModelFileLoaderConversionsHelper.stringToNormalizedVector3d(geometry.getPlane().getNormal());
               Vector2D size = ModelFileLoaderConversionsHelper.stringToVector2d(geometry.getPlane().getSize());

               AxisAngle planeRotation = EuclidGeometryTools.axisAngleFromZUpToVector3D(normal);
               rotate(planeRotation);
               addCube(size.getX(), size.getY(), 0.005, getDefaultAppearanceIfNull(appearance));
            }
            else if(geometry.getHeightMap() != null)
            {
               String id = convertToResourceIdentifier(resourceDirectories, resourceClassLoader, geometry.getHeightMap().getUri());
               SDFHeightMap heightMap = new SDFHeightMap(id, geometry.getHeightMap());


               AppearanceDefinition app = DEFAULT_APPEARANCE;
               if(geometry.getHeightMap().getTextures() != null)
               {
                  double width = heightMap.getBoundingBox().getMaxX() - heightMap.getBoundingBox().getMinX();
                  HeightBasedTerrainBlend sdfTerrainBlend = new HeightBasedTerrainBlend(heightMap);
                  for(Texture text : geometry.getHeightMap().getTextures())
                  {
                     double size = Double.parseDouble(text.getSize());
                     double scale = width/size;
                     sdfTerrainBlend.addTexture(scale, convertToResourceIdentifier(resourceDirectories, resourceClassLoader, text.getDiffuse()),
                           convertToResourceIdentifier(resourceDirectories, resourceClassLoader, text.getNormal()));
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

   private void addMesh(String mesh, String submesh, boolean centerSubmesh, RigidBodyTransform visualPose, AppearanceDefinition appearance,
                        List<String> resourceDirectories, ClassLoader resourceClassLoader)
   {

      // STL files do not have appearances
      if (ModelFileType.getModelTypeFromId(mesh) == ModelFileType._STL)
      {
         appearance = getDefaultAppearanceIfNull(appearance);
      }
      addModelFile(mesh, submesh, centerSubmesh, resourceDirectories, resourceClassLoader, appearance);
   }

   private String convertToResourceIdentifier(List<String> resourceDirectories, ClassLoader resourceClassLoader, String meshPath)
   {
      if(meshPath.equals("__default__"))
      {
         meshPath = "file://media/materials/scripts/gazebo.material";
      }

      if (meshPath.contains("models/silverspine_roll.obj"))
         System.out.println();

      if (resourceDirectories.size() == 0)
      {
         String id = tryConversion(meshPath, "", resourceClassLoader);
         if (id != null) return id;
      }

      for (String resourceDirectory : resourceDirectories)
      {
         String id = tryConversion(meshPath, resourceDirectory, resourceClassLoader);
         if (id != null) return id;
      }

      System.out.println(meshPath);
      throw new RuntimeException("Resource not found: " + meshPath);
   }

   private String tryConversion(String meshPath, String resourceDirectory, ClassLoader resourceClassLoader)
   {
      try
      {
         URI meshURI = new URI(meshPath);

         String authority = meshURI.getAuthority() == null ? "" : meshURI.getAuthority();
         String id = resourceDirectory + authority + meshURI.getPath();
//            System.out.println("PATH: " + meshURI.getPath());
//            System.out.println("AUTH: " + meshURI.getAuthority());
//            System.out.println("ID: " + id);
         if (resourceClassLoader == null)
            resourceClassLoader = getClass().getClassLoader();
         URL resource = resourceClassLoader.getResource(id);
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
