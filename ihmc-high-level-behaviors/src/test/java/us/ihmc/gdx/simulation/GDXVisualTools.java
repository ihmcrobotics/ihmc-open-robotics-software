package us.ihmc.gdx.simulation;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.model.Node;
import org.apache.commons.io.FilenameUtils;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.gizmo.DynamicGDXModel;
import us.ihmc.javaFXToolkit.JavaFXTools;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.geometry.*;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;

import java.io.*;
import java.lang.reflect.Modifier;
import java.net.MalformedURLException;
import java.net.URISyntaxException;
import java.net.URL;
import java.util.*;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class GDXVisualTools
{
   private static final AxisAngle ROTATE_PI_X = new AxisAngle(Axis3D.X, 180.0);
   private static final Color DEFAULT_COLOR = Color.BLUE;
   private static final Material DEFAULT_MATERIAL = new Material(ColorAttribute.createDiffuse(DEFAULT_COLOR));
   private static final ModelInstance DEFAULT_GEOMETRY = null;

   public static ModelInstance collectNodes(List<VisualDefinition> visualDefinitions)
   {
      return collectNodes(visualDefinitions, null);
   }

   public static ModelInstance collectNodes(List<VisualDefinition> visualDefinitions, ClassLoader resourceClassLoader)
   {
      List<ModelInstance> nodes = visualDefinitions.stream().map(definition -> toNode(definition, resourceClassLoader)).filter(node -> node != null)
                                          .collect(Collectors.toList());

      if (nodes.isEmpty())
         return null;
      else if (nodes.size() == 1)
         return nodes.get(0);
      else
      {
         Model model = new Model();
         for (ModelInstance node : nodes)
         {
            model.nodes.addAll(node.nodes);
         }
         return new ModelInstance(model);
      }
   }

   public static ModelInstance toNode(VisualDefinition visualDefinition, ClassLoader resourceClassLoader)
   {
      ModelInstance node = toShape3D(visualDefinition.getGeometryDefinition(), visualDefinition.getMaterialDefinition(), resourceClassLoader);

      if (node != null && visualDefinition.getOriginPose() != null)
      {
         GDXTools.toGDX(visualDefinition.getOriginPose(), node.transform);
      }

      return node;
   }

   public static ModelInstance toShape3D(GeometryDefinition geometryDefinition, MaterialDefinition materialDefinition, ClassLoader resourceClassLoader)
   {
      if (geometryDefinition instanceof ModelFileGeometryDefinition)
      {
         Node[] nodes = importModel((ModelFileGeometryDefinition) geometryDefinition, resourceClassLoader);
         if (nodes == null)
            return null;

         //         if (materialDefinition != null)
         //         {
         //            Material material = toMaterial(materialDefinition);
         //            for (Node node : nodes)
         //               ((Shape3D) node).setMaterial(material);
         //         }
         return nodes.length == 1 ? nodes[0] : new Group(nodes);
      }

      DynamicGDXModel gdxModel = new DynamicGDXModel(); // TODO: Should just pass DynamicGDXModel around?
      gdxModel.setMaterial(toMaterial(materialDefinition));
      gdxModel.setMesh(GDXTriangleMesh3DDefinitionInterpreter.interpretDefinition(geometryDefinition));
      return gdxModel.getOrCreateModelInstance();

      return DEFAULT_GEOMETRY;
   }

   public static Node[] importModel(ModelFileGeometryDefinition geometryDefinition, ClassLoader resourceClassLoader)
   {
      if (geometryDefinition == null || geometryDefinition.getFileName() == null)
         return DEFAULT_MESH_VIEWS;

      String filename = geometryDefinition.getFileName();

      if (resourceClassLoader == null)
         resourceClassLoader = JavaFXTools.class.getClassLoader();
      URL fileURL = resourceClassLoader.getResource(filename);

      if (fileURL == null)
      {
         File file = new File(filename);
         try
         {
            fileURL = file.toURI().toURL();
         }
         catch (MalformedURLException e)
         {
            throw new RuntimeException(e);
         }
      }

      Node[] importedModel = importModel(fileURL);

      Vector3D scale = geometryDefinition.getScale();

      if (scale != null && importedModel != null && importedModel.length > 0)
      {
         if (importedModel.length == 1)
         {
            importedModel[0].getTransforms().add(new Scale(scale.getX(), scale.getY(), scale.getZ()));
         }
         else
         {
            Group group = new Group(importedModel);
            group.getTransforms().add(new Scale(scale.getX(), scale.getY(), scale.getZ()));
            importedModel = new Node[] {group};
         }
      }

      return importedModel;
   }

   public static Node[] importModel(URL fileURL)
   {
      try
      {
         String file = fileURL.getFile();
         String fileExtension = FilenameUtils.getExtension(file).toLowerCase();
         Node[] importedNodes;

         switch (fileExtension)
         {
            case "dae":
               importedNodes = GDXVisualTools.importColladaModel(fileURL);
               break;
            case "stl":
               importedNodes = GDXVisualTools.importSTLModel(fileURL);
               break;
            case "obj":
               importedNodes = GDXVisualTools.importOBJModel(fileURL);
               break;
            default:
               importedNodes = DEFAULT_MESH_VIEWS;
               break;
         }
         setNodeIDs(importedNodes, FilenameUtils.getBaseName(file), true);
         return importedNodes;
      }
      catch (Exception e)
      {
         LogTools.error("Could not import model file: " + fileURL.toExternalForm() + "\n " + e.getClass().getSimpleName() + ": " + e.getMessage());
         return null;
      }
   }

   public static Node[] importColladaModel(URL fileURL)
   {
      MeshView[] importedModel;
      CachedImportedModel cachedImportedModel = cachedColladaModels.get(fileURL.toExternalForm());

      if (cachedImportedModel == null)
      {
         ColModelImporter importer = new ColModelImporter();
         importer.getOptions().add(ColImportOption.IGNORE_LIGHTS);
         importer.getOptions().add(ColImportOption.IGNORE_CAMERAS);
         importer.getOptions().add(ColImportOption.GENERATE_NORMALS);
         importer.read(fileURL);

         ColAsset asset = importer.getAsset();
         importedModel = unwrapGroups(importer.getImport(), MeshView.class).toArray(new MeshView[0]);
         Stream.of(importedModel).forEach(model -> model.getTransforms().add(0, new Rotate(-90, Rotate.X_AXIS)));

         if (!EuclidCoreTools.epsilonEquals(1.0, asset.getUnitMeter(), 1.0e-3))
            Stream.of(importedModel)
                  .forEach(model -> model.getTransforms().add(0, new Scale(asset.getUnitMeter(), asset.getUnitMeter(), asset.getUnitMeter())));

         cacheMaterials(importer, importedModel, cachedNamedColladaMaterials);
         cachedColladaModels.put(fileURL.toExternalForm(), new CachedImportedModel(importedModel));
         importer.close();
      }
      else
      {
         importedModel = cachedImportedModel.newMeshViews();
      }

      return importedModel;
   }

   public static Node[] importSTLModel(URL fileURL)
   {
      StlMeshImporter importer = new StlMeshImporter();
      importer.read(fileURL);
      MeshView meshView = new MeshView(importer.getImport());
      meshView.getTransforms().add(ROTATE_PI_X);
      importer.close();
      return new Node[] {meshView};
   }

   public static Node[] importOBJModel(URL fileURL) throws URISyntaxException, IOException
   {
      MeshView[] importedModel;
      String externalForm = fileURL.toExternalForm();
      CachedImportedModel cachedImportedModel = cachedOBJModels.get(externalForm);

      if (cachedImportedModel == null)
      {
         ObjModelImporter importer = new ObjModelImporter();
         importer.setOptions(ObjImportOption.GENERATE_NORMALS);
         try
         {
            importer.read(fileURL);
         }
         catch (ImportException e)
         {
            if (e.getMessage().contains("Material not found"))
            {
               String filePath = fileURL.getPath();
               File tempFileWithoutMTL = new File(filePath.substring(0, filePath.lastIndexOf(".")) + "Temp.obj");
               if (tempFileWithoutMTL.exists())
               {
                  throw e;
               }
               tempFileWithoutMTL.createNewFile();
               BufferedReader bufferedReader = new BufferedReader(new FileReader(new File(fileURL.toURI())));
               BufferedWriter bufferedWriter = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(tempFileWithoutMTL)));

               String line;
               while ((line = bufferedReader.readLine()) != null)
               {
                  if (line.toLowerCase().startsWith("mtllib"))
                     continue;
                  if (line.toLowerCase().startsWith("usemtl"))
                     continue;

                  try
                  {
                     bufferedWriter.write(line);
                     bufferedWriter.newLine();
                  }
                  catch (Exception e1)
                  {
                     tempFileWithoutMTL.delete();
                     bufferedReader.close();
                     bufferedWriter.close();
                     throw e1;
                  }
               }

               bufferedReader.close();
               bufferedWriter.close();

               try
               {
                  importer.close();
                  importer = new ObjModelImporter();
                  importer.setOptions(ObjImportOption.GENERATE_NORMALS);
                  importer.read(tempFileWithoutMTL);
               }
               catch (Exception e1)
               {
                  tempFileWithoutMTL.delete();
                  throw e1;
               }
               tempFileWithoutMTL.delete();
            }
         }

         importedModel = importer.getImport();
         Stream.of(importedModel).forEach(model -> model.getTransforms().add(ROTATE_PI_X));
         cacheMaterials(importer, importedModel, cachedNamedOBJMaterials);
         cachedOBJModels.put(fileURL.toExternalForm(), new CachedImportedModel(importedModel));

         importer.close();
      }
      else
      {
         importedModel = cachedImportedModel.newMeshViews();
      }

      return importedModel;
   }

   private static void cacheMaterials(ModelImporter importer, MeshView[] importedModels, Map<Long, Material> cacheToUpdate)
   {
      Map<String, Material> namedMaterials = importer.getNamedMaterials();

      if (namedMaterials == null)
         return;

      Map<Image, FilePath> imageFilePaths = importer.getImageFilePaths();

      Map<Material, Long> materialToHashCodeMap = new HashMap<>();

      for (Map.Entry<String, Material> entry : namedMaterials.entrySet())
      {
         Material material = entry.getValue();
         long materialHashCode = entry.getKey().hashCode();

         Image materialImage = material.getDiffuseMap();
         if (materialImage != null)
            materialHashCode = 31L * materialHashCode + imageFilePaths.get(materialImage).getAbsolutePath().hashCode();
         materialImage = material.getSpecularMap();
         if (materialImage != null)
            materialHashCode = 31L * materialHashCode + imageFilePaths.get(materialImage).getAbsolutePath().hashCode();
         materialImage = material.getBumpMap();
         if (materialImage != null)
            materialHashCode = 31L * materialHashCode + imageFilePaths.get(materialImage).getAbsolutePath().hashCode();
         materialImage = material.getSelfIlluminationMap();
         if (materialImage != null)
            materialHashCode = 31L * materialHashCode + imageFilePaths.get(materialImage).getAbsolutePath().hashCode();

         materialToHashCodeMap.put(material, materialHashCode);
         cacheToUpdate.putIfAbsent(materialHashCode, material);
      }

      for (MeshView importedModel : importedModels)
      {
         Long materialHashCode = materialToHashCodeMap.get(importedModel.getMaterial());
         importedModel.setMaterial(cacheToUpdate.get(materialHashCode));
      }
   }

   public static <T extends Node> List<T> unwrapGroups(Node[] nodes, Class<T> filterClass)
   {
      if (nodes == null || nodes.length == 0)
         return null;

      List<T> filteredNodes = new ArrayList<>();
      for (Node node : nodes)
         filteredNodes.addAll(unwrapGroups(node, filterClass));
      return filteredNodes;
   }

   public static <T extends Node> List<T> unwrapGroups(Node node, Class<T> filterClass)
   {
      if (!(node.hasChildren()))
      {
         if (filterClass.isInstance(node))
            return Collections.singletonList(filterClass.cast(node));
         else
            return Collections.emptyList();
      }

      if (node.getChildCount() < 1)
         return Collections.emptyList();

      List<T> filteredNodes = new ArrayList<>();

      // TODO: What to do here?

      return filteredNodes;
   }

   public static Material toMaterial(MaterialDefinition materialDefinition)
   {
      if (materialDefinition == null)
         return DEFAULT_MATERIAL;

      Color color = toColor(materialDefinition.getDiffuseColor());
      return new Material(ColorAttribute.createDiffuse(color));
   }

   public static Color toColor(ColorDefinition colorDefinition)
   {
      return toColor(colorDefinition, DEFAULT_COLOR);
   }

   public static Color toColor(ColorDefinition colorDefinition, Color defaultValue)
   {
      if (colorDefinition == null)
         return defaultValue;
      else
         return GDXTools.toGDX(colorDefinition.getRed(), colorDefinition.getGreen(), colorDefinition.getBlue(), colorDefinition.getAlpha());
   }

   public static final List<String> colorNameList;

   static
   {
      List<String> list = Stream.of(Color.class.getDeclaredFields()).filter(field -> Modifier.isStatic(field.getModifiers()))
                                .filter(field -> Modifier.isPublic(field.getModifiers())).filter(field -> field.getType() == Color.class)
                                .map(field -> field.getName().toLowerCase()).collect(Collectors.toList());
      colorNameList = Collections.unmodifiableList(list);
   }

   public static void setNodeIDs(Node[] nodes, String id, boolean overrideExistingIDs)
   {
      if (nodes == null || nodes.length == 0)
         return;
      if (nodes.length == 1)
      {
         setNodeIDs(nodes[0], id, overrideExistingIDs);
      }
      else
      {
         for (int i = 0; i < nodes.length; i++)
         {
            setNodeIDs(nodes[i], id + "_" + i, overrideExistingIDs);
         }
      }
   }

   public static void setMissingNodeIDs(Node node, String id)
   {
      setNodeIDs(node, id, false);
   }

   public static void setNodeIDs(Node node, String id, boolean overrideExistingIDs)
   {
      if (node == null)
         return;

      if (node.id == null || overrideExistingIDs)
         node.id = id;

      if (node.hasChildren())
      {
         for (int i = 0; i < node.getChildCount(); i++)
         {
            setNodeIDs(node.getChild(i), id + "_" + i, overrideExistingIDs);
         }
      }
   }
}
