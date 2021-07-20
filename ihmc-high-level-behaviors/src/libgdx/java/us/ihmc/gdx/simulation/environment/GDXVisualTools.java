package us.ihmc.gdx.simulation.environment;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import us.ihmc.gdx.tools.GDXModelLoader;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.gizmo.DynamicGDXModel;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;

import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;

public class GDXVisualTools
{
   private static final Color DEFAULT_COLOR = Color.BLUE;
   private static final Material DEFAULT_MATERIAL = new Material(ColorAttribute.createDiffuse(DEFAULT_COLOR));

   public static ModelInstance collectNodes(List<VisualDefinition> visualDefinitions)
   {
      return collectNodes(visualDefinitions, null);
   }

   public static ModelInstance collectNodes(List<VisualDefinition> visualDefinitions, ClassLoader resourceClassLoader)
   {
      List<ModelInstance> nodes = visualDefinitions.stream()
                                                   .map(definition -> toNode(definition, resourceClassLoader))
                                                   .filter(Objects::nonNull)
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
         ModelFileGeometryDefinition modelFileGeometryDefinition = (ModelFileGeometryDefinition) geometryDefinition;
         String fileName = modelFileGeometryDefinition.getFileName();

         String modifiedFileName = GDXModelLoader.modifyFileName(fileName);
         if (modifiedFileName == null)
            return null;

         return new ModelInstance(GDXModelLoader.loadG3DModel(modifiedFileName));
      }

      DynamicGDXModel gdxModel = new DynamicGDXModel(); // TODO: Should just pass DynamicGDXModel around?
      gdxModel.setMaterial(toMaterial(materialDefinition));
      Mesh mesh = GDXTriangleMesh3DDefinitionInterpreter.interpretDefinition(TriangleMesh3DFactories.TriangleMesh(geometryDefinition), false);
      gdxModel.setMesh(mesh);
      return gdxModel.getOrCreateModelInstance();
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
}
