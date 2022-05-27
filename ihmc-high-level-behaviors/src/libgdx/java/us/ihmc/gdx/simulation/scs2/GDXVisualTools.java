package us.ihmc.gdx.simulation.scs2;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.attributes.BlendingAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import us.ihmc.gdx.tools.GDXModelLoader;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.gizmo.DynamicGDXModel;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;

import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;

public class GDXVisualTools
{
   private static final Color DEFAULT_COLOR = Color.BLUE;
   private static final Material DEFAULT_MATERIAL = new Material(ColorAttribute.createDiffuse(DEFAULT_COLOR));

   public static List<DynamicGDXModel> collectNodes(List<VisualDefinition> visualDefinitions)
   {
      return collectNodes(visualDefinitions, null);
   }

   public static List<DynamicGDXModel> collectNodes(List<VisualDefinition> visualDefinitions, ClassLoader resourceClassLoader)
   {
      return visualDefinitions.stream()
                              .map(definition -> toNode(definition, resourceClassLoader))
                              .filter(Objects::nonNull)
                              .collect(Collectors.toList());
   }

   public static DynamicGDXModel toNode(VisualDefinition visualDefinition, ClassLoader resourceClassLoader)
   {
      DynamicGDXModel node = toShape3D(visualDefinition.getGeometryDefinition(), visualDefinition.getMaterialDefinition(), resourceClassLoader);

      if (node != null && visualDefinition.getOriginPose() != null)
      {
         node.getLocalTransform().set(visualDefinition.getOriginPose());
      }

      return node;
   }

   public static List<DynamicGDXModel> collectCollisionNodes(List<CollisionShapeDefinition> collisionShapeDefinitions)
   {
      return collisionShapeDefinitions.stream()
                                      .map(GDXVisualTools::toNode)
                                      .filter(Objects::nonNull)
                                      .collect(Collectors.toList());
   }

   public static DynamicGDXModel toNode(CollisionShapeDefinition collisionShapeDefinition)
   {
      ColorDefinition diffuseColor = ColorDefinitions.DarkRed();
      diffuseColor.setAlpha(0.6);
      MaterialDefinition materialDefinition = new MaterialDefinition(diffuseColor);
      DynamicGDXModel node = toShape3D(collisionShapeDefinition.getGeometryDefinition(), materialDefinition, null);

      if (node != null && collisionShapeDefinition.getOriginPose() != null)
      {
         node.getLocalTransform().set(collisionShapeDefinition.getOriginPose());
      }

      return node;
   }

   public static DynamicGDXModel toShape3D(GeometryDefinition geometryDefinition, MaterialDefinition materialDefinition, ClassLoader resourceClassLoader)
   {
      DynamicGDXModel gdxModel = new DynamicGDXModel(); // TODO: Should just pass DynamicGDXModel around?
      if (geometryDefinition instanceof ModelFileGeometryDefinition)
      {
         ModelFileGeometryDefinition modelFileGeometryDefinition = (ModelFileGeometryDefinition) geometryDefinition;
         String modelFileName = modelFileGeometryDefinition.getFileName();

         if (modelFileName == null)
            return null;

         gdxModel.setModel(GDXModelLoader.load(modelFileName));

         if (materialDefinition != null && materialDefinition.getDiffuseColor() != null)
         {
            for (Material material : gdxModel.getModel().materials)
            {
               Color color = toColor(materialDefinition.getDiffuseColor(), Color.WHITE);
               material.set(ColorAttribute.createDiffuse(color));
               if (materialDefinition.getDiffuseColor().getAlpha() < 1.0)
               {
                  material.set(new BlendingAttribute(true, (float) materialDefinition.getDiffuseColor().getAlpha()));
               }
            }
         }
      }
      else
      {
         gdxModel.setMaterial(toMaterial(materialDefinition));
         Mesh mesh = GDXTriangleMesh3DDefinitionInterpreter.interpretDefinition(TriangleMesh3DFactories.TriangleMesh(geometryDefinition), false);
         gdxModel.setMesh(mesh);
      }
      return gdxModel;
   }

   public static Material toMaterial(MaterialDefinition materialDefinition)
   {
      if (materialDefinition == null)
         return DEFAULT_MATERIAL;

      Color color = toColor(materialDefinition.getDiffuseColor(), Color.WHITE);

      Material attributes = new Material(ColorAttribute.createDiffuse(color));

      if (materialDefinition.getDiffuseColor() != null && materialDefinition.getDiffuseColor().getAlpha() < 1.0)
      {
         attributes.set(new BlendingAttribute(true, (float) materialDefinition.getDiffuseColor().getAlpha()));
      }

      if (materialDefinition.getDiffuseMap() != null && materialDefinition.getDiffuseMap().getFilename() != null)
      {
         Texture textureFromFile = new Texture(materialDefinition.getDiffuseMap().getFilename());
         attributes.set(TextureAttribute.createDiffuse(textureFromFile));
      }

      return attributes;
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
