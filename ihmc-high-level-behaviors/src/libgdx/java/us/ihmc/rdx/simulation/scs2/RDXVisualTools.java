package us.ihmc.rdx.simulation.scs2;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.attributes.BlendingAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.graphics.g3d.model.MeshPart;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import org.lwjgl.opengl.GL41;
import us.ihmc.rdx.tools.RDXModelLoader;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.gizmo.RDXVisualModelInstance;
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

public class RDXVisualTools
{
   private static final Color DEFAULT_COLOR = Color.BLUE;

   public static List<RDXVisualModelInstance> collectNodes(List<VisualDefinition> visualDefinitions)
   {
      return collectNodes(visualDefinitions, null);
   }

   public static List<RDXVisualModelInstance> collectNodes(List<VisualDefinition> visualDefinitions, ClassLoader resourceClassLoader)
   {
      return visualDefinitions.stream()
                              .map(definition -> toNode(definition, resourceClassLoader))
                              .filter(Objects::nonNull)
                              .collect(Collectors.toList());
   }

   public static RDXVisualModelInstance toNode(VisualDefinition visualDefinition, ClassLoader resourceClassLoader)
   {
      RDXVisualModelInstance node = toShape3D(visualDefinition.getGeometryDefinition(), visualDefinition.getMaterialDefinition(), resourceClassLoader);

      if (node != null && visualDefinition.getOriginPose() != null)
      {
         node.getLocalTransform().set(visualDefinition.getOriginPose());
      }

      return node;
   }

   public static List<RDXVisualModelInstance> collectCollisionNodes(List<CollisionShapeDefinition> collisionShapeDefinitions)
   {
      return collisionShapeDefinitions.stream()
                                      .map(RDXVisualTools::toNode)
                                      .filter(Objects::nonNull)
                                      .collect(Collectors.toList());
   }

   public static RDXVisualModelInstance toNode(CollisionShapeDefinition collisionShapeDefinition)
   {
      ColorDefinition diffuseColor = ColorDefinitions.DarkRed();
      diffuseColor.setAlpha(0.6);
      MaterialDefinition materialDefinition = new MaterialDefinition(diffuseColor);
      RDXVisualModelInstance node = toShape3D(collisionShapeDefinition.getGeometryDefinition(), materialDefinition, null);

      if (node != null && collisionShapeDefinition.getOriginPose() != null)
      {
         node.getLocalTransform().set(collisionShapeDefinition.getOriginPose());
      }

      return node;
   }

   public static RDXVisualModelInstance toShape3D(GeometryDefinition geometryDefinition, MaterialDefinition materialDefinition, ClassLoader resourceClassLoader)
   {
      RDXVisualModelInstance modelInstance;
      if (geometryDefinition instanceof ModelFileGeometryDefinition)
      {
         ModelFileGeometryDefinition modelFileGeometryDefinition = (ModelFileGeometryDefinition) geometryDefinition;
         String modelFileName = modelFileGeometryDefinition.getFileName();

         if (modelFileName == null)
            return null;

         modelInstance = new RDXVisualModelInstance(RDXModelLoader.load(modelFileName));

         if (materialDefinition != null && materialDefinition.getDiffuseColor() != null)
         {
            for (Material material : modelInstance.materials)
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
         ModelBuilder modelBuilder = new ModelBuilder();
         modelBuilder.begin();
         modelBuilder.node().id = geometryDefinition.getName();

         Mesh mesh = RDXTriangleMesh3DDefinitionInterpreter.interpretDefinition(TriangleMesh3DFactories.TriangleMesh(geometryDefinition), false);
         MeshPart meshPart = new MeshPart("xyz", mesh, 0, mesh.getNumIndices(), GL41.GL_TRIANGLES);
         modelBuilder.part(meshPart, toMaterial(materialDefinition));
         modelInstance = new RDXVisualModelInstance(modelBuilder.end());
      }
      return modelInstance;
   }

   public static Material toMaterial(MaterialDefinition materialDefinition)
   {
      if (materialDefinition == null)
         return new Material(ColorAttribute.createDiffuse(DEFAULT_COLOR));

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
         return LibGDXTools.toLibGDX(colorDefinition.getRed(), colorDefinition.getGreen(), colorDefinition.getBlue(), colorDefinition.getAlpha());
   }
}
