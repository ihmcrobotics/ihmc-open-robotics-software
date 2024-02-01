package us.ihmc.rdx.simulation.scs2;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.attributes.BlendingAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.graphics.g3d.model.MeshPart;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import gnu.trove.map.hash.TDoubleObjectHashMap;
import org.lwjgl.opengl.GL41;
import us.ihmc.mecano.multiBodySystem.CrossFourBarJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.rdx.tools.RDXModelInstanceScaler;
import us.ihmc.rdx.tools.RDXModelLoader;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.gizmo.RDXVisualModelInstance;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition;
import us.ihmc.scs2.definition.visual.*;

import java.util.HashMap;
import java.util.List;
import java.util.Objects;
import java.util.function.Consumer;
import java.util.stream.Collectors;

public class RDXVisualTools
{
   public static final double NO_SCALING = 1.0;
   /**
    * Make the desired ghost robot a little larger, so it shows up more cleanly.
    * This is because a desired ghost will generally be showing up partially colliding
    * or overlapping with an opaque estimate of the robot's current state.
    * It's possible that 10% is too much, if you feel it is, it probably is, and
    * change it.
    */
   public static final double DESIRED_ROBOT_SCALING = 1.1;
   private static final Color DEFAULT_COLOR = Color.BLUE;

   private static final Object modelDataLoadingSyncObject = new Object();
   private static final HashMap<String, TDoubleObjectHashMap<Model>> modelFileNameToScaledModelMap = new HashMap<>();

   public static List<RDXVisualModelInstance> collectNodes(List<VisualDefinition> visualDefinitions)
   {
      return collectNodes(visualDefinitions, NO_SCALING);
   }

   public static List<RDXVisualModelInstance> collectNodes(List<VisualDefinition> visualDefinitions, double scaleFactor)
   {
      return visualDefinitions.stream()
                              .map(definition -> toNode(definition, scaleFactor))
                              .filter(Objects::nonNull)
                              .collect(Collectors.toList());
   }

   public static RDXVisualModelInstance toNode(VisualDefinition visualDefinition, double scaleFactor)
   {
      RDXVisualModelInstance node = toShape3D(visualDefinition.getGeometryDefinition(), visualDefinition.getMaterialDefinition(), scaleFactor);

      if (node != null && visualDefinition.getOriginPose() != null)
      {
         node.getLocalTransform().set(visualDefinition.getOriginPose());
      }

      return node;
   }

   public static List<RDXVisualModelInstance> collectCollisionNodes(List<CollisionShapeDefinition> collisionShapeDefinitions)
   {
      return collectCollisionNodes(collisionShapeDefinitions, NO_SCALING);
   }

   public static List<RDXVisualModelInstance> collectCollisionNodes(List<CollisionShapeDefinition> collisionShapeDefinitions, double scaleFactor)
   {
      return collisionShapeDefinitions.stream()
                                      .map(collisionShapeDefinition -> toNode(collisionShapeDefinition, scaleFactor))
                                      .filter(Objects::nonNull)
                                      .collect(Collectors.toList());
   }

   public static RDXVisualModelInstance toNode(CollisionShapeDefinition collisionShapeDefinition, double scaleFactor)
   {
      ColorDefinition diffuseColor = ColorDefinitions.DarkRed();
      diffuseColor.setAlpha(0.6);
      MaterialDefinition materialDefinition = new MaterialDefinition(diffuseColor);
      RDXVisualModelInstance node = toShape3D(collisionShapeDefinition.getGeometryDefinition(), materialDefinition, scaleFactor);

      if (node != null && collisionShapeDefinition.getOriginPose() != null)
      {
         node.getLocalTransform().set(collisionShapeDefinition.getOriginPose());
      }

      return node;
   }

   public static RDXVisualModelInstance toShape3D(GeometryDefinition geometryDefinition, MaterialDefinition materialDefinition, double scaleFactor)
   {
      Model model;
      RDXVisualModelInstance modelInstance;
      boolean scaleNeeded = scaleFactor != NO_SCALING;
      if (geometryDefinition instanceof ModelFileGeometryDefinition modelFileGeometryDefinition)
      {
         String modelFileName = modelFileGeometryDefinition.getFileName();

         if (modelFileName == null)
            return null;

         if (scaleNeeded)
         {
            // Make loading faster by only loading each scaled model once each
            synchronized (modelDataLoadingSyncObject)
            {
               TDoubleObjectHashMap<Model> scaleToModelMap = modelFileNameToScaledModelMap.get(modelFileName);
               if (scaleToModelMap == null)
               {
                  scaleToModelMap = new TDoubleObjectHashMap<>();
                  modelFileNameToScaledModelMap.put(modelFileName, scaleToModelMap);
               }

               model = scaleToModelMap.get(scaleFactor);
               if (model == null)
               {
                  RDXModelInstanceScaler scaler = new RDXModelInstanceScaler(modelFileName);
                  model = scaler.scaleForModel(scaleFactor);
                  scaleToModelMap.put(scaleFactor, model);
               }
            }
         }
         else
         {
            model = RDXModelLoader.load(modelFileName);
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

         Model unscaledModel = modelBuilder.end();

         if (scaleNeeded)
         {
//            RDXModelInstanceScaler scaler = new RDXModelInstanceScaler(unscaledModel);
//            model = scaler.scaleForModel(scaleFactor);
            model = unscaledModel; // TODO: Fix scaling for meshes
         }
         else
         {
            model = unscaledModel;
         }
      }

      modelInstance = new RDXVisualModelInstance(model);

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

   // TODO: Figure out how to make this general to all things that extend RigidBody
   public static void collectRDXRigidBodiesIncludingPossibleFourBars(RDXRigidBody rigidBody, Consumer<RDXRigidBody> rigidBodyConsumer)
   {
      rigidBodyConsumer.accept(rigidBody);
      for (JointBasics childrenJoint : rigidBody.getChildrenJoints())
      {
         if (childrenJoint instanceof CrossFourBarJoint fourBarJoint)
         {
            RDXRigidBody bodyDA = (RDXRigidBody) fourBarJoint.getJointA().getSuccessor();
            rigidBodyConsumer.accept(bodyDA);
            RDXRigidBody bodyBC = (RDXRigidBody) fourBarJoint.getJointB().getSuccessor();
            rigidBodyConsumer.accept(bodyBC);
         }
      }
   }
}
