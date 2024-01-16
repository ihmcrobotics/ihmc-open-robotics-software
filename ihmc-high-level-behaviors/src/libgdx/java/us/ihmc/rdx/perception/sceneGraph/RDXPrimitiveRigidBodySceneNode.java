package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImFloat;
import us.ihmc.euclid.transform.RigidBodyTransform;
<<<<<<< Updated upstream
=======
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D32;
>>>>>>> Stashed changes
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.perception.sceneGraph.rigidBody.primitive.PrimitiveRigidBodySceneNode;
import us.ihmc.perception.sceneGraph.rigidBody.primitive.PrimitiveRigidBodyShape;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.ui.RDX3DPanel;

import java.util.Set;

/**
 * A "ghost" colored model.
 */
public class RDXPrimitiveRigidBodySceneNode extends RDXRigidBodySceneNode
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   private static final float DEFAULT_DIMENSION = 0.1F;

   private RDXModelInstance modelInstance;

   private final ImFloat xLength = new ImFloat(DEFAULT_DIMENSION);
   private final ImFloat yLength = new ImFloat(DEFAULT_DIMENSION);
   private final ImFloat zLength = new ImFloat(DEFAULT_DIMENSION);
   private final ImFloat xRadius = new ImFloat(DEFAULT_DIMENSION);
   private final ImFloat yRadius = new ImFloat(DEFAULT_DIMENSION);
   private final ImFloat zRadius = new ImFloat(DEFAULT_DIMENSION);

   public RDXPrimitiveRigidBodySceneNode(PrimitiveRigidBodySceneNode primitiveRigidBodySceneNode, RDX3DPanel panel3D)
   {
      super(primitiveRigidBodySceneNode, new RigidBodyTransform(), panel3D);

      switch (primitiveRigidBodySceneNode.getShape())
      {
<<<<<<< Updated upstream
         case BOX -> modelInstance = new RDXModelInstance(RDXModelBuilder.createBox(DEFAULT_DIMENSION, DEFAULT_DIMENSION, DEFAULT_DIMENSION, Color.WHITE));
         case PRISM -> modelInstance = new RDXModelInstance(RDXModelBuilder.createPrism(DEFAULT_DIMENSION, DEFAULT_DIMENSION, DEFAULT_DIMENSION, Color.WHITE));
         case CYLINDER -> modelInstance = new RDXModelInstance(RDXModelBuilder.createCylinder(DEFAULT_DIMENSION, DEFAULT_DIMENSION, Color.WHITE));
         case ELLIPSOID ->
               modelInstance = new RDXModelInstance(RDXModelBuilder.createEllipsoid(DEFAULT_DIMENSION, DEFAULT_DIMENSION, DEFAULT_DIMENSION, Color.WHITE));
         case CONE -> modelInstance = new RDXModelInstance(RDXModelBuilder.createCone(DEFAULT_DIMENSION, DEFAULT_DIMENSION, Color.WHITE));
=======
         case BOX -> modelInstance = new RDXModelInstance(RDXModelBuilder.createBox(lengths.getX32(), lengths.getY32(), lengths.getZ32(), Color.WHITE));
         case PRISM -> modelInstance = new RDXModelInstance(RDXModelBuilder.createPrism(lengths.getX32(),
                                                                                        lengths.getY32(),
                                                                                        lengths.getZ32(),
                                                                                        new Point3D(0, 0, -zLength.get() / 2),
                                                                                        Color.WHITE));
         case CYLINDER -> modelInstance = new RDXModelInstance(RDXModelBuilder.createCylinder(lengths.getZ32(),
                                                                                              radii.getX32(),
                                                                                              new Point3D(0, 0, -zLength.get() / 2),
                                                                                              Color.WHITE));
         case ELLIPSOID -> modelInstance = new RDXModelInstance(RDXModelBuilder.createEllipsoid(radii.getX32(),
                                                                                                radii.getY32(),
                                                                                                radii.getZ32(),
                                                                                                new Point3D(),
                                                                                                Color.WHITE));
         case CONE -> modelInstance = new RDXModelInstance(RDXModelBuilder.createCone(lengths.getZ32(),
                                                                                      radii.getX32(),
                                                                                      new Point3D(0, 0, -zLength.get() / 2),
                                                                                      Color.WHITE));
>>>>>>> Stashed changes
      }
      modelInstance.setColor(GHOST_COLOR);
   }

   @Override
   public void renderImGuiWidgets(SceneGraphModificationQueue modificationQueue, SceneGraph sceneGraph)
   {
      super.renderImGuiWidgets(modificationQueue, sceneGraph);

      ImGui.text("Modify shape:");

      PrimitiveRigidBodyShape shape = ((PrimitiveRigidBodySceneNode) getSceneNode()).getShape();

      switch (shape)
      {
         case BOX ->
         {
            boolean reshaped = false;
            if (ImGuiTools.volatileInputFloat(labels.get("depth"), xLength))
               reshaped = true;
            if (ImGuiTools.volatileInputFloat(labels.get("width"), yLength))
               reshaped = true;
            if (ImGuiTools.volatileInputFloat(labels.get("height"), zLength))
               reshaped = true;
            if (reshaped)
            {
               if (modelInstance != null)
                  modelInstance.model.dispose();
               modelInstance = new RDXModelInstance(RDXModelBuilder.createBox(xLength.get(), yLength.get(), zLength.get(), Color.WHITE));
               modelInstance.setColor(GHOST_COLOR);
            }
         }
         case PRISM ->
         {
            boolean reshaped = false;
            if (ImGuiTools.volatileInputFloat(labels.get("depth"), xLength))
               reshaped = true;
            if (ImGuiTools.volatileInputFloat(labels.get("width"), yLength))
               reshaped = true;
            if (ImGuiTools.volatileInputFloat(labels.get("height"), zLength))
               reshaped = true;
            if (reshaped)
            {
               if (modelInstance != null)
                  modelInstance.model.dispose();
               modelInstance = new RDXModelInstance(RDXModelBuilder.createPrism(xLength.get(),
                                                                                yLength.get(),
                                                                                zLength.get(),
                                                                                new Point3D(0, 0, -zLength.get() / 2),
                                                                                Color.WHITE));
               modelInstance.setColor(GHOST_COLOR);
            }
         }
         case CYLINDER ->
         {
            boolean reshaped = false;
            if (ImGuiTools.volatileInputFloat(labels.get("radius"), xRadius))
               reshaped = true;
            if (ImGuiTools.volatileInputFloat(labels.get("height"), zLength))
               reshaped = true;
            if (reshaped)
            {
               if (modelInstance != null)
                  modelInstance.model.dispose();
               modelInstance = new RDXModelInstance(RDXModelBuilder.createCylinder(zLength.get(),
                                                                                   xRadius.get(),
                                                                                   new Point3D(0, 0, -zLength.get() / 2),
                                                                                   Color.WHITE));
               modelInstance.setColor(GHOST_COLOR);
            }
         }
         case ELLIPSOID ->
         {
            boolean reshaped = false;
            if (ImGuiTools.volatileInputFloat(labels.get("xRadius"), xRadius))
               reshaped = true;
            if (ImGuiTools.volatileInputFloat(labels.get("yRadius"), yRadius))
               reshaped = true;
            if (ImGuiTools.volatileInputFloat(labels.get("zRadius"), zRadius))
               reshaped = true;
            if (reshaped)
            {
               if (modelInstance != null)
                  modelInstance.model.dispose();
               modelInstance = new RDXModelInstance(RDXModelBuilder.createEllipsoid(xRadius.get(),
                                                                                    yRadius.get(),
                                                                                    zRadius.get(),
                                                                                    new Point3D(),
                                                                                    Color.WHITE));
               modelInstance.setColor(GHOST_COLOR);
            }
         }
         case CONE ->
         {
            boolean reshaped = false;
            if (ImGuiTools.volatileInputFloat(labels.get("radius"), xRadius))
               reshaped = true;
            if (ImGuiTools.volatileInputFloat(labels.get("height"), zLength))
               reshaped = true;
            if (reshaped)
            {
               if (modelInstance != null)
                  modelInstance.model.dispose();
               modelInstance = new RDXModelInstance(RDXModelBuilder.createCone(zLength.get(),
                                                                               xRadius.get(),
                                                                               new Point3D(0, 0, -zLength.get() / 2),
                                                                               Color.WHITE));
               modelInstance.setColor(GHOST_COLOR);
            }
         }
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      super.getRenderables(renderables, pool, sceneLevels);

      if (sceneLevels.contains(RDXSceneLevel.MODEL))
         modelInstance.getRenderables(renderables, pool);
   }

   @Override
   public RDXModelInstance getModelInstance()
   {
      return modelInstance;
   }
}
