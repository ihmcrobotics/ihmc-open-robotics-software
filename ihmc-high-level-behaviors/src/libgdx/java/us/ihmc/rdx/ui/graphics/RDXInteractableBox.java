package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

public class RDXInteractableBox implements RenderableProvider
{
   private final ResettableExceptionHandlingExecutorService executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
   private ModelInstance modelInstance;
   private Model lastModel;
   private volatile Runnable buildMeshAndCreateModelInstance = null;

   private final Point3D[] vertices = new Point3D[8];
   private final Color color = new Color(0.7f, 0.7f, 0.7f, 1.0f);
   private final RDXPose3DGizmo pose3DGizmo = new RDXPose3DGizmo();
   private final Box3D box3D;
   private final String name;

   private final RigidBodyTransform tempTransform = new RigidBodyTransform();

   public RDXInteractableBox(RDXBaseUI baseUI, Box3D box3D, String name)
   {
      this.name = name;

      pose3DGizmo.create(baseUI.getPrimary3DPanel());
      pose3DGizmo.getTransformToParent().getTranslation().set(box3D.getPosition());
      pose3DGizmo.getTransformToParent().getRotation().set(box3D.getOrientation());
      pose3DGizmo.update();

      baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(pose3DGizmo::calculate3DViewPick);
      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(pose3DGizmo::process3DViewInput);
      baseUI.getPrimaryScene().addRenderableProvider(pose3DGizmo);
      baseUI.getImGuiPanelManager().addPanel(pose3DGizmo.createTunerPanel(name));

      this.box3D = box3D;

      for (int i = 0; i < vertices.length; i++)
      {
         vertices[i] = new Point3D();
      }
      box3D.getVertices(vertices);
      generateMesh(box3D);
   }

   public void update()
   {
      pose3DGizmo.update();
      box3D.getPose().setToZero();
      box3D.getPose().applyTransform(pose3DGizmo.getTransformToParent());
//      generateMesh(box3D);

//      box3D.getVertices(vertices);
//      lastModel.meshes
//      modelInstance.
      if (modelInstance != null)
         LibGDXTools.toLibGDX(new Pose3D(box3D.getPose()), tempTransform, modelInstance.transform);
      if (buildMeshAndCreateModelInstance != null)
      {
         buildMeshAndCreateModelInstance.run();
         buildMeshAndCreateModelInstance = null;
      }
   }

   public void setColor(Color color)
   {
      this.color.set(color);
   }

   public void clear()
   {
      generateMesh(new Box3D());
   }

   public void generateMeshAsync(Box3D box)
   {
      executorService.clearQueueAndExecute(() -> generateMesh(box));
   }

   private synchronized void generateMesh(Box3D box)
   {
      box.getVertices(vertices);

      buildMeshAndCreateModelInstance = () ->
      {
         if (lastModel != null)
            lastModel.dispose();

         lastModel = RDXModelBuilder.buildModel(meshBuilder ->
         {
            double lineWidth = 0.03;
            meshBuilder.addMultiLineBox(vertices, lineWidth, color);
         }, "box");
         modelInstance = new ModelInstance(lastModel); // TODO: Clean up garbage and look into reusing the Model
      };
   }

   public void dispose()
   {
      executorService.destroy();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      // sync over current and add
      if (modelInstance != null)
      {
         modelInstance.getRenderables(renderables, pool);
      }

      if (pose3DGizmo != null)
      {
         pose3DGizmo.getRenderables(renderables, pool);
      }
   }

   public Box3D getBox3D()
   {
      return box3D;
   }

   public Vector3DReadOnly getDimensions()
   {
      return box3D.getSize();
   }

   public RigidBodyTransform getTransformToWorld()
   {
      return pose3DGizmo.getTransformToWorld();
   }

   public RDXPose3DGizmo getPose3DGizmo()
   {
      return pose3DGizmo;
   }
}
