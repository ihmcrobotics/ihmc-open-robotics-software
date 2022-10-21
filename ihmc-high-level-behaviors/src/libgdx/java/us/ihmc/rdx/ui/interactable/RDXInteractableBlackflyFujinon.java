package us.ihmc.rdx.ui.interactable;

import com.badlogic.gdx.graphics.PerspectiveCamera;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.model.data.ModelData;
import com.badlogic.gdx.math.Frustum;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.RDXModelLoader;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.affordances.RDXInteractableFrameModel;
import us.ihmc.rdx.ui.gizmo.CylinderRayIntersection;
import us.ihmc.rdx.visualizers.RDXFrustumVisualizer;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

public class RDXInteractableBlackflyFujinon
{
   private final RDXInteractableFrameModel interactableFrameModel = new RDXInteractableFrameModel();
   private final CylinderRayIntersection cylinderIntersection = new CylinderRayIntersection();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean renderFrustum = new ImBoolean(false);
   private final ImBoolean renderARImage = new ImBoolean(false);
   private final double frustimNear = 0.05;
   private final ImFloat frustumFar = new ImFloat(5.0f);
   private RDXFrustumVisualizer frustumVisualizer;
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final Matrix4 tempMatrix4 = new Matrix4();
   private PerspectiveCamera camera;

   public RDXInteractableBlackflyFujinon(RDX3DPanel panel3D)
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      ReferenceFrame referenceFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(), transform);
      create(panel3D, referenceFrame, transform);
   }

   public RDXInteractableBlackflyFujinon(RDX3DPanel panel3D, ReferenceFrame referenceFrameToRepresent, RigidBodyTransform transformToParentToModify)
   {
      create(panel3D, referenceFrameToRepresent, transformToParentToModify);
   }

   private void create(RDX3DPanel panel3D, ReferenceFrame referenceFrameToRepresent, RigidBodyTransform transformToParentToModify)
   {
      ModelData sensorModel = RDXModelLoader.loadModelData("environmentObjects/blackflyFujinon/BlackflyFujinon.g3dj");
      interactableFrameModel.create(referenceFrameToRepresent, transformToParentToModify, panel3D, sensorModel, this::calculateClosestCollision);

      interactableFrameModel.setExtendedContextMenu(this::renderImGuiContextMenu);

      float verticalFOV = 100.0f;
      int imageWidth = 1024;
      int imageHeight = 1024;
      camera = new PerspectiveCamera(verticalFOV, imageWidth, imageHeight);
      camera.near = (float) frustimNear;
      camera.far = frustumFar.get();
      camera.position.setZero();
      camera.up.set(0.0f, 0.0f, 1.0f);
      camera.direction.set(1.0f, 0.0f, 0.0f);
      frustumVisualizer = new RDXFrustumVisualizer();

      panel3D.getScene().addRenderableProvider(this::getVirtualRenderables, RDXSceneLevel.VIRTUAL);
   }

   public void update()
   {
      if (renderFrustum.get())
      {
         interactableFrameModel.getReferenceFrame().getTransformToDesiredFrame(tempTransform, ReferenceFrame.getWorldFrame());
         LibGDXTools.toGDX(tempTransform, tempMatrix4);

         camera.near = (float) frustimNear;
         camera.far = frustumFar.get();
         camera.position.setZero();
         camera.up.set(0.0f, 0.0f, 1.0f);
         camera.direction.set(1.0f, 0.0f, 0.0f);
         camera.transform(tempMatrix4);
         camera.update(true);

         frustumVisualizer.generateMesh(camera.frustum);
         frustumVisualizer.update();
      }
   }

   private void renderImGuiContextMenu()
   {
      ImGui.checkbox(labels.get("Render frustum"), renderFrustum);
      ImGui.checkbox(labels.get("Render AR image"), renderARImage);
      ImGui.inputFloat(labels.get("AR image distance"), frustumFar);
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (renderFrustum.get())
      {
         frustumVisualizer.getRenderables(renderables, pool);
      }
   }

   private double calculateClosestCollision(Line3DReadOnly mousePickRay)
   {
      cylinderIntersection.update(0.08, 0.03, new Point3D(-0.03, 0.0, 0.0), Axis3D.X, interactableFrameModel.getReferenceFrame());
      return cylinderIntersection.intersect(mousePickRay);
   }

   public RDXInteractableFrameModel getInteractableFrameModel()
   {
      return interactableFrameModel;
   }

   public Frustum getFrustum()
   {
      return camera.frustum;
   }

   public ImBoolean getRenderARImage()
   {
      return renderARImage;
   }
}
