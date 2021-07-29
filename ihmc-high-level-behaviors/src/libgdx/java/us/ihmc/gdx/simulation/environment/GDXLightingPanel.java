package us.ihmc.gdx.simulation.environment;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.math.Quaternion;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImFloat;
import us.ihmc.gdx.lighting.GDXDirectionalLight;
import us.ihmc.gdx.lighting.GDXLight;
import us.ihmc.gdx.lighting.GDXPointLight;
import us.ihmc.gdx.lighting.GDXShadowManager;
import us.ihmc.gdx.sceneManager.GDX3DSceneManager;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.gizmo.GDXPose3DGizmo;

import java.util.HashMap;
import java.util.Map;

public class GDXLightingPanel implements RenderableProvider
{
   private static final String WINDOW_NAME = "Lighting";

   private static Model pointModel = null;
   private static Model directionalModel = null;

   private final GDX3DSceneManager sceneManager;
   private final GDXShadowManager shadowManager;

   private final HashMap<ModelInstance, GDXLight> lights = new HashMap<>();
   private final HashMap<ModelInstance, GDXPose3DGizmo> gizmos = new HashMap<>();

   private final ImFloat ambientLight = new ImFloat(0.4f);

   public GDXLightingPanel(GDX3DSceneManager manager)
   {
      this.sceneManager = manager;
      this.shadowManager = manager.getShadowManager();
   }

   private static Model getOrCreatePointModel()
   {
      if (pointModel == null)
      {
         pointModel = GDXModelPrimitives.buildModel(meshBuilder -> meshBuilder.addSphere(0.1f, Color.YELLOW), "pointModel");
      }

      return pointModel;
   }

   private static Model getOrCreateDirectionalModel()
   {
      if (directionalModel == null)
      {
         directionalModel = GDXModelPrimitives.buildModel(meshBuilder -> meshBuilder.addBox(0.2f, 0.2f, 0.05f, Color.YELLOW), "directionalModel");
      }

      return directionalModel;
   }

   private void createDirectionalLight() {
      ModelInstance lightModel = new ModelInstance(getOrCreateDirectionalModel());
      lightModel.transform.translate(0, 0, 1);

      Vector3 translation = new Vector3();
      Quaternion rotation = new Quaternion();
      lightModel.transform.getTranslation(translation);
      lightModel.transform.getRotation(rotation);
      GDXLight light = new GDXDirectionalLight(translation, new Vector3(rotation.x, rotation.y, rotation.z));

      shadowManager.addLight(light);
      shadowManager.update();

      lights.put(lightModel, light);

      GDXPose3DGizmo gizmo = new GDXPose3DGizmo();
      gizmo.create(sceneManager.getCamera3D());
      gizmo.getTransform().getTranslation().set(0, 0, 1);

      gizmos.put(lightModel, gizmo);
   }

   public void update() {
      for (Map.Entry<ModelInstance, GDXPose3DGizmo> entry : gizmos.entrySet()) {
         GDXTools.toGDX(entry.getValue().getTransform(), entry.getKey().transform);
      }
   }

   private void createPointLight() {
      ModelInstance lightModel = new ModelInstance(getOrCreateDirectionalModel());
      lightModel.transform.translate(0, 0, 1);

      Vector3 translation = new Vector3();
      lightModel.transform.getTranslation(translation);
      GDXLight light = new GDXPointLight(translation);

      shadowManager.addLight(light);
      shadowManager.update();

      lights.put(lightModel, light);
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {

      for (Map.Entry<ModelInstance, GDXPose3DGizmo> entry : gizmos.entrySet()) {
         entry.getKey().getRenderables(renderables, pool);
         entry.getValue().getRenderables(renderables, pool);
      }
   }

   public String getWindowName()
   {
      return WINDOW_NAME;
   }

   public void renderImGuiWidgets()
   {
      if (ImGui.sliderFloat("Ambient light", ambientLight.getData(), 0, 1)) {
         shadowManager.setAmbientLight(ambientLight.get());
      }
   }
}
