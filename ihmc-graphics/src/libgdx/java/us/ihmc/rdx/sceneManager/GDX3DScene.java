package us.ihmc.rdx.sceneManager;

import com.badlogic.gdx.graphics.Camera;
import com.badlogic.gdx.graphics.g3d.Environment;
import com.badlogic.gdx.graphics.g3d.ModelBatch;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.DirectionalLightsAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.PointLightsAttribute;
import com.badlogic.gdx.graphics.g3d.environment.DirectionalLight;
import com.badlogic.gdx.graphics.g3d.environment.PointLight;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.rdx.lighting.GDXDirectionalLight;
import us.ihmc.rdx.lighting.GDXPointLight;
import us.ihmc.rdx.lighting.GDXShadowManager;
import us.ihmc.rdx.simulation.DepthSensorShaderProvider;
import us.ihmc.rdx.tools.GDXModelBuilder;
import us.ihmc.rdx.tools.GDXTools;
import us.ihmc.rdx.ui.GDXImGuiBasedUI;

import java.util.*;

public class GDX3DScene
{
   private final HashSet<ModelInstance> modelInstances = new HashSet<>();
   private final Set<GDXRenderableAdapter> renderables = new HashSet<>();

   private TreeSet<GDXSceneLevel> sceneLevelsToRender;
   private float ambientLight = 0.4f;
   private boolean shadowsEnabled = false;
   private GDXShadowManager shadowManager;
   private ModelBatch shadowsDisabledModelBatch;
   private Environment shadowsDisabledEnvironment;
   private final PointLightsAttribute shadowsDisabledPointLights = new PointLightsAttribute();
   private final DirectionalLightsAttribute shadowsDisabledDirectionalLights = new DirectionalLightsAttribute();

   public void create()
   {
      create(GDXSceneLevel.MODEL, GDXSceneLevel.VIRTUAL);
   }

   public void create(GDXSceneLevel... sceneLevelsToRender)
   {
      this.sceneLevelsToRender = new TreeSet<>();
      Collections.addAll(this.sceneLevelsToRender, sceneLevelsToRender);

      Pair<String, String> shaderStrings = GDXTools.loadCombinedShader(getClass().getName().replace(".", "/") + ".glsl");
      String vertexShader = shaderStrings.getLeft();
      String fragmentShader = shaderStrings.getRight();
      shadowsDisabledModelBatch = new ModelBatch(null, new DepthSensorShaderProvider(vertexShader, fragmentShader), null);
      shadowsDisabledEnvironment = new Environment();
      shadowsDisabledEnvironment.set(ColorAttribute.createAmbientLight(ambientLight, ambientLight, ambientLight, 1.0f));
      shadowsDisabledEnvironment.set(shadowsDisabledPointLights);
      shadowsDisabledEnvironment.set(shadowsDisabledDirectionalLights);

      shadowManager = new GDXShadowManager(GDXImGuiBasedUI.ANTI_ALIASING, ambientLight);
   }

   public void preRender(Camera camera)
   {
      if (shadowsEnabled)
      {
         shadowManager.preRender(camera);
      }
      else
      {
         shadowsDisabledModelBatch.begin(camera);
      }
   }

   public void render()
   {
      if (shadowsEnabled)
      {
         renderInternal(shadowManager.getShadowSceneBatch(), GDXSceneLevel.MODEL.SINGLETON_SET);
      }
      else
      {
         renderInternal(shadowsDisabledModelBatch, sceneLevelsToRender);
      }
   }

   public void render(GDXSceneLevel exclusiveSceneLevel)
   {
      renderInternal(shadowsDisabledModelBatch, exclusiveSceneLevel.SINGLETON_SET);
   }

   public void render(Set<GDXSceneLevel> sceneLevels)
   {
      renderInternal(shadowsDisabledModelBatch, sceneLevels);
   }

   // For testing shadows in particular
   public void renderShadowMap(Camera camera, int x, int y)
   {
      if (shadowsEnabled)
      {
         for (GDXRenderableAdapter renderable : renderables)
         {
            renderable.setSceneLevelsToRender(GDXSceneLevel.MODEL.SINGLETON_SET);
         }

         shadowManager.renderShadows(camera, renderables, x, y);
      }
   }

   // For simulated sensors in particular
   public void renderExternalBatch(ModelBatch batch, Set<GDXSceneLevel> sceneLevel)
   {
      renderInternal(batch, sceneLevel);
   }

   // For VR in particular
   public void renderToCamera(Camera camera)
   {
      if (shadowsEnabled)
      {
         shadowManager.preRender(camera);
         renderInternal(shadowManager.getShadowSceneBatch(), GDXSceneLevel.MODEL.SINGLETON_SET);
      }
      else
      {
         shadowsDisabledModelBatch.begin(camera);
         renderInternal(shadowsDisabledModelBatch, sceneLevelsToRender);
      }
      postRender(camera, GDXSceneLevel.VIRTUAL.SINGLETON_SET);
   }

   private void renderInternal(ModelBatch modelBatch, Set<GDXSceneLevel> sceneLevelsToRender)
   {
      // All rendering except modelBatch.begin() and end()
      // Avoid rendering things twice
      for (GDXRenderableAdapter renderable : renderables)
      {
         renderable.setSceneLevelsToRender(sceneLevelsToRender);

         if (shadowsEnabled)
            modelBatch.render(renderable);
         else
            modelBatch.render(renderable, shadowsDisabledEnvironment);
      }
   }

   public void postRender(Camera camera, Set<GDXSceneLevel> sceneLevels)
   {
      if (shadowsEnabled)
      {
         shadowManager.postRender();
      }
      else
      {
         shadowsDisabledModelBatch.end();
      }

      if (shadowsEnabled && sceneLevels.contains(GDXSceneLevel.VIRTUAL))
      {
         // Render all virtual objects using the primary model batch
         // FIXME: This has a problem where the virtual renderables aren't going to be occluded correctly.
         shadowsDisabledModelBatch.begin(camera);
         for (GDXRenderableAdapter renderable : renderables)
         {
            renderable.setSceneLevelsToRender(GDXSceneLevel.VIRTUAL.SINGLETON_SET);
            shadowsDisabledModelBatch.render(renderable);
         }
         shadowsDisabledModelBatch.end();
      }
   }

   public void dispose()
   {
      for (ModelInstance modelInstance : modelInstances)
      {
         ExceptionTools.handle(modelInstance.model::dispose, DefaultExceptionHandler.PRINT_MESSAGE);
      }

      shadowManager.dispose();
      shadowsDisabledModelBatch.dispose();
   }

   public GDXRenderableAdapter addModelInstance(ModelInstance modelInstance)
   {
      return addModelInstance(modelInstance, GDXSceneLevel.MODEL);
   }

   public GDXRenderableAdapter addModelInstance(ModelInstance modelInstance, GDXSceneLevel sceneLevel)
   {
      modelInstances.add(modelInstance);
      return addRenderableProvider(modelInstance, sceneLevel);
   }

   public GDXRenderableAdapter addCoordinateFrame(double size)
   {
      return addModelInstance(GDXModelBuilder.createCoordinateFrameInstance(size), GDXSceneLevel.VIRTUAL);
   }

   public GDXRenderableAdapter addRenderableProvider(RenderableProvider renderableProvider)
   {
      return addRenderableProvider(renderableProvider, GDXSceneLevel.MODEL);
   }

   public GDXRenderableAdapter addRenderableProvider(RenderableProvider renderableProvider, GDXSceneLevel sceneLevel)
   {
      GDXRenderableAdapter renderableAdapter = new GDXRenderableAdapter(renderableProvider, sceneLevel);
      renderables.add(renderableAdapter);
      return renderableAdapter;
   }

   public GDXRenderableAdapter addRenderableProvider(GDXRenderableProvider renderableProvider)
   {
      GDXRenderableAdapter renderableAdapter = new GDXRenderableAdapter(renderableProvider);
      renderables.add(renderableAdapter);
      return renderableAdapter;
   }

   public void addRenderableAdapter(GDXRenderableAdapter renderableAdapter)
   {
      renderables.add(renderableAdapter);
   }

   public void removeRenderableAdapter(GDXRenderableAdapter renderableAdapter)
   {
      renderables.remove(renderableAdapter);
   }

   public void addDefaultLighting()
   {
      setAmbientLight(0.914f);
      GDXPointLight pointLight = new GDXPointLight();
      pointLight.getPosition().set(10.0, 10.0, 10.0);
      addPointLight(pointLight);
      pointLight = new GDXPointLight();
      pointLight.getPosition().set(10.0, -10.0, 10.0);
      addPointLight(pointLight);
      pointLight = new GDXPointLight();
      pointLight.getPosition().set(-10.0, 10.0, 10.0);
      addPointLight(pointLight);
      pointLight = new GDXPointLight();
      pointLight.getPosition().set(-10.0, -10.0, 10.0);
      addPointLight(pointLight);
   }

   public void clearLights()
   {
      shadowsDisabledPointLights.lights.clear();
      shadowsDisabledDirectionalLights.lights.clear();
      shadowManager.getPointLights().clear();
      shadowManager.getDirectionalLights().clear();
   }

   public void addPointLight(GDXPointLight pointLight)
   {
      PointLight pointLightAttribute = GDX3DSceneTools.createPointLight(pointLight.getPosition().getX32(),
                                                                        pointLight.getPosition().getY32(),
                                                                        pointLight.getPosition().getZ32());
      pointLight.setAttribute(pointLightAttribute);
      shadowsDisabledPointLights.lights.add(pointLightAttribute);
      shadowManager.getPointLights().add(pointLight);
   }

   public void addDirectionalLight(GDXDirectionalLight directionalLight)
   {
      DirectionalLight directionalLightAttribute = GDX3DSceneTools.createDirectionalLight(directionalLight.getDirection().getX32(),
                                                                                          directionalLight.getDirection().getY32(),
                                                                                          directionalLight.getDirection().getZ32());
      directionalLight.setAttribute(directionalLightAttribute);
      shadowsDisabledDirectionalLights.lights.add(directionalLightAttribute);
      shadowManager.getDirectionalLights().add(directionalLight);
   }

   public void removePointLight(GDXPointLight pointLight)
   {
      shadowsDisabledPointLights.lights.removeValue(pointLight.getAttribute(), true);
      shadowManager.getPointLights().remove(pointLight);
   }

   public void removeDirectionalLight(GDXDirectionalLight directionalLight)
   {
      shadowsDisabledDirectionalLights.lights.removeValue(directionalLight.getAttribute(), true);
      shadowManager.getDirectionalLights().remove(directionalLight);
   }

   public void setAmbientLight(float ambientLight)
   {
      this.ambientLight = ambientLight;
      shadowsDisabledEnvironment.set(ColorAttribute.createAmbientLight(ambientLight, ambientLight, ambientLight, 1.0f));
      shadowManager.setAmbientLight(ambientLight);
   }

   public void setShadowsEnabled(boolean shadowsEnabled)
   {
      this.shadowsEnabled = shadowsEnabled;
   }

   public float getAmbientLight()
   {
      return ambientLight;
   }

   public GDXShadowManager getShadowManager()
   {
      return shadowManager;
   }

   public TreeSet<GDXSceneLevel> getSceneLevelsToRender()
   {
      return sceneLevelsToRender;
   }
}
