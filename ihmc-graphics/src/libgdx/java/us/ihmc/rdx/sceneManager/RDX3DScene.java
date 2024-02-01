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
import us.ihmc.rdx.lighting.RDXDirectionalLight;
import us.ihmc.rdx.lighting.RDXPointLight;
import us.ihmc.rdx.lighting.RDXShadowManager;
import us.ihmc.rdx.simulation.DepthSensorShaderProvider;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.vr.RDXVREye;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.*;

public class RDX3DScene
{
   private final HashSet<ModelInstance> modelInstances = new HashSet<>();
   private final Set<RDXRenderableAdapter> renderables = new HashSet<>();
   private final Map<Object, RDXRenderableAdapter> renderableOwnerKeyMap = new HashMap<>();

   private TreeSet<RDXSceneLevel> sceneLevelsToRender;
   private float ambientLight = 0.4f;
   private boolean shadowsEnabled = false;
   private RDXShadowManager shadowManager;
   private ModelBatch shadowsDisabledModelBatch;
   private Environment shadowsDisabledEnvironment;
   private final PointLightsAttribute shadowsDisabledPointLights = new PointLightsAttribute();
   private final DirectionalLightsAttribute shadowsDisabledDirectionalLights = new DirectionalLightsAttribute();

   public void create()
   {
      create(RDXSceneLevel.MODEL, RDXSceneLevel.VIRTUAL);
   }

   public void create(RDXSceneLevel... sceneLevelsToRender)
   {
      this.sceneLevelsToRender = new TreeSet<>();
      Collections.addAll(this.sceneLevelsToRender, sceneLevelsToRender);

      Pair<String, String> shaderStrings = LibGDXTools.loadCombinedShader(getClass().getName().replace(".", "/") + ".glsl");
      String vertexShader = shaderStrings.getLeft();
      String fragmentShader = shaderStrings.getRight();
      shadowsDisabledModelBatch = new ModelBatch(null, new DepthSensorShaderProvider(vertexShader, fragmentShader), null);
      shadowsDisabledEnvironment = new Environment();
      shadowsDisabledEnvironment.set(ColorAttribute.createAmbientLight(ambientLight, ambientLight, ambientLight, 1.0f));
      shadowsDisabledEnvironment.set(shadowsDisabledPointLights);
      shadowsDisabledEnvironment.set(shadowsDisabledDirectionalLights);

      shadowManager = new RDXShadowManager(RDXBaseUI.ANTI_ALIASING, ambientLight);
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
         renderInternal(shadowManager.getShadowSceneBatch(), RDXSceneLevel.MODEL.SINGLETON_SET);
      }
      else
      {
         renderInternal(shadowsDisabledModelBatch, sceneLevelsToRender);
      }
   }

   public void render(RDXSceneLevel exclusiveSceneLevel)
   {
      renderInternal(shadowsDisabledModelBatch, exclusiveSceneLevel.SINGLETON_SET);
   }

   public void render(Set<RDXSceneLevel> sceneLevels)
   {
      renderInternal(shadowsDisabledModelBatch, sceneLevels);
   }

   // For testing shadows in particular
   public void renderShadowMap(Camera camera, int x, int y)
   {
      if (shadowsEnabled)
      {
         for (RDXRenderableAdapter renderable : renderables)
         {
            renderable.setSceneLevelsToRender(RDXSceneLevel.MODEL.SINGLETON_SET);
         }

         shadowManager.renderShadows(camera, renderables, x, y);
      }
   }

   // For simulated sensors in particular
   public void renderExternalBatch(ModelBatch batch, Set<RDXSceneLevel> sceneLevel)
   {
      renderInternal(batch, sceneLevel);
   }

   // For VR in particular
   public void renderToCamera(Camera camera)
   {
      if (camera instanceof RDXVREye eye)
      {
         if (eye.getSide() == RobotSide.LEFT)
         {
            sceneLevelsToRender.add(RDXSceneLevel.VR_EYE_LEFT);
         }

         if (eye.getSide() == RobotSide.RIGHT)
         {
            sceneLevelsToRender.add(RDXSceneLevel.VR_EYE_RIGHT);
         }
      }

      if (shadowsEnabled)
      {
         shadowManager.preRender(camera);
         renderInternal(shadowManager.getShadowSceneBatch(), RDXSceneLevel.MODEL.SINGLETON_SET);
      }
      else
      {
         shadowsDisabledModelBatch.begin(camera);
         renderInternal(shadowsDisabledModelBatch, sceneLevelsToRender);
      }

      if (camera instanceof RDXVREye eye)
      {
         if (eye.getSide() == RobotSide.LEFT)
         {
            sceneLevelsToRender.remove(RDXSceneLevel.VR_EYE_LEFT);
         }

         if (eye.getSide() == RobotSide.RIGHT)
         {
            sceneLevelsToRender.remove(RDXSceneLevel.VR_EYE_RIGHT);
         }
      }

      postRender(camera, RDXSceneLevel.VIRTUAL.SINGLETON_SET);
   }

   private void renderInternal(ModelBatch modelBatch, Set<RDXSceneLevel> sceneLevelsToRender)
   {
      // All rendering except modelBatch.begin() and end()
      // Avoid rendering things twice
      for (RDXRenderableAdapter renderable : renderables)
      {
         renderable.setSceneLevelsToRender(sceneLevelsToRender);

         if (shadowsEnabled)
            modelBatch.render(renderable);
         else
            modelBatch.render(renderable, shadowsDisabledEnvironment);
      }
   }

   public void postRender(Camera camera, Set<RDXSceneLevel> sceneLevels)
   {
      if (shadowsEnabled)
      {
         shadowManager.postRender();
      }
      else
      {
         shadowsDisabledModelBatch.end();
      }

      if (shadowsEnabled && sceneLevels.contains(RDXSceneLevel.VIRTUAL))
      {
         // Render all virtual objects using the primary model batch
         // FIXME: This has a problem where the virtual renderables aren't going to be occluded correctly.
         shadowsDisabledModelBatch.begin(camera);
         for (RDXRenderableAdapter renderable : renderables)
         {
            renderable.setSceneLevelsToRender(RDXSceneLevel.VIRTUAL.SINGLETON_SET);
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

   public RDXRenderableAdapter addModelInstance(ModelInstance modelInstance)
   {
      return addModelInstance(modelInstance, RDXSceneLevel.MODEL);
   }

   public RDXRenderableAdapter addModelInstance(ModelInstance modelInstance, RDXSceneLevel sceneLevel)
   {
      modelInstances.add(modelInstance);
      return addRenderableProvider(modelInstance, sceneLevel);
   }

   public RDXRenderableAdapter addCoordinateFrame(double size)
   {
      return addModelInstance(RDXModelBuilder.createCoordinateFrameInstance(size), RDXSceneLevel.VIRTUAL);
   }

   public RDXRenderableAdapter addRenderableProvider(RenderableProvider renderableProvider)
   {
      return addRenderableProvider(renderableProvider, RDXSceneLevel.MODEL);
   }

   public RDXRenderableAdapter addRenderableProvider(RenderableProvider renderableProvider, RDXSceneLevel sceneLevel)
   {
      RDXRenderableAdapter renderableAdapter = new RDXRenderableAdapter(renderableProvider, sceneLevel);
      renderables.add(renderableAdapter);
      return renderableAdapter;
   }

   public RDXRenderableAdapter addRenderableProvider(RDXRenderableProvider renderableProvider)
   {
      RDXRenderableAdapter renderableAdapter = new RDXRenderableAdapter(renderableProvider);
      renderables.add(renderableAdapter);
      return renderableAdapter;
   }

   public void addRenderableProvider(Object ownerKey, RDXRenderableProvider renderableProvider)
   {
      RDXRenderableAdapter renderableAdapter = new RDXRenderableAdapter(renderableProvider);
      renderableOwnerKeyMap.put(ownerKey, renderableAdapter);
      renderables.add(renderableAdapter);
   }

   public void addRenderableProvider(Object ownerKey, RenderableProvider renderableProvider)
   {
      addRenderableProvider(ownerKey, renderableProvider, RDXSceneLevel.MODEL);
   }

   public void addRenderableProvider(Object ownerKey, RenderableProvider renderableProvider, RDXSceneLevel sceneLevel)
   {
      RDXRenderableAdapter renderableAdapter = new RDXRenderableAdapter(renderableProvider, sceneLevel);
      renderableOwnerKeyMap.put(ownerKey, renderableAdapter);
      renderables.add(renderableAdapter);
   }

   public void addRenderableAdapter(RDXRenderableAdapter renderableAdapter)
   {
      renderables.add(renderableAdapter);
   }

   public void removeRenderable(Object ownerKey)
   {
      renderables.remove(renderableOwnerKeyMap.remove(ownerKey));
   }

   public void removeRenderableAdapter(RDXRenderableAdapter renderableAdapter)
   {
      renderables.remove(renderableAdapter);
   }

   public void addDefaultLighting()
   {
      setAmbientLight(0.914f);
      RDXPointLight pointLight = new RDXPointLight();
      pointLight.getPosition().set(10.0, 10.0, 10.0);
      addPointLight(pointLight);
      pointLight = new RDXPointLight();
      pointLight.getPosition().set(10.0, -10.0, 10.0);
      addPointLight(pointLight);
      pointLight = new RDXPointLight();
      pointLight.getPosition().set(-10.0, 10.0, 10.0);
      addPointLight(pointLight);
      pointLight = new RDXPointLight();
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

   public void addPointLight(RDXPointLight pointLight)
   {
      PointLight pointLightAttribute = RDX3DSceneTools.createPointLight(pointLight.getPosition().getX32(),
                                                                        pointLight.getPosition().getY32(),
                                                                        pointLight.getPosition().getZ32());
      pointLight.setAttribute(pointLightAttribute);
      shadowsDisabledPointLights.lights.add(pointLightAttribute);
      shadowManager.getPointLights().add(pointLight);
   }

   public void addDirectionalLight(RDXDirectionalLight directionalLight)
   {
      DirectionalLight directionalLightAttribute = RDX3DSceneTools.createDirectionalLight(directionalLight.getDirection().getX32(),
                                                                                          directionalLight.getDirection().getY32(),
                                                                                          directionalLight.getDirection().getZ32());
      directionalLight.setAttribute(directionalLightAttribute);
      shadowsDisabledDirectionalLights.lights.add(directionalLightAttribute);
      shadowManager.getDirectionalLights().add(directionalLight);
   }

   public void removePointLight(RDXPointLight pointLight)
   {
      shadowsDisabledPointLights.lights.removeValue(pointLight.getAttribute(), true);
      shadowManager.getPointLights().remove(pointLight);
   }

   public void removeDirectionalLight(RDXDirectionalLight directionalLight)
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

   public RDXShadowManager getShadowManager()
   {
      return shadowManager;
   }

   public TreeSet<RDXSceneLevel> getSceneLevelsToRender()
   {
      return sceneLevelsToRender;
   }
}