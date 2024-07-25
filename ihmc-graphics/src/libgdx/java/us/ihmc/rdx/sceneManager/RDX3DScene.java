package us.ihmc.rdx.sceneManager;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.assets.loaders.resolvers.ClasspathFileHandleResolver;
import com.badlogic.gdx.assets.loaders.resolvers.InternalFileHandleResolver;
import com.badlogic.gdx.graphics.Camera;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Cubemap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.Attribute;
import com.badlogic.gdx.graphics.g3d.Environment;
import com.badlogic.gdx.graphics.g3d.ModelBatch;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.DirectionalLightsAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.PointLightsAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.SpotLightsAttribute;
import com.badlogic.gdx.graphics.g3d.environment.PointLight;
import com.badlogic.gdx.graphics.g3d.environment.SpotLight;
import com.badlogic.gdx.graphics.g3d.shaders.DepthShader;
import com.badlogic.gdx.graphics.g3d.utils.DepthShaderProvider;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import net.mgsx.gltf.scene3d.attributes.PBRCubemapAttribute;
import net.mgsx.gltf.scene3d.attributes.PBRFloatAttribute;
import net.mgsx.gltf.scene3d.attributes.PBRMatrixAttribute;
import net.mgsx.gltf.scene3d.attributes.PBRTextureAttribute;
import net.mgsx.gltf.scene3d.lights.DirectionalLightEx;
import net.mgsx.gltf.scene3d.lights.PointLightEx;
import net.mgsx.gltf.scene3d.lights.SpotLightEx;
import net.mgsx.gltf.scene3d.scene.CascadeShadowMap;
import net.mgsx.gltf.scene3d.scene.MirrorSource;
import net.mgsx.gltf.scene3d.scene.SceneManager;
import net.mgsx.gltf.scene3d.scene.SceneRenderableSorter;
import net.mgsx.gltf.scene3d.scene.SceneSkybox;
import net.mgsx.gltf.scene3d.scene.TransmissionSource;
import net.mgsx.gltf.scene3d.scene.Updatable;
import net.mgsx.gltf.scene3d.shaders.PBRCommon;
import net.mgsx.gltf.scene3d.shaders.PBRShaderConfig;
import net.mgsx.gltf.scene3d.shaders.PBRShaderProvider;
import net.mgsx.gltf.scene3d.utils.EnvironmentCache;
import net.mgsx.gltf.scene3d.utils.EnvironmentUtil;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.rdx.lighting.RDXDirectionalLight;
import us.ihmc.rdx.lighting.RDXPointLight;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.vr.RDXVREye;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.*;

public class RDX3DScene
{
   public static final float DEFAULT_AMBIENT_LIGHT_INTENSITY = 1.0f;
   public static final float DEFAULT_POINT_LIGHT_INTENSITY = 660.0f;
   public static final float DEFAULT_DIRECTIONAL_LIGHT_INTENSITY = 2.0f;

   private final HashSet<ModelInstance> modelInstances = new HashSet<>();
   private final Set<RDXRenderableAdapter> renderables = new HashSet<>();
   private final Map<Object, RDXRenderableAdapter> renderableOwnerKeyMap = new HashMap<>();

   private TreeSet<RDXSceneLevel> sceneLevelsToRender;
   private ColorAttribute ambientLight;
   private float pointLightIntensity = DEFAULT_POINT_LIGHT_INTENSITY;
   private float directionalLightIntensity = DEFAULT_DIRECTIONAL_LIGHT_INTENSITY;
   private Cubemap diffuseCubemap;
   private Cubemap environmentCubemap;
   private Cubemap specularCubemap;
   private Texture brdfLUT;
   private ModelBatch colorModelBatch;
   private ModelBatch depthModelBatch;
   private Environment environment;
   private final PointLightsAttribute pointLights = new PointLightsAttribute();
   private final DirectionalLightsAttribute directionalLights = new DirectionalLightsAttribute();
   private final SpotLightsAttribute spotLights = new SpotLightsAttribute();
   private SceneSkybox sceneSkybox;
   private TransmissionSource transmissionSource;
   private MirrorSource mirrorSource;
   private CascadeShadowMap cascadeShadowMap;
   protected final EnvironmentCache computedEnvironement = new EnvironmentCache();

   private PointLightsAttribute computedPointLights = new PointLightsAttribute();
   private SpotLightsAttribute computedSpotLights = new SpotLightsAttribute();

   public void create()
   {
      create(RDXSceneLevel.MODEL, RDXSceneLevel.VIRTUAL);
   }

   public void create(RDXSceneLevel... sceneLevelsToRender)
   {
      this.sceneLevelsToRender = new TreeSet<>();
      Collections.addAll(this.sceneLevelsToRender, sceneLevelsToRender);

      int maxBones = 0; // We aren't using bones
      PBRShaderConfig pbrColorShaderConfig = new PBRShaderConfig();
      pbrColorShaderConfig.numBones = maxBones;
      pbrColorShaderConfig.numPointLights = 10; // Increase max
      // pbrColorShaderConfig.numSpotLights = X  <-- Use this to enable spot lights

      DepthShader.Config depthShaderConfig = new DepthShader.Config();
      depthShaderConfig.numBones = maxBones;

      PBRShaderProvider pbrColorShader = PBRShaderProvider.createDefault(pbrColorShaderConfig);
      DepthShaderProvider pbrDepthShader = PBRShaderProvider.createDefaultDepth(depthShaderConfig);
      SceneRenderableSorter sceneRenderableSorter = new SceneRenderableSorter();

      colorModelBatch = new ModelBatch(pbrColorShader, sceneRenderableSorter);
      depthModelBatch = new ModelBatch(pbrDepthShader);

      environment = new Environment();
      ambientLight = ColorAttribute.createAmbientLight(DEFAULT_AMBIENT_LIGHT_INTENSITY,
                                                       DEFAULT_AMBIENT_LIGHT_INTENSITY,
                                                       DEFAULT_AMBIENT_LIGHT_INTENSITY,
                                                       1.0f);
      environment.set(ambientLight);
      environment.set(pointLights);
      environment.set(directionalLights);
      environment.set(spotLights);

      diffuseCubemap = EnvironmentUtil.createCubemap(new ClasspathFileHandleResolver(),
                                                     "cubeMaps/demo1/diffuse/diffuse_", ".jpg",
                                                     EnvironmentUtil.FACE_NAMES_NEG_POS);
      environmentCubemap = EnvironmentUtil.createCubemap(new ClasspathFileHandleResolver(),
                                                         "cubeMaps/demo1/environment/environment_", ".jpg",
                                                         EnvironmentUtil.FACE_NAMES_NEG_POS);
      int lods = 10;
      specularCubemap = EnvironmentUtil.createCubemap(new ClasspathFileHandleResolver(),
                                                      "cubeMaps/demo1/specular/specular_", "_", ".jpg",
                                                      lods,
                                                      EnvironmentUtil.FACE_NAMES_NEG_POS);
      brdfLUT = new Texture(Gdx.files.classpath("brdfLUT.png"));

      environment.set(PBRCubemapAttribute.createDiffuseEnv(diffuseCubemap));
      environment.set(PBRCubemapAttribute.createSpecularEnv(specularCubemap));
      environment.set(new PBRTextureAttribute(PBRTextureAttribute.BRDFLUTTexture, brdfLUT));
      environment.set(new PBRFloatAttribute(PBRFloatAttribute.ShadowBias, 0f));

      sceneSkybox = new SceneSkybox(environmentCubemap);
      Matrix4 transform = new Matrix4();
      transform.rotate(1.0f, 0.0f, 0.0f, -90.0f);
      PBRMatrixAttribute envRotation = PBRMatrixAttribute.createEnvRotation(transform);
      environment.set(envRotation);
      sceneSkybox.setRotation(envRotation.matrix);
   }

   public void preRender(Camera camera)
   {
      float deltaTime = Gdx.graphics.getDeltaTime();

      computedEnvironement.setCache(environment);
      computedPointLights.lights.clear();
      computedSpotLights.lights.clear();
      if (environment != null)
      {
         for (Attribute attribute : environment)
         {
            if (attribute instanceof PointLightsAttribute pointLightsAttribute)
            {
               computedPointLights.lights.addAll(pointLightsAttribute.lights);
               computedEnvironement.replaceCache(computedPointLights);
            }
            else if (attribute instanceof SpotLightsAttribute spotLightsAttribute)
            {
               computedSpotLights.lights.addAll(spotLightsAttribute.lights);
               computedEnvironement.replaceCache(computedSpotLights);
            }
            else
            {
               computedEnvironement.set(attribute);
            }
         }
      }
      cullLights(camera);

      for (RDXRenderableAdapter renderable : renderables)
      {
         if (renderable instanceof Updatable updatable)
         {
            updatable.update(camera, deltaTime);
         }
      }

      sceneSkybox.update(camera, deltaTime);

      PBRCommon.enableSeamlessCubemaps();

      colorModelBatch.begin(camera);
   }

   protected void cullLights(Camera camera)
   {
      PointLightsAttribute pointLightsAttribute = environment.get(PointLightsAttribute.class, PointLightsAttribute.Type);
      if (pointLightsAttribute != null)
      {
         for (PointLight light : pointLightsAttribute.lights)
         {
            if (light instanceof PointLightEx pointLightEx)
            {
               if (pointLightEx.range != null && !camera.frustum.sphereInFrustum(pointLightEx.position, pointLightEx.range))
               {
                  computedPointLights.lights.removeValue(pointLightEx, true);
               }
            }
         }
      }
      SpotLightsAttribute spotLightsAttribute = environment.get(SpotLightsAttribute.class, SpotLightsAttribute.Type);
      if (spotLightsAttribute != null)
      {
         for (SpotLight light : spotLightsAttribute.lights)
         {
            if (light instanceof SpotLightEx spotLightEx)
            {
               if (spotLightEx.range != null && !camera.frustum.sphereInFrustum(spotLightEx.position, spotLightEx.range))
               {
                  computedSpotLights.lights.removeValue(spotLightEx, true);
               }
            }
         }
      }
   }

   public void preRenderDepth(Camera camera)
   {
      depthModelBatch.begin(camera);
   }

   public void render()
   {
      renderInternal(colorModelBatch, sceneLevelsToRender);
   }

   public void render(RDXSceneLevel exclusiveSceneLevel)
   {
      renderInternal(colorModelBatch, exclusiveSceneLevel.SINGLETON_SET);
   }

   public void render(Set<RDXSceneLevel> sceneLevels)
   {
      renderInternal(colorModelBatch, sceneLevels);
   }

   // For simulated sensors in particular
   public void renderExternalBatch(ModelBatch batch, Set<RDXSceneLevel> sceneLevel)
   {
      renderInternal(batch, sceneLevel);
   }

   public void renderDepth(Set<RDXSceneLevel> sceneLevel)
   {
      renderInternal(depthModelBatch, sceneLevel);
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

      colorModelBatch.begin(camera);
      renderInternal(colorModelBatch, sceneLevelsToRender);

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

      postRender();
   }

   private void renderInternal(ModelBatch modelBatch, Set<RDXSceneLevel> sceneLevelsToRender)
   {
      // All rendering except modelBatch.begin() and end()
      // Avoid rendering things twice
      for (RDXRenderableAdapter renderable : renderables)
      {
         renderable.setSceneLevelsToRender(sceneLevelsToRender);

         modelBatch.render(renderable, computedEnvironement);
      }
      modelBatch.render(sceneSkybox);
   }

   public void postRender()
   {
      colorModelBatch.end();
   }

   public void postRenderDepth()
   {
      depthModelBatch.end();
   }

   public void dispose()
   {
      for (ModelInstance modelInstance : modelInstances)
      {
         ExceptionTools.handle(modelInstance.model::dispose, DefaultExceptionHandler.PRINT_MESSAGE);
      }

      colorModelBatch.dispose();
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
      addRenderableInternal(renderableAdapter);
      return renderableAdapter;
   }

   public RDXRenderableAdapter addRenderableProvider(RDXRenderableProvider renderableProvider)
   {
      RDXRenderableAdapter renderableAdapter = new RDXRenderableAdapter(renderableProvider);
      addRenderableInternal(renderableAdapter);
      return renderableAdapter;
   }

   public void addRenderableProvider(Object ownerKey, RDXRenderableProvider renderableProvider)
   {
      RDXRenderableAdapter renderableAdapter = new RDXRenderableAdapter(renderableProvider);
      renderableOwnerKeyMap.put(ownerKey, renderableAdapter);
      addRenderableInternal(renderableAdapter);
   }

   public void addRenderableProvider(Object ownerKey, RenderableProvider renderableProvider)
   {
      addRenderableProvider(ownerKey, renderableProvider, RDXSceneLevel.MODEL);
   }

   public void addRenderableProvider(Object ownerKey, RenderableProvider renderableProvider, RDXSceneLevel sceneLevel)
   {
      RDXRenderableAdapter renderableAdapter = new RDXRenderableAdapter(renderableProvider, sceneLevel);
      renderableOwnerKeyMap.put(ownerKey, renderableAdapter);
      addRenderableInternal(renderableAdapter);
   }

   public void addRenderableAdapter(RDXRenderableAdapter renderableAdapter)
   {
      addRenderableInternal(renderableAdapter);
   }

   private void addRenderableInternal(RDXRenderableAdapter renderableAdapter)
   {
      // Used to debug ClassCastException thrown by PBRShader
      // System.err.println("Adding %s@%d".formatted(renderableAdapter.getClass().getName(), renderableAdapter.hashCode()));
      // new Throwable().printStackTrace();

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
      environment.add(new DirectionalLightEx().set(Color.WHITE, new Vector3(-1.0f, -4.0f, -2.0f), directionalLightIntensity));
      environment.add(new DirectionalLightEx().set(Color.WHITE, new Vector3(0.0f, 0.0f, 1.0f), directionalLightIntensity));

      Float range = null; // infinite range
      environment.add(new PointLightEx().set(Color.WHITE, new Vector3(10.0f, 10.0f, 10.0f), pointLightIntensity, range));
      environment.add(new PointLightEx().set(Color.WHITE, new Vector3(10.0f, -10.0f, 10.0f), pointLightIntensity, range));
      environment.add(new PointLightEx().set(Color.WHITE, new Vector3(-10.0f, 10.0f, 10.0f), pointLightIntensity, range));
      environment.add(new PointLightEx().set(Color.WHITE, new Vector3(-10.0f, -10.0f, 10.0f), pointLightIntensity, range));
   }

   public void clearLights()
   {
      pointLights.lights.clear();
      directionalLights.lights.clear();
   }

   public void addPointLight(RDXPointLight pointLight)
   {
      environment.add(pointLight.getPointLightEx());
   }

   public void addDirectionalLight(RDXDirectionalLight directionalLight)
   {
      environment.add(directionalLight.getDirectionalLightEx());
   }

   public void removePointLight(RDXPointLight pointLight)
   {
      pointLights.lights.removeValue(pointLight.getPointLightEx(), true);
   }

   public void removeDirectionalLight(RDXDirectionalLight directionalLight)
   {
      directionalLights.lights.removeValue(directionalLight.getDirectionalLightEx(), true);
   }

   public void setAmbientLightIntensity(float ambientLightIntensity)
   {
      ambientLight.color.r = ambientLightIntensity;
      ambientLight.color.g = ambientLightIntensity;
      ambientLight.color.b = ambientLightIntensity;
   }

   public float getAmbientLightIntensity()
   {
      return ambientLight.color.r;
   }

   public float getPointLightIntensity()
   {
      return pointLightIntensity;
   }

   public void setPointLightIntensity(float pointLightIntensity)
   {
      this.pointLightIntensity = pointLightIntensity;

      for (int i = 0; i < pointLights.lights.size; i++)
      {
         pointLights.lights.get(i).intensity = pointLightIntensity;
      }
   }

   public float getDirectionalLightIntensity()
   {
      return directionalLightIntensity;
   }

   public void setDirectionalLightIntensity(float directionalLightIntensity)
   {
      this.directionalLightIntensity = directionalLightIntensity;

      for (int i = 0; i < directionalLights.lights.size; i++)
      {
         if (directionalLights.lights.get(i) instanceof DirectionalLightEx directionalLightEx)
         {
            directionalLightEx.intensity = directionalLightIntensity;
         }
      }
   }

   public Environment getEnvironment()
   {
      return environment;
   }

   public TreeSet<RDXSceneLevel> getSceneLevelsToRender()
   {
      return sceneLevelsToRender;
   }
}