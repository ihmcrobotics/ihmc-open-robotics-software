package us.ihmc.gdx;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.*;
import com.badlogic.gdx.graphics.g3d.ModelBatch;
import com.badlogic.gdx.graphics.g3d.shaders.DepthShader;
import com.badlogic.gdx.graphics.g3d.utils.DepthShaderProvider;
import com.badlogic.gdx.graphics.glutils.FloatFrameBuffer;
import com.badlogic.gdx.graphics.glutils.FrameBuffer;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.utils.ScreenUtils;
import com.badlogic.gdx.utils.viewport.ScreenViewport;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.gdx.sceneManager.GDX3DSceneManager;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

import java.util.Random;

/**
 * Lidar could be simulated with a viewportHeight of 1 and rotating the camera
 */
public class GDXDepthSensorSimulator
{
   private final Random random = new Random();
   private final float fieldOfViewY;
   private final float viewportWidth;
   private final float viewportHeight;
   private PerspectiveCamera camera;
   private int framebufferId;
   private DepthShaderProvider depthShaderProvider;

   private ModelBatch modelBatch;
   private int depthTextureId;
   private ScreenViewport viewport;
   private FrameBuffer frameBuffer;
   private FloatFrameBuffer floatframeBuffer;

   private RecyclingArrayList<Point3D32> points;
   private final Color tempGDXColor = new Color();
   private final Vector3 depthPoint = new Vector3();

   private final AffineTransform tempWorldTransform = new AffineTransform();
   private final PoseReferenceFrame cameraReferenceFrame = new PoseReferenceFrame("depthCameraFrame", ReferenceFrame.getWorldFrame());
   private final FramePoint3D tempFramePoint = new FramePoint3D();

   public GDXDepthSensorSimulator(float fieldOfViewY, float viewportWidth, float viewportHeight)
   {
      this.fieldOfViewY = fieldOfViewY;
      this.viewportWidth = viewportWidth;
      this.viewportHeight = viewportHeight;
   }

   public void create()
   {
      camera = new PerspectiveCamera(fieldOfViewY, viewportWidth, viewportHeight);
      camera.near = 0.01f;
      camera.far = 2000.0f;
      viewport = new ScreenViewport(camera);

      DepthShader.Config depthShaderConfig = new DepthShader.Config();
//      depthShaderConfig.numDirectionalLights = 0;
//      depthShaderConfig.numPointLights = 0;
//      depthShaderConfig.numSpotLights = 0;
//      depthShaderConfig.numBones = 0;
      depthShaderConfig.defaultCullFace = GL20.GL_BACK;
      depthShaderConfig.defaultAlphaTest = 1.0f;
//      depthShaderConfig.vertexShader = Gdx.files.classpath("depthsensor.vertex.glsl").readString();
//      depthShaderConfig.fragmentShader = Gdx.files.classpath("depthsensor.fragment.glsl").readString();
      //      config.depthBufferOnly = true;
      depthShaderProvider = new DepthShaderProvider(depthShaderConfig);

      modelBatch = new ModelBatch(depthShaderProvider);
//      modelBatch = new ModelBatch();

      boolean hasDepth = true;
      boolean hasStencil = true;
      frameBuffer = new FrameBuffer(Pixmap.Format.RGBA8888, (int) viewportWidth, (int) viewportHeight, hasDepth, hasStencil);
//      GLFrameBuffer.FloatFrameBufferBuilder depthBufferBuilder = new GLFrameBuffer.FloatFrameBufferBuilder((int) viewportWidth, (int) viewportHeight);
//      frameBuffer = new FloatFrameBuffer()FrameBuffer(depthBufferBuilder);
      floatframeBuffer = new FloatFrameBuffer((int) viewportWidth, (int) viewportHeight, true);

      points = new RecyclingArrayList<>(frameBuffer.getWidth() * frameBuffer.getHeight(), Point3D32::new);
   }

   public void render(GDX3DSceneManager gdx3DSceneManager)
   {
      camera.near = 0.01f;

      frameBuffer.begin();
      floatframeBuffer.begin();
      gdx3DSceneManager.glClearGray();

      //      viewport.getCamera().translate(-random.nextFloat(), -random.nextFloat(), -random.nextFloat());
//      viewport.getCamera().position

      viewport.update((int) viewportWidth, (int) viewportHeight);
      modelBatch.begin(camera);

      Gdx.gl.glViewport(0, 0, (int) viewportWidth, (int) viewportHeight);

      gdx3DSceneManager.renderRegisteredObjectsWithEnvironment(modelBatch, GDXSceneLevel.REAL_ENVIRONMENT);

      modelBatch.end();

      int width = frameBuffer.getWidth();
      int height = frameBuffer.getHeight();
//      frameBuffer.getDepthBufferHandle();
      Pixmap pixmap = ScreenUtils.getFrameBufferPixmap(0, 0, width, height);

//      ByteBuffer underlyingBuffer = frameBuffer.getColorBufferTexture().getTextureData().consumePixmap().getPixels();
//      Gdx.gl.glReadPixels(0, 0, width, height, GL_DEPTH_COMPONENT16, GL32.GL_HIGH_FLOAT, underlyingBuffer);

      frameBuffer.end();
      floatframeBuffer.end();

      points.clear();

//            GLOnlyTextureData textureData = (GLOnlyTextureData) frameBuffer.getColorBufferTexture().getTextureData();
//      Pixmap pixmap = textureData.consumePixmap();
      for (int y = 0; y < height; y++) // comes out flipped
      {
         for (int x = 0; x < width; x++)
         {
            int color = pixmap.getPixel(x, y);
            tempGDXColor.set(color);
            float depthReading = tempGDXColor.r;
            depthPoint.set((float) x, (3.0f * (float) height / 2.0f) - (float) y, depthReading);
            viewport.unproject(depthPoint);

//            if (depthPoint.z < camera.near)
            {
               tempFramePoint.setToZero(cameraReferenceFrame);
               tempFramePoint.set(depthPoint.x, depthPoint.y, depthPoint.z);
//               tempFramePoint.set(depthPoint.x, depthPoint.y - (height / 2.0f), depthPoint.z);
//               tempFramePoint.changeFrame(ReferenceFrame.getWorldFrame());

               Point3D32 point = points.add();
               point.set(tempFramePoint);
               point.addZ(random.nextDouble() * 0.007);
            }
         }
      }
      pixmap.dispose();

//      frameBuffer.getColorBufferTexture().getTextureData().consumePixmap().getPixels().

      // bind original framebuffer?
   }

   public void dispose()
   {
      Gdx.gl.glDeleteFramebuffer(framebufferId);
      Gdx.gl.glDeleteTexture(depthTextureId);
   }

   public void setCameraWorldTransform(Matrix4 worldTransform)
   {
      camera.position.setZero();
      camera.up.set(0.0f, 0.0f, 1.0f);
      camera.direction.set(1.0f, 0.0f, 0.0f);
      camera.transform(worldTransform);
      GDXTools.toEuclid(worldTransform, tempWorldTransform);
      cameraReferenceFrame.setX(tempWorldTransform.getTranslation().getX());
      cameraReferenceFrame.setY(tempWorldTransform.getTranslation().getY());
      cameraReferenceFrame.setZ(tempWorldTransform.getTranslation().getZ());
      cameraReferenceFrame.setOrientationAndUpdate(tempWorldTransform.getRotationView());
   }

   public PerspectiveCamera getCamera()
   {
      return camera;
   }

   public RecyclingArrayList<Point3D32> getPoints()
   {
      return points;
   }
}
