package us.ihmc.gdx.vr;

import static org.lwjgl.openvr.VR.VR_ShutdownInternal;

import java.nio.IntBuffer;

import org.lwjgl.PointerBuffer;
import org.lwjgl.openvr.OpenVR;
import org.lwjgl.openvr.RenderModel;
import org.lwjgl.openvr.RenderModelTextureMap;
import org.lwjgl.openvr.RenderModelVertex;
import org.lwjgl.openvr.TrackedDevicePose;
import org.lwjgl.openvr.VR;
import org.lwjgl.openvr.VRCompositor;
import org.lwjgl.openvr.VREvent;
import org.lwjgl.openvr.VRRenderModels;
import org.lwjgl.openvr.VRSystem;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Pixmap.Format;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.VertexAttribute;
import com.badlogic.gdx.graphics.g2d.SpriteBatch;
import com.badlogic.gdx.graphics.g2d.TextureRegion;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.graphics.g3d.model.MeshPart;
import com.badlogic.gdx.graphics.g3d.model.Node;
import com.badlogic.gdx.graphics.g3d.model.NodePart;
import com.badlogic.gdx.graphics.glutils.FrameBuffer;
import com.badlogic.gdx.graphics.glutils.GLFrameBuffer;
import com.badlogic.gdx.graphics.glutils.PixmapTextureData;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.BufferUtils;
import com.badlogic.gdx.utils.Disposable;
import com.badlogic.gdx.utils.GdxRuntimeException;
import com.badlogic.gdx.utils.ObjectMap;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

/**
 * Responsible for initializing the VR system, managing rendering surfaces,
 * getting tracking device poses, submitting the rendering results to the HMD
 * and rendering the surfaces side by side to the companion window on the
 * desktop. Wrapper around OpenVR.
 * <p>
 * FIXME add multisampling plus draw/resolve buffers
 */
public class GDXVRContext implements Disposable
{
   // couple of scratch buffers
   private final IntBuffer error = BufferUtils.newIntBuffer(1);
   private final IntBuffer scratch = BufferUtils.newIntBuffer(1), scratch2 = BufferUtils.newIntBuffer(1);

   // per eye data such as rendering surfaces, textures, regions, cameras etc. for each eye
   private final SideDependentList<GDXVRPerEyeData> perEyeData = new SideDependentList<>();

   // batcher to draw eye rendering surface to companion window
   private final SpriteBatch batcher;

   // internal native objects to get device poses
   private final TrackedDevicePose.Buffer trackedDevicePoses = TrackedDevicePose.create(VR.k_unMaxTrackedDeviceCount);
   private final TrackedDevicePose.Buffer trackedDeviceGamePoses = TrackedDevicePose.create(VR.k_unMaxTrackedDeviceCount);

   // devices, their poses and listeners
   private final GDXVRDevicePose[] devicePoses = new GDXVRDevicePose[VR.k_unMaxTrackedDeviceCount];
   private final GDXVRDevice[] devices = new GDXVRDevice[VR.k_unMaxTrackedDeviceCount];
   private final Array<GDXVRDeviceListener> listeners = new Array<>();
   private final VREvent event = VREvent.create();

   // render models
   private final ObjectMap<String, Model> models = new ObjectMap<>();

   // book keeping
   private RobotSide currentEye = null;
   private boolean renderingStarted = false;
   private boolean initialDevicesReported = false;

   // offsets for translation and rotation from tracker to world space
   private final Vector3 trackerSpaceOriginToWorldSpaceTranslationOffset = new Vector3();
   private final Matrix4 trackerSpaceToWorldspaceRotationOffset = new Matrix4();
   private final RigidBodyTransformReadOnly toZUpXForward = new RigidBodyTransform(new YawPitchRoll(Math.toRadians(90.0),
                                                                                                    Math.toRadians(90.0),
                                                                                                    Math.toRadians(0.0)), new Point3D());
   private final RigidBodyTransformReadOnly toZForwardYUp;
   {
      RigidBodyTransform toXForwardZUpTemp = new RigidBodyTransform(toZUpXForward);
      toXForwardZUpTemp.invert();
      toZForwardYUp = toXForwardZUpTemp;
   }

   /**
    * Creates a new GDXVRContext, initializes the VR system, and
    * sets up rendering surfaces with depth attachments.
    */
   public GDXVRContext()
   {
      this(1, false);
   }

   /**
    * Creates a new GDXVRContext, initializes the VR system,
    * and sets up rendering surfaces.
    *
    * @param renderTargetMultiplier multiplier to scale the render surface dimensions as a replacement for multisampling
    * @param hasStencil             whether the rendering surfaces should have a stencil buffer
    * @throws {@link GdxRuntimeException} if the system could not be initialized
    */
   public GDXVRContext(float renderTargetMultiplier, boolean hasStencil)
   {
      int token = VR.VR_InitInternal(error, VR.EVRApplicationType_VRApplication_Scene);
      checkInitError(error);
      OpenVR.create(token);

      VR.VR_GetGenericInterface(VR.IVRCompositor_Version, error);
      checkInitError(error);

      VR.VR_GetGenericInterface(VR.IVRRenderModels_Version, error);
      checkInitError(error);

      for (int i = 0; i < devicePoses.length; i++)
      {
         devicePoses[i] = new GDXVRDevicePose(i);
      }

      VRSystem.VRSystem_GetRecommendedRenderTargetSize(scratch, scratch2);
      int width = (int) (scratch.get(0) * renderTargetMultiplier);
      int height = (int) (scratch2.get(0) * renderTargetMultiplier);

      setupEye(RobotSide.LEFT, width, height, hasStencil);
      setupEye(RobotSide.RIGHT, width, height, hasStencil);

      batcher = new SpriteBatch();
   }

   private void setupEye(RobotSide eye, int width, int height, boolean hasStencil)
   {
      FrameBuffer buffer = new FrameBuffer(Format.RGBA8888, width, height, true, hasStencil);
      TextureRegion region = new TextureRegion(buffer.getColorBufferTexture());
      region.flip(false, true);
      GDXVRCamera camera = new GDXVRCamera(this, eye);
      camera.near = 0.1f;
      camera.far = 1000f;
      perEyeData.set(eye, new GDXVRPerEyeData(buffer, region, camera));
   }

   private void checkInitError(IntBuffer errorBuffer)
   {
      if (errorBuffer.get(0) != VR.EVRInitError_VRInitError_None)
      {
         int error = errorBuffer.get(0);
         throw new GdxRuntimeException("VR Initialization error: " + VR.VR_GetVRInitErrorAsEnglishDescription(error));
      }
   }

   /**
    * Returns the tracker space to world space translation offset. All positional vectors
    * returned by {@link GDXVRDevice} methods taking a {@link GDXVRSpace#World} are
    * multiplied offset by this vector. This allows offsetting {@link GDXVRDevice}
    * positions and orientations in world space.
    */
   public Vector3 getTrackerSpaceOriginToWorldSpaceTranslationOffset()
   {
      return trackerSpaceOriginToWorldSpaceTranslationOffset;
   }

   /**
    * Returns the tracker space to world space rotation offset. All rotational vectors
    * returned by {@link GDXVRDevice} methods taking a {@link GDXVRSpace#World} are
    * rotated by this offset. This allows offsetting {@link GDXVRDevice}
    * orientations in world space. The matrix needs to only have
    * rotational components.
    */
   public Matrix4 getTrackerSpaceToWorldspaceRotationOffset()
   {
      return trackerSpaceToWorldspaceRotationOffset;
   }

   public RigidBodyTransformReadOnly getToZForwardYUp()
   {
      return toZForwardYUp;
   }

   public RigidBodyTransformReadOnly getToZUpXForward()
   {
      return toZUpXForward;
   }

   /**
    * Adds a {@link GDXVRDeviceListener} to receive events
    */
   public void addListener(GDXVRDeviceListener listener)
   {
      this.listeners.add(listener);
   }

   /**
    * Removes a {@link GDXVRDeviceListener}
    */
   public void removeListener(GDXVRDeviceListener listener)
   {
      this.listeners.removeValue(listener, true);
   }

   /**
    * @return the first {@link GDXVRDevice} of the given {@link GDXVRDeviceType} or null.
    */
   public GDXVRDevice getDeviceByType(GDXVRDeviceType type)
   {
      for (GDXVRDevice d : devices)
      {
         if (d != null && d.getType() == type)
            return d;
      }
      return null;
   }

   /**
    * @return all {@link GDXVRDevice} instances of the given {@link GDXVRDeviceType}.
    */
   public Array<GDXVRDevice> getDevicesByType(GDXVRDeviceType type)
   {
      Array<GDXVRDevice> result = new Array<>();
      for (GDXVRDevice d : devices)
      {
         if (d != null && d.getType() == type)
            result.add(d);
      }
      return result;
   }

   /**
    * @return all currently connected {@link GDXVRDevice} instances.
    */
   public Array<GDXVRDevice> getDevices()
   {
      Array<GDXVRDevice> result = new Array<>();
      for (GDXVRDevice d : devices)
      {
         if (d != null)
            result.add(d);
      }
      return result;
   }

   /**
    * @return the {@link GDXVRDevice} of ype {@link GDXVRDeviceType#Controller} that matches the role, or null.
    */
   public GDXVRDevice getControllerByRole(GDXVRControllerRole role)
   {
      for (GDXVRDevice d : devices)
      {
         if (d != null && d.getType() == GDXVRDeviceType.Controller && d.getControllerRole() == role)
            return d;
      }
      return null;
   }

   public GDXVRDevicePose getDevicePose(int deviceIndex)
   {
      if (deviceIndex < 0 || deviceIndex >= devicePoses.length)
         throw new IndexOutOfBoundsException("Device index must be >= 0 and <= " + devicePoses.length);
      return devicePoses[deviceIndex];
   }

   /**
    * Start rendering. Call beginEye to setup rendering
    * for each individual eye. End rendering by calling
    * #end
    */
   public void begin()
   {
      if (renderingStarted)
         throw new GdxRuntimeException("Last begin() call not completed, call end() before starting a new render");
      renderingStarted = true;

      perEyeData.get(RobotSide.LEFT).getCamera().update();
      perEyeData.get(RobotSide.RIGHT).getCamera().update();
   }

   /**
    * Get the latest tracking data and send events to {@link GDXVRDeviceListener} instance registered with the context.
    * <p>
    * Must be called before begin!
    */
   public void pollEvents()
   {
      VRCompositor.VRCompositor_WaitGetPoses(trackedDevicePoses, trackedDeviceGamePoses);

      if (!initialDevicesReported)
      {
         for (int index = 0; index < devices.length; index++)
         {
            if (VRSystem.VRSystem_IsTrackedDeviceConnected(index))
            {
               createDevice(index);
               for (GDXVRDeviceListener l : listeners)
               {
                  l.connected(devices[index]);
               }
            }
         }
         initialDevicesReported = true;
      }

      for (int device = 0; device < VR.k_unMaxTrackedDeviceCount; device++)
      {
         TrackedDevicePose trackedPose = trackedDevicePoses.get(device);
         GDXVRDevicePose pose = devicePoses[device];

         GDXVRTools.hmdMat34ToMatrix4(trackedPose.mDeviceToAbsoluteTracking(), pose.getTransform());
         pose.getVelocity().set(trackedPose.vVelocity().v(0), trackedPose.vVelocity().v(1), trackedPose.vVelocity().v(2));
         pose.getAngularVelocity().set(trackedPose.vAngularVelocity().v(0), trackedPose.vAngularVelocity().v(1), trackedPose.vAngularVelocity().v(2));
         pose.setConnected(trackedPose.bDeviceIsConnected());
         pose.setValid(trackedPose.bPoseIsValid());

         if (devices[device] != null)
         {
            devices[device].updateAxesAndPosition();
            if (devices[device].getModelInstance() != null)
            {
               devices[device].getModelInstance().transform.idt()
                                                      .translate(trackerSpaceOriginToWorldSpaceTranslationOffset)
                                                      .mul(trackerSpaceToWorldspaceRotationOffset)
                                                      .mul(pose.getTransform());
            }
         }
      }

      while (VRSystem.VRSystem_PollNextEvent(event))
      {
         int index = event.trackedDeviceIndex();
         if (index < 0 || index > VR.k_unMaxTrackedDeviceCount)
            continue;
         int button = 0;

         switch (event.eventType())
         {
            case VR.EVREventType_VREvent_TrackedDeviceActivated:
               createDevice(index);
               for (GDXVRDeviceListener l : listeners)
               {
                  l.connected(devices[index]);
               }
               break;
            case VR.EVREventType_VREvent_TrackedDeviceDeactivated:
               index = event.trackedDeviceIndex();
               if (devices[index] == null)
                  continue;
               for (GDXVRDeviceListener l : listeners)
               {
                  l.disconnected(devices[index]);
               }
               devices[index] = null;
               break;
            case VR.EVREventType_VREvent_ButtonPress:
               if (devices[index] == null)
                  continue;
               button = event.data().controller().button();
               devices[index].setButton(button, true);
               for (GDXVRDeviceListener l : listeners)
               {
                  l.buttonPressed(devices[index], button);
               }
               break;
            case VR.EVREventType_VREvent_ButtonUnpress:
               if (devices[index] == null)
                  continue;
               button = event.data().controller().button();
               devices[index].setButton(button, false);
               for (GDXVRDeviceListener l : listeners)
               {
                  l.buttonReleased(devices[index], button);
               }
               break;
         }
      }
   }

   private void createDevice(int index)
   {
      GDXVRDeviceType type = null;
      int deviceClass = VRSystem.VRSystem_GetTrackedDeviceClass(index);
      switch (deviceClass)
      {
         case VR.ETrackedDeviceClass_TrackedDeviceClass_HMD:
            type = GDXVRDeviceType.HeadMountedDisplay;
            break;
         case VR.ETrackedDeviceClass_TrackedDeviceClass_Controller:
            type = GDXVRDeviceType.Controller;
            break;
         case VR.ETrackedDeviceClass_TrackedDeviceClass_TrackingReference:
            type = GDXVRDeviceType.BaseStation;
            break;
         case VR.ETrackedDeviceClass_TrackedDeviceClass_GenericTracker:
            type = GDXVRDeviceType.Generic;
            break;
         default:
            return;
      }

      GDXVRControllerRole role = GDXVRControllerRole.Unknown;
      if (type == GDXVRDeviceType.Controller)
      {
         int r = VRSystem.VRSystem_GetControllerRoleForTrackedDeviceIndex(index);
         switch (r)
         {
            case VR.ETrackedControllerRole_TrackedControllerRole_LeftHand:
               role = GDXVRControllerRole.LeftHand;
               break;
            case VR.ETrackedControllerRole_TrackedControllerRole_RightHand:
               role = GDXVRControllerRole.RightHand;
               break;
         }
      }
      devices[index] = new GDXVRDevice(this, devicePoses[index], type, role);
      devices[index].updateAxesAndPosition();
   }

   /**
    * @return the {@link GDXVRPerEyeData} such as rendering surface and camera
    */
   public GDXVRPerEyeData getEyeData(RobotSide eye)
   {
      return perEyeData.get(eye);
   }

   /**
    * Start rendering to the rendering surface for the given eye.
    * Complete by calling {@link #endEye()}.
    */
   public void beginEye(RobotSide eye)
   {
      if (!renderingStarted)
         throw new GdxRuntimeException("Call begin() before calling beginEye()");
      if (currentEye != null)
         throw new GdxRuntimeException("Last beginEye() call not completed, call endEye() before starting a new render");
      currentEye = eye;
      perEyeData.get(eye).getFrameBuffer().begin();
   }

   /**
    * Completes rendering to the rendering surface for the given eye.
    */
   public void endEye()
   {
      if (currentEye == null)
         throw new GdxRuntimeException("Call beginEye() before endEye()");
      perEyeData.get(currentEye).getFrameBuffer().end();
      currentEye = null;
   }

   /**
    * Completes rendering and submits the rendering surfaces to the
    * head mounted display.
    */
   public void end()
   {
      if (!renderingStarted)
         throw new GdxRuntimeException("Call begin() before end()");
      renderingStarted = false;

      VRCompositor.VRCompositor_Submit(VR.EVREye_Eye_Left, perEyeData.get(RobotSide.LEFT).getTexture(), null, VR.EVRSubmitFlags_Submit_Default);
      VRCompositor.VRCompositor_Submit(VR.EVREye_Eye_Right, perEyeData.get(RobotSide.RIGHT).getTexture(), null, VR.EVRSubmitFlags_Submit_Default);
   }

   public void dispose()
   {
      for (GDXVRPerEyeData eyeData : perEyeData)
         eyeData.getFrameBuffer().dispose();
      batcher.dispose();
      VR_ShutdownInternal();
   }

   /**
    * Resizes the companion window so the rendering buffers
    * can be displayed without stretching.
    */
   public void resizeCompanionWindow()
   {
      GLFrameBuffer<Texture> buffer = perEyeData.get(RobotSide.LEFT).getFrameBuffer();
      Gdx.graphics.setWindowedMode(buffer.getWidth(), buffer.getHeight());
   }

   /**
    * Renders the content of the given eye's rendering surface
    * to the entirety of the companion window.
    */
   public void renderToCompanionWindow(RobotSide eye)
   {
      GLFrameBuffer<Texture> buffer = perEyeData.get(eye).getFrameBuffer();
      TextureRegion region = perEyeData.get(eye).getRegion();
      batcher.getProjectionMatrix().setToOrtho2D(0, 0, buffer.getWidth(), buffer.getHeight());
      batcher.begin();
      batcher.draw(region, 0, 0);
      batcher.end();
   }

   public Model loadRenderModel(String name)
   {
      if (models.containsKey(name))
         return models.get(name);

      // FIXME we load the models synchronously cause we are lazy
      int error = 0;
      PointerBuffer modelPointer = PointerBuffer.allocateDirect(1);
      do
      {
         error = VRRenderModels.VRRenderModels_LoadRenderModel_Async(name, modelPointer);
      }
      while (error == VR.EVRRenderModelError_VRRenderModelError_Loading);

      if (error != VR.EVRRenderModelError_VRRenderModelError_None)
         return null;
      RenderModel renderModel = new RenderModel(modelPointer.getByteBuffer(RenderModel.SIZEOF));

      error = 0;
      PointerBuffer texturePointer = PointerBuffer.allocateDirect(1);
      do
      {
         error = VRRenderModels.VRRenderModels_LoadTexture_Async(renderModel.diffuseTextureId(), texturePointer);
      }
      while (error == VR.EVRRenderModelError_VRRenderModelError_Loading);

      if (error != VR.EVRRenderModelError_VRRenderModelError_None)
      {
         VRRenderModels.VRRenderModels_FreeRenderModel(renderModel);
         return null;
      }

      RenderModelTextureMap renderModelTexture = new RenderModelTextureMap(texturePointer.getByteBuffer(RenderModelTextureMap.SIZEOF));

      // convert to a Model
      Mesh mesh = new Mesh(true,
                           renderModel.unVertexCount(),
                           renderModel.unTriangleCount() * 3,
                           VertexAttribute.Position(),
                           VertexAttribute.Normal(),
                           VertexAttribute.TexCoords(0));
      MeshPart meshPart = new MeshPart(name, mesh, 0, renderModel.unTriangleCount() * 3, GL20.GL_TRIANGLES);
      RenderModelVertex.Buffer vertices = renderModel.rVertexData();
      float[] packedVertices = new float[8 * renderModel.unVertexCount()];
      int i = 0;
      while (vertices.remaining() > 0)
      {
         RenderModelVertex v = vertices.get();
         packedVertices[i++] = v.vPosition().v(0);
         packedVertices[i++] = v.vPosition().v(1);
         packedVertices[i++] = v.vPosition().v(2);

         packedVertices[i++] = v.vNormal().v(0);
         packedVertices[i++] = v.vNormal().v(1);
         packedVertices[i++] = v.vNormal().v(2);

         packedVertices[i++] = v.rfTextureCoord().get(0);
         packedVertices[i++] = v.rfTextureCoord().get(1);
      }
      mesh.setVertices(packedVertices);
      short[] indices = new short[renderModel.unTriangleCount() * 3];
      renderModel.IndexData().get(indices);
      mesh.setIndices(indices);

      Pixmap pixmap = new Pixmap(renderModelTexture.unWidth(), renderModelTexture.unHeight(), Format.RGBA8888);
      byte[] pixels = new byte[renderModelTexture.unWidth() * renderModelTexture.unHeight() * 4];
      renderModelTexture.rubTextureMapData(pixels.length).get(pixels);
      pixmap.getPixels().put(pixels);
      pixmap.getPixels().position(0);
      Texture texture = new Texture(new PixmapTextureData(pixmap, pixmap.getFormat(), true, true));
      Material material = new Material(new TextureAttribute(TextureAttribute.Diffuse, texture));

      Model model = new Model();
      model.meshes.add(mesh);
      model.meshParts.add(meshPart);
      model.materials.add(material);
      Node node = new Node();
      node.id = name;
      node.parts.add(new NodePart(meshPart, material));
      model.nodes.add(node);
      model.manageDisposable(mesh);
      model.manageDisposable(texture);

      VRRenderModels.VRRenderModels_FreeRenderModel(renderModel);
      VRRenderModels.VRRenderModels_FreeTexture(renderModelTexture);

      models.put(name, model);

      return model;
   }
}