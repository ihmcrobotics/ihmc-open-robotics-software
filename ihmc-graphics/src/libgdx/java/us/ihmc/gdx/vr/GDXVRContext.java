package us.ihmc.gdx.vr;

import static org.lwjgl.openvr.VR.VR_ShutdownInternal;

import java.nio.IntBuffer;
import java.util.*;
import java.util.function.Consumer;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import org.lwjgl.PointerBuffer;
import org.lwjgl.opengl.GL41;
import org.lwjgl.openvr.*;

import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Pixmap.Format;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.VertexAttribute;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.graphics.g3d.model.MeshPart;
import com.badlogic.gdx.graphics.g3d.model.Node;
import com.badlogic.gdx.graphics.g3d.model.NodePart;
import com.badlogic.gdx.graphics.glutils.PixmapTextureData;
import com.badlogic.gdx.utils.BufferUtils;
import com.badlogic.gdx.utils.GdxRuntimeException;
import com.badlogic.gdx.utils.ObjectMap;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.gdx.sceneManager.GDX3DSceneManager;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.log.LogTools;
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
public class GDXVRContext
{
   // couple of scratch buffers
   private final IntBuffer errorPointer = BufferUtils.newIntBuffer(1);
   private final IntBuffer widthPointer = BufferUtils.newIntBuffer(1);
   private final IntBuffer heightPointer = BufferUtils.newIntBuffer(1);

   // per eye data such as rendering surfaces, textures, regions, cameras etc. for each eye
   private final SideDependentList<GDXVREye> eyes = new SideDependentList<>();

   // internal native objects to get device poses
   private TrackedDevicePose.Buffer trackedDevicePoses;
   private TrackedDevicePose.Buffer trackedDeviceGamePoses;

   // devices, their poses and listeners
   private final GDXVRDevice[] devices = new GDXVRDevice[VR.k_unMaxTrackedDeviceCount];
   private final RigidBodyTransform tempHeadsetTransform = new RigidBodyTransform();
   private ModelInstance headsetCoordinateFrame;

   {
      Arrays.fill(devices, null); // null means device is not connected
   }
   private VREvent event;
   private final SideDependentList<Integer> controllerIndexes = new SideDependentList<>(null, null);
   private final HashMap<Integer, RobotSide> indexToControlllerSideMap = new HashMap<>();
   private int width;
   private int height;
   private Integer headsetIndex = null;
   private final TreeSet<Integer> trackerIndexes = new TreeSet<>();
   private final TreeSet<Integer> genericIndexes = new TreeSet<>();
   private final ArrayList<Consumer<GDXVRContext>> vrInputProcessors = new ArrayList<>();

   // render models
   private final ObjectMap<String, Model> models = new ObjectMap<>();

   // book keeping
   private RobotSide currentEye = null;
   private boolean initialDevicesReported = false;

   // ReferenceFrame.getWorldFrame() is Z-up frame
   // Finger axis definition is right hand, Thumb +Z, Index +X, Middle +Y
   // The default orientation of the OpenVR frame is such that
   // your thumb is pointing at your face and your index finger pointing right
   // The default orientation of the IHMC Zup frame is such that
   // your thumb is up and your index finger is pointing away from you
   public static final RigidBodyTransformReadOnly openVRYUpToIHMCZUpSpace = new RigidBodyTransform(
         new YawPitchRoll(          // For this transformation, we start with IHMC ZUp with index forward and thumb up
            Math.toRadians(-90.0),  // rotating around thumb, index goes forward to right
            Math.toRadians(0.0),    // no rotation about middle finger
            Math.toRadians(90.0)    // rotating about index finger, thumb goes up to toward you
         ),
         new Point3D()
   );
   private final RigidBodyTransform tempVRPlayAreaZUp = new RigidBodyTransform();
   /** When the VR player teleports, it adds onto the transform from VR play area frame to world ZUp frame. */
   private final RigidBodyTransform totalTransformFromVRPlayAreaToIHMCZUpWorld = new RigidBodyTransform();
   /** The VR play area is on the floor in the center of your VR tracker space area. Also called tracker frame. */
   private final ReferenceFrame vrPlayAreaYUpZBackFrame
         = ReferenceFrameTools.constructFrameWithChangingTransformToParent("vrPlayAreaFrame",
                                                                           ReferenceFrame.getWorldFrame(),
                                                                           totalTransformFromVRPlayAreaToIHMCZUpWorld);

   public void initSystem()
   {
      LogTools.info("Initializing");
      event = VREvent.create(); // takes 92 ms
      trackedDevicePoses = TrackedDevicePose.create(VR.k_unMaxTrackedDeviceCount); // 10 ms
      trackedDeviceGamePoses = TrackedDevicePose.create(VR.k_unMaxTrackedDeviceCount); // 10 ms

      int token = VR.VR_InitInternal(errorPointer, VR.EVRApplicationType_VRApplication_Scene); // takes 148 ms
      checkInitError(errorPointer);
      OpenVR.create(token); // takes 24 ms

      VR.VR_GetGenericInterface(VR.IVRCompositor_Version, errorPointer);
      checkInitError(errorPointer);

      VR.VR_GetGenericInterface(VR.IVRRenderModels_Version, errorPointer);
      checkInitError(errorPointer);

      VRSystem.VRSystem_GetRecommendedRenderTargetSize(widthPointer, heightPointer);
      float renderTargetMultiplier = 1.0f; // multiplier to scale the render surface dimensions as a replacement for multisampling
      width = (int) (widthPointer.get(0) * renderTargetMultiplier);
      height = (int) (heightPointer.get(0) * renderTargetMultiplier);
   }

   /** Needs to be on libGDX thread. */
   public void setupEyes()
   {
      LogTools.info("VR per eye render size: {} x {}", width, height);
      for (RobotSide side : RobotSide.values)
      {
         eyes.set(side, new GDXVREye(side, () -> devices[headsetIndex], vrPlayAreaYUpZBackFrame, width, height));
      }
      headsetCoordinateFrame = GDXModelPrimitives.createCoordinateFrameInstance(0.3);
   }

   private void checkInitError(IntBuffer errorBuffer)
   {
      if (errorBuffer.get(0) != VR.EVRInitError_VRInitError_None)
      {
         int error = errorBuffer.get(0);
         throw new GdxRuntimeException("VR Initialization error: " + VR.VR_GetVRInitErrorAsEnglishDescription(error));
      }
   }

   /** This method waits for OpenVR to say "go" and gathers the latest data.
    *  The goal is to submit frames to the eyes ASAP after this returns,
    *  then get back to this method before the frame is over. */
   public void waitGetPoses()
   {
      VRCompositor.VRCompositor_WaitGetPoses(trackedDevicePoses, trackedDeviceGamePoses);
      // VRCompositor.VRCompositor_GetLastPoses(trackedDevicePoses, trackedDeviceGamePoses); // Is there a way to wait better?
   }

   /** Must be called before begin! */
   public void pollEvents()
   {
      if (!initialDevicesReported)
      {
         for (int deviceIndex = 0; deviceIndex < VR.k_unMaxTrackedDeviceCount; deviceIndex++)
         {
            if (VRSystem.VRSystem_IsTrackedDeviceConnected(deviceIndex))
            {
               createDevice(deviceIndex);
            }
         }
         initialDevicesReported = true;
      }

      // TODO: cache the devices; don't iterate over 64 every time?
      // TODO: maybe do it at a lower frequency just to catch new devices?
      for (int deviceIndex = 0; deviceIndex < VR.k_unMaxTrackedDeviceCount; deviceIndex++)
      {
         GDXVRDevice device = devices[deviceIndex];
         if (device != null)
         {
            TrackedDevicePose trackedPose = trackedDevicePoses.get(deviceIndex);

            HmdVector3 velocity = trackedPose.vVelocity();
            HmdVector3 angularVelocity = trackedPose.vAngularVelocity();
            HmdMatrix34 openVRRigidBodyTransform = trackedPose.mDeviceToAbsoluteTracking();

            device.getVelocity().set(velocity.v(0), velocity.v(1), velocity.v(2));
            device.getAngularVelocity().set(angularVelocity.v(0), angularVelocity.v(1), angularVelocity.v(2));
            device.setValid(trackedPose.bPoseIsValid());
            device.updatePoseInTrackerFrame(openVRRigidBodyTransform, vrPlayAreaYUpZBackFrame);
         }
      }

      getHeadset(headset ->
      {
         headset.getPose().changeFrame(ReferenceFrame.getWorldFrame());
         headset.getPose().get(tempHeadsetTransform);
         GDXTools.toGDX(tempHeadsetTransform, headsetCoordinateFrame.transform);
         headset.getPose(ReferenceFrame.getWorldFrame(), headsetCoordinateFrame.transform);
      });

      for (RobotSide side : RobotSide.values)
      {
         getController(side, GDXVRDevice::resetBeforeUpdate);
      }

      while (VRSystem.VRSystem_PollNextEvent(event))
      {
         int deviceIndex = event.trackedDeviceIndex();
         if (deviceIndex < 0 || deviceIndex > VR.k_unMaxTrackedDeviceCount)
            continue;
         int button = 0;

         switch (event.eventType())
         {
            case VR.EVREventType_VREvent_TrackedDeviceActivated:
               createDevice(deviceIndex);
               break;
            case VR.EVREventType_VREvent_TrackedDeviceDeactivated:
               deviceIndex = event.trackedDeviceIndex();
               if (devices[deviceIndex] == null)
                  continue;
               updateDevice(deviceIndex, null);
               break;
            case VR.EVREventType_VREvent_ButtonPress:
               if (devices[deviceIndex] == null)
                  continue;
               button = event.data().controller().button();
               devices[deviceIndex].setButton(button, true);
               devices[deviceIndex].setButtonPressed(button);
               break;
            case VR.EVREventType_VREvent_ButtonUnpress:
               if (devices[deviceIndex] == null)
                  continue;
               button = event.data().controller().button();
               devices[deviceIndex].setButton(button, false);
               devices[deviceIndex].setButtonReleased(button);
               break;
         }
      }

      for (Consumer<GDXVRContext> vrInputProcessor : vrInputProcessors)
      {
         vrInputProcessor.accept(this);
      }
   }

   private void updateDevice(int deviceIndex, GDXVRDevice device)
   {
      if (device == null)
      {
         for (RobotSide side : RobotSide.values)
         {
            Integer controllerIndex = controllerIndexes.get(side);
            if (controllerIndex != null && controllerIndex == deviceIndex)
            {
               controllerIndexes.set(side, null);
            }

            if (indexToControlllerSideMap.get(deviceIndex) != null)
            {
               indexToControlllerSideMap.put(deviceIndex, null);
            }
         }
         if (headsetIndex == deviceIndex)
         {
            headsetIndex = null;
         }
         trackerIndexes.remove(deviceIndex);
         genericIndexes.remove(deviceIndex);
      }

      devices[deviceIndex] = device;
   }

   private void createDevice(int deviceIndex)
   {
      GDXVRDeviceType type;
      int deviceClass = VRSystem.VRSystem_GetTrackedDeviceClass(deviceIndex);
      switch (deviceClass)
      {
         case VR.ETrackedDeviceClass_TrackedDeviceClass_HMD:
            type = GDXVRDeviceType.HeadMountedDisplay;
            headsetIndex = deviceIndex;
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

      GDXVRControllerRole controllerRole = GDXVRControllerRole.Unknown;
      if (type == GDXVRDeviceType.Controller)
      {
         int controllerRoleID = VRSystem.VRSystem_GetControllerRoleForTrackedDeviceIndex(deviceIndex);
         switch (controllerRoleID)
         {
            case VR.ETrackedControllerRole_TrackedControllerRole_LeftHand:
               controllerIndexes.set(RobotSide.LEFT, deviceIndex);
               indexToControlllerSideMap.put(deviceIndex, RobotSide.LEFT);
               controllerRole = GDXVRControllerRole.LeftHand;
               break;
            case VR.ETrackedControllerRole_TrackedControllerRole_RightHand:
               controllerIndexes.set(RobotSide.RIGHT, deviceIndex);
               indexToControlllerSideMap.put(deviceIndex, RobotSide.RIGHT);
               controllerRole = GDXVRControllerRole.RightHand;
               break;
         }
      }
      if (type == GDXVRDeviceType.BaseStation)
      {
         trackerIndexes.add(deviceIndex);
      }
      if (type == GDXVRDeviceType.Generic)
      {
         genericIndexes.add(deviceIndex);
      }
      updateDevice(deviceIndex, new GDXVRDevice(deviceIndex, type, controllerRole, this::loadRenderModel));
   }

   /**
    * Completes rendering and submits the rendering surfaces to the
    * head mounted display.
    */
   public void renderEyes(GDX3DSceneManager sceneManager)
   {
      for (RobotSide side : RobotSide.values)
      {
         eyes.get(side).render(sceneManager);
      }

      // These lines take some time
      // TODO: Do anti aliasing and set EVRSubmitFlags_Submit_GlRenderBuffer
      VRCompositor.VRCompositor_Submit(VR.EVREye_Eye_Left, eyes.get(RobotSide.LEFT).getOpenVRTexture(), null, VR.EVRSubmitFlags_Submit_Default);
      VRCompositor.VRCompositor_Submit(VR.EVREye_Eye_Right, eyes.get(RobotSide.RIGHT).getOpenVRTexture(), null, VR.EVRSubmitFlags_Submit_Default);
      GL41.glFlush(); // recommended by OpenVR docs. Needed?
   }

   public void dispose()
   {
      for (GDXVREye eyeData : eyes)
         eyeData.getFrameBuffer().dispose();

      // TODO: Dispose models

      VR_ShutdownInternal();
   }

   public void teleport(Consumer<RigidBodyTransform> vrPlayAreaZUpConsumer)
   {
      vrPlayAreaZUpConsumer.accept(tempVRPlayAreaZUp);
      teleport(tempVRPlayAreaZUp);
   }

   public void teleport(RigidBodyTransform vrPlayAreaZUp)
   {
      totalTransformFromVRPlayAreaToIHMCZUpWorld.set(openVRYUpToIHMCZUpSpace);
      vrPlayAreaZUp.transform(totalTransformFromVRPlayAreaToIHMCZUpWorld);
      vrPlayAreaYUpZBackFrame.update();
   }

   private Model loadRenderModel(String name)
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
      MeshPart meshPart = new MeshPart(name, mesh, 0, renderModel.unTriangleCount() * 3, GL41.GL_TRIANGLES);
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

   public void addVRInputProcessor(Consumer<GDXVRContext> processVRInput)
   {
      vrInputProcessors.add(processVRInput);
   }

   /**
    * Gives a controller only if it's connected. Otherwise no worries.
    */
   public void getController(RobotSide side, Consumer<GDXVRDevice> controllerIfConnected)
   {
      Integer controllerIndex = controllerIndexes.get(side);
      if (controllerIndex != null)
      {
         controllerIfConnected.accept(devices[controllerIndex]);
      }
   }

   /**
    * Gives the headset only if it's connected. Otherwise no worries.
    */
   public void getHeadset(Consumer<GDXVRDevice> headsetIfConnected)
   {
      if (isHeadsetConnected())
      {
         headsetIfConnected.accept(devices[headsetIndex]);
      }
   }

   public ModelInstance getHeadsetCoordinateFrame()
   {
      return headsetCoordinateFrame;
   }

   public boolean isHeadsetConnected()
   {
      return headsetIndex != null;
   }

   public void getBaseStations(Consumer<GDXVRDevice> baseStationConsumer)
   {
      for (Integer trackerIndex : trackerIndexes)
      {
         baseStationConsumer.accept(devices[trackerIndex]);
      }
   }

   public void getGenericDevices(Consumer<GDXVRDevice> genericDeviceConsumer)
   {
      for (Integer trackerIndex : genericIndexes)
      {
         genericDeviceConsumer.accept(devices[trackerIndex]);
      }
   }

   public SideDependentList<GDXVREye> getEyes()
   {
      return eyes;
   }

   public ReferenceFrame getVRPlayAreaFrame()
   {
      return vrPlayAreaYUpZBackFrame;
   }
}