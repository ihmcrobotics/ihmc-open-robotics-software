package us.ihmc.gdx.vr;

import static org.lwjgl.openvr.VR.VR_ShutdownInternal;

import java.nio.IntBuffer;
import java.nio.LongBuffer;
import java.util.*;
import java.util.function.Consumer;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import org.lwjgl.opengl.GL41;
import org.lwjgl.openvr.*;

import com.badlogic.gdx.utils.BufferUtils;
import com.badlogic.gdx.utils.GdxRuntimeException;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.gdx.sceneManager.GDX3DScene;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.WorkspaceDirectory;
import us.ihmc.tools.io.WorkspaceFile;

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
   private final LongBuffer mainActionSetHandle = BufferUtils.newLongBuffer(1);
   private VRActiveActionSet.Buffer activeActionSets;

   private VREvent event;
   private int width;
   private int height;
   private final ArrayList<Consumer<GDXVRContext>> vrPickCalculators = new ArrayList<>();
   private final ArrayList<Consumer<GDXVRContext>> vrInputProcessors = new ArrayList<>();

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
   /** When the VR player teleports, it adds onto the transform from VR play area frame to world ZUp frame. */
   private final RigidBodyTransform teleportIHMCZUpToIHMCZUpWorld = new RigidBodyTransform();
   private final ReferenceFrame teleportFrameIHMCZUp
         = ReferenceFrameTools.constructFrameWithChangingTransformToParent("teleportFrame",
                                                                           ReferenceFrame.getWorldFrame(),
                                                                           teleportIHMCZUpToIHMCZUpWorld);
   /** The VR play area is on the floor in the center of your VR tracker space area. Also called tracker frame. */
   private final ReferenceFrame vrPlayAreaYUpZBackFrame
         = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("vrPlayAreaFrame",
                                                                           teleportFrameIHMCZUp,
                                                                           openVRYUpToIHMCZUpSpace);

   private final GDXVRHeadset headset = new GDXVRHeadset(vrPlayAreaYUpZBackFrame);
   private final SideDependentList<GDXVRController> controllers = new SideDependentList<>(new GDXVRController(RobotSide.LEFT, vrPlayAreaYUpZBackFrame),
                                                                                          new GDXVRController(RobotSide.RIGHT, vrPlayAreaYUpZBackFrame));
   private final HashMap<Integer, GDXVRBaseStation> baseStations = new HashMap<>();
   private final ArrayList<GDXVRPickResult> pickResults = new ArrayList<>();
   private GDXVRPickResult selectedPick = null;

   public void initSystem()
   {
      LogTools.info("Initializing");

      event = VREvent.create(); // about 92 ms
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

      WorkspaceDirectory directory = new WorkspaceDirectory("ihmc-open-robotics-software", "ihmc-graphics/src/libgdx/resources", getClass(), "/vr");
      WorkspaceFile actionManifestFile = new WorkspaceFile(directory, "actions.json");
      VRInput.VRInput_SetActionManifestPath(actionManifestFile.getFilePath().toString());

      VRInput.VRInput_GetActionSetHandle("/actions/main", mainActionSetHandle);
      headset.initSystem();
      for (RobotSide side : RobotSide.values)
      {
         controllers.get(side).initSystem();
      }
      // TODO: Bindings for /user/gamepad

      activeActionSets = VRActiveActionSet.create(1);
      activeActionSets.ulActionSet(mainActionSetHandle.get(0));
      activeActionSets.ulRestrictedToDevice(VR.k_ulInvalidInputValueHandle);
   }

   /** Needs to be on libGDX thread. */
   public void setupEyes()
   {
      LogTools.info("VR per eye render size: {} x {}", width, height);
      for (RobotSide side : RobotSide.values)
      {
         eyes.set(side, new GDXVREye(side, headset, width, height));
      }
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

   /**
    *  For input, see https://github.com/ValveSoftware/openvr/wiki/SteamVR-Input
    *  and https://github.com/ValveSoftware/openvr/issues/1151
    */
   public void pollEvents()
   {
      VRInput.VRInput_UpdateActionState(activeActionSets, VRActiveActionSet.SIZEOF);

      headset.update(trackedDevicePoses);

      for (RobotSide side : RobotSide.values)
      {
         controllers.get(side).update(trackedDevicePoses);
      }

      while (VRSystem.VRSystem_PollNextEvent(event))
      {
         int deviceIndex = event.trackedDeviceIndex();
         if (event.eventType() == VR.EVREventType_VREvent_TrackedDeviceActivated)
         {
            int deviceClass = VRSystem.VRSystem_GetTrackedDeviceClass(deviceIndex);
            if (deviceClass == VR.ETrackedDeviceClass_TrackedDeviceClass_TrackingReference)
            {
               baseStations.put(deviceIndex, new GDXVRBaseStation(vrPlayAreaYUpZBackFrame, deviceIndex));
            }
         }
         else if (event.eventType() == VR.EVREventType_VREvent_TrackedDeviceDeactivated)
         {
            int deviceClass = VRSystem.VRSystem_GetTrackedDeviceClass(deviceIndex);
            if (deviceClass == VR.ETrackedDeviceClass_TrackedDeviceClass_TrackingReference)
            {
               baseStations.remove(deviceIndex);
            }
         }
      }

      pickResults.clear();
      for (Consumer<GDXVRContext> vrPickCalculator : vrPickCalculators)
      {
         vrPickCalculator.accept(this);
      }
      calculateSelectedPick();
      for (Consumer<GDXVRContext> vrInputProcessor : vrInputProcessors)
      {
         vrInputProcessor.accept(this);
      }
   }

   private void calculateSelectedPick()
   {
      selectedPick = null;
      for (GDXVRPickResult pickResult : pickResults)
      {
         if (selectedPick == null)
         {
            selectedPick = pickResult;
         }
         else if (pickResult.getDistanceToCamera() < selectedPick.getDistanceToCamera())
         {
            selectedPick = pickResult;
         }
      }
   }

   /**
    * Completes rendering and submits the rendering surfaces to the
    * head mounted display.
    */
   public void renderEyes(GDX3DScene scene)
   {
      for (RobotSide side : RobotSide.values)
      {
         eyes.get(side).render(scene);
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

   public void teleport(Consumer<RigidBodyTransform> teleportIHMCZUpToIHMCZUpWorldConsumer)
   {
      teleportIHMCZUpToIHMCZUpWorldConsumer.accept(teleportIHMCZUpToIHMCZUpWorld);
      teleportFrameIHMCZUp.update();
   }

   public void addVRPickCalculator(Consumer<GDXVRContext> calculateVRPick)
   {
      vrPickCalculators.add(calculateVRPick);
   }

   public void addVRInputProcessor(Consumer<GDXVRContext> processVRInput)
   {
      vrInputProcessors.add(processVRInput);
   }

   public void getHeadsetRenderable(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (headset.isConnected())
      {
         ModelInstance modelInstance = headset.getModelInstance();
         if (modelInstance != null)
         {
            modelInstance.getRenderables(renderables, pool);
         }
      }
   }

   public void getControllerRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (RobotSide side : RobotSide.values)
      {
         GDXVRController controller = controllers.get(side);
         if (controller.isConnected())
         {
            ModelInstance modelInstance = controller.getModelInstance();
            if (modelInstance != null)
            {
               modelInstance.getRenderables(renderables, pool);
               controller.getPickSphere().getRenderables(renderables, pool);
            }
         }
      }
   }

   public void getBaseStationRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (GDXVRBaseStation baseStation : baseStations.values())
      {
         ModelInstance modelInstance = baseStation.getModelInstance();
         if (modelInstance != null)
         {
            modelInstance.getRenderables(renderables, pool);
         }
      }
   }

   public GDXVRController getController(RobotSide side)
   {
      return controllers.get(side);
   }

   public GDXVRHeadset getHeadset()
   {
      return headset;
   }

   public Collection<GDXVRBaseStation> getBaseStations()
   {
      return baseStations.values();
   }

   public SideDependentList<GDXVREye> getEyes()
   {
      return eyes;
   }

   public ReferenceFrame getOpenVRPlayAreaYUpFrame()
   {
      return vrPlayAreaYUpZBackFrame;
   }

   public ReferenceFrame getTeleportFrameIHMCZUp()
   {
      return teleportFrameIHMCZUp;
   }

   public RigidBodyTransform getTeleportIHMCZUpToIHMCZUpWorld()
   {
      return teleportIHMCZUpToIHMCZUpWorld;
   }

   public void addPickResult(GDXVRPickResult pickResult)
   {
      pickResults.add(pickResult);
   }

   public GDXVRPickResult getSelectedPick()
   {
      return selectedPick;
   }
}