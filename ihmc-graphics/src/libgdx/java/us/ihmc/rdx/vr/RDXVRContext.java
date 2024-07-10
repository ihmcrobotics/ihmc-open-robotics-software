package us.ihmc.rdx.vr;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.BufferUtils;
import com.badlogic.gdx.utils.GdxRuntimeException;
import com.badlogic.gdx.utils.Pool;
import org.lwjgl.opengl.GL41;
import org.lwjgl.openvr.CompositorFrameTiming;
import org.lwjgl.openvr.OpenVR;
import org.lwjgl.openvr.TrackedDevicePose;
import org.lwjgl.openvr.VR;
import org.lwjgl.openvr.VRActiveActionSet;
import org.lwjgl.openvr.VRCompositor;
import org.lwjgl.openvr.VREvent;
import org.lwjgl.openvr.VRInput;
import org.lwjgl.openvr.VRSystem;
import us.ihmc.commons.thread.Notification;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.sceneManager.RDX3DScene;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.JSONTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.io.WorkspaceResourceFile;

import java.nio.IntBuffer;
import java.nio.LongBuffer;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.SortedSet;
import java.util.TreeSet;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

import static org.lwjgl.openvr.VR.VR_ShutdownInternal;
import static org.lwjgl.openvr.VRSystem.VRSystem_GetStringTrackedDeviceProperty;

/**
 * Responsible for initializing the VR system, managing rendering surfaces,
 * getting tracking device poses, submitting the rendering results to the HMD
 * and rendering the surfaces side by side to the companion window on the
 * desktop. Wrapper around OpenVR.
 * <p>
 * FIXME add multisampling plus draw/resolve buffers
 */
public class RDXVRContext
{
   // couple of scratch buffers
   private final IntBuffer errorPointer = BufferUtils.newIntBuffer(1);
   private final IntBuffer widthPointer = BufferUtils.newIntBuffer(1);
   private final IntBuffer heightPointer = BufferUtils.newIntBuffer(1);

   // per eye data such as rendering surfaces, textures, regions, cameras etc. for each eye
   private final SideDependentList<RDXVREye> eyes = new SideDependentList<>();

   // internal native objects to get device poses
   private final AtomicReference<RDXVRTrackedDevicePose[]> trackedDevicePosesParsedRef = new AtomicReference<>();
   private TrackedDevicePose.Buffer trackedDevicePoses;
   private TrackedDevicePose.Buffer trackedDeviceGamePoses;

   // devices, their poses and listeners
   private final LongBuffer mainActionSetHandle = BufferUtils.newLongBuffer(1);
   private VRActiveActionSet.Buffer activeActionSets;

   private VREvent event;
   private int width;
   private int height;
   private final ArrayList<Consumer<RDXVRContext>> vrPickCalculators = new ArrayList<>();
   private final ArrayList<Consumer<RDXVRContext>> vrInputProcessors = new ArrayList<>();
   private final Map<Object, Consumer<RDXVRContext>> vrPickCalculatorOwnerKeyMap = new HashMap<>();
   private final Map<Object, Consumer<RDXVRContext>> vrInputProcessorOwnerKeyMap = new HashMap<>();

   // ReferenceFrame.getWorldFrame() is Z-up frame
   // Finger axis definition is right hand, Thumb +Z, Index +X, Middle +Y
   // The default orientation of the OpenVR frame is such that
   // your thumb is pointing at your face and your index finger pointing right
   // The default orientation of the IHMC Zup frame is such that
   // your thumb is up and your index finger is pointing away from you
   public static final RigidBodyTransformReadOnly openVRYUpToIHMCZUpSpace = new RigidBodyTransform(new YawPitchRoll(
         // For this transformation, we start with IHMC ZUp with index forward and thumb up
         Math.toRadians(-90.0),
         // rotating around thumb, index goes forward to right
         Math.toRadians(0.0),
         // no rotation about middle finger
         Math.toRadians(90.0)
         // rotating about index finger, thumb goes up to toward you
   ), new Point3D());
   /**
    * When the VR player teleports, it adds onto the transform from VR play area frame to world ZUp frame.
    */
   private final RigidBodyTransform teleportIHMCZUpToIHMCZUpWorld = new RigidBodyTransform();
   private final ReferenceFrame teleportFrameIHMCZUp = ReferenceFrameTools.constructFrameWithChangingTransformToParent("teleportFrame",
                                                                                                                       ReferenceFrame.getWorldFrame(),
                                                                                                                       teleportIHMCZUpToIHMCZUpWorld);
   /**
    * The VR play area is on the floor in the center of your VR tracker space area. Also called tracker frame.
    */
   private final ReferenceFrame vrPlayAreaYUpZBackFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("vrPlayAreaFrame",
                                                                                                                            teleportFrameIHMCZUp,
                                                                                                                            openVRYUpToIHMCZUpSpace);
   private RDXVRControllerModel controllerModel = RDXVRControllerModel.UNKNOWN;
   private final RDXVRHeadset headset = new RDXVRHeadset(vrPlayAreaYUpZBackFrame);
   private final SideDependentList<RDXVRController> controllers = new SideDependentList<>(new RDXVRController(RDXVRControllerModel.UNKNOWN,
                                                                                                              RobotSide.LEFT,
                                                                                                              vrPlayAreaYUpZBackFrame),
                                                                                          new RDXVRController(RDXVRControllerModel.UNKNOWN,
                                                                                                              RobotSide.RIGHT,
                                                                                                              vrPlayAreaYUpZBackFrame));
   private final Map<Integer, RDXVRBaseStation> baseStations = new HashMap<>();
   private final Map<String, RDXVRTracker> trackers =  new HashMap<>(); // <serial number, tracker>
   private final Map<String, String> savedTrackersRoleMap = new HashMap<>(); // <role, serial number>
   private final Map<String, String> trackersRoleMap = new HashMap<>(); // <role, serial number>
   private final SortedSet<String> availableTrackerRoles = new TreeSet<>()
   {
      {
         add("Chest");
         add("Waist");
         add("Left Wrist");
         add("Right Wrist");
         add("Left Ankle");
         add("Right Ankle");
      }
   };
   private final List<String> newTrackerSerialNumber = new ArrayList<>();
   private final List<String> removedTrackerSerialNumber = new ArrayList<>();
   private final Notification rolesResetNotification = new Notification();
   private final Notification loadingRolesNotification = new Notification();
   private WorkspaceResourceFile trackerRolesFile;

   public void initSystem()
   {
      LogTools.info("Initializing");

      event = VREvent.create(); // about 92 ms
      trackedDevicePoses = TrackedDevicePose.create(VR.k_unMaxTrackedDeviceCount);
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

      WorkspaceResourceDirectory directory = new WorkspaceResourceDirectory(getClass(), "/vr");
      WorkspaceResourceFile actionManifestFile = new WorkspaceResourceFile(directory, "actions.json");
      JSONFileTools.load(actionManifestFile, node ->
      {
         JSONTools.forEachArrayElement(node, "default_bindings", objectNode ->
         {
            String controllerBindings = objectNode.get("binding_url").asText();
            if (controllerBindings.contains("focus3"))
               controllerModel = RDXVRControllerModel.FOCUS3;
            else
               controllerModel = RDXVRControllerModel.INDEX;
         });
      });
      LogTools.info("Using VR controller model: {}", controllerModel);
      VRInput.VRInput_SetActionManifestPath(actionManifestFile.getFilesystemFile().toString());

      VRInput.VRInput_GetActionSetHandle("/actions/main", mainActionSetHandle);
      headset.initSystem();
      for (RDXVRController controller : controllers)
      {
         controller.setModel(controllerModel);
         controller.initSystem();
      }

      trackerRolesFile = new WorkspaceResourceFile(directory, "tracker_roles.json");
      JSONFileTools.load(trackerRolesFile, node ->
      {
         for (String role : availableTrackerRoles)
         {
            savedTrackersRoleMap.put(role, node.get(role).asText());
         }
      });
      int[] deviceIndices = new int[5]; // maximum number of trackers per dongle
      IntBuffer trackerIndices = IntBuffer.wrap(deviceIndices);
      int numberOfTrackers = VRSystem.VRSystem_GetSortedTrackedDeviceIndicesOfClass(VR.ETrackedDeviceClass_TrackedDeviceClass_GenericTracker,
                                                                                    trackerIndices,
                                                                                    -1);
      for (int i = 0; i < numberOfTrackers; i++)
      {
         int deviceIndex = trackerIndices.get(i);
         if (!trackers.containsKey(getSerialNumber(deviceIndex)))
         {
            trackers.put(getSerialNumber(deviceIndex), new RDXVRTracker(vrPlayAreaYUpZBackFrame, deviceIndex));
         }
      }

      activeActionSets = VRActiveActionSet.create(1);
      activeActionSets.ulActionSet(mainActionSetHandle.get(0));
      activeActionSets.ulRestrictedToDevice(VR.k_ulInvalidInputValueHandle);
   }

   /**
    * Needs to be on libGDX thread.
    */
   public void setupEyes()
   {
      LogTools.info("VR per eye render size: {} x {}", width, height);
      for (RobotSide side : RobotSide.values)
      {
         eyes.set(side, new RDXVREye(side, headset, width, height));
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

   private double openvrInitialTime = 0.0;
   private double systemInitialTime = Double.NaN;
   private CompositorFrameTiming.Buffer latestFrameTimingBuffer = CompositorFrameTiming.create(1);

   { // FIXME Workaround to set the field m_nsize before request timing, without that, the VRCompositor.VRCompositor_GetFrameTimings() will not work. Should submit issue to LWJGL
      getUnsafeInstance().putInt(null, latestFrameTimingBuffer.address() + CompositorFrameTiming.M_NSIZE, latestFrameTimingBuffer.sizeof());
   }

   /**
    * This method waits for OpenVR to say "go" and gathers the latest data.
    * The goal is to submit frames to the eyes ASAP after this returns,
    * then get back to this method before the frame is over.
    */
   public void waitGetPoses()
   {

      VRCompositor.VRCompositor_WaitGetPoses(trackedDevicePoses, trackedDeviceGamePoses);
      int nRead = VRCompositor.VRCompositor_GetFrameTimings(latestFrameTimingBuffer);
      CompositorFrameTiming compositorFrameTiming = latestFrameTimingBuffer.get(0);
      double systemTimeInSeconds = compositorFrameTiming.m_flSystemTimeInSeconds();
      double newPosesReadyMs = compositorFrameTiming.m_flNewPosesReadyMs();
      double openvrTime = systemTimeInSeconds + newPosesReadyMs * 0.001;
      double systemTime = 1.0e-9 * System.nanoTime();
      if (openvrInitialTime == 0.0)
      {
         systemInitialTime = systemTime;
         openvrInitialTime = openvrTime;
      }
      openvrTime -= openvrInitialTime;
      systemTime -= systemInitialTime;

//      LogTools.info(nRead + " - Times: OpenVR=%6.5f, System=%6.5f, Delta=%6.5f".formatted(openvrTime, systemTime, openvrTime - systemTime));

      //      VRCompositor.VRCompositor_GetLastPoses(trackedDevicePoses, trackedDeviceGamePoses); // Is there a way to wait better?
      long measurementTimestamp = (long) (openvrTime * 1.0e9);
      RDXVRTrackedDevicePose[] temp = new RDXVRTrackedDevicePose[VR.k_unMaxTrackedDeviceCount];
      for (int i = 0; i < temp.length; i++)
      {
         temp[i] = new RDXVRTrackedDevicePose(measurementTimestamp, trackedDevicePoses, i);
      }
      trackedDevicePosesParsedRef.set(temp);
   }

   /**
    * For input, see https://github.com/ValveSoftware/openvr/wiki/SteamVR-Input
    * and https://github.com/ValveSoftware/openvr/issues/1151
    */
   public void pollEvents()
   {
      RDXVRTrackedDevicePose[] newTrackedDevicePosesParsed = trackedDevicePosesParsedRef.getAndSet(null);

      if (newTrackedDevicePosesParsed == null)
         return;

      VRInput.VRInput_UpdateActionState(activeActionSets, VRActiveActionSet.SIZEOF);

      headset.update(newTrackedDevicePosesParsed);

      for (RobotSide side : RobotSide.values)
      {
         controllers.get(side).update(newTrackedDevicePosesParsed);
      }

      while (VRSystem.VRSystem_PollNextEvent(event)) // a tracker is connected/disconnected after initialization
      {
         int deviceIndex = event.trackedDeviceIndex();
         int deviceClass = VRSystem.VRSystem_GetTrackedDeviceClass(deviceIndex);
         if (deviceClass == VR.ETrackedDeviceClass_TrackedDeviceClass_GenericTracker)
         {
            if (event.eventType() == VR.EVREventType_VREvent_TrackedDeviceActivated)
            {
               if (!trackers.containsKey(getSerialNumber(deviceIndex)))
               {
                  trackers.put(getSerialNumber(deviceIndex), new RDXVRTracker(vrPlayAreaYUpZBackFrame, deviceIndex));
                  newTrackerSerialNumber.add(getSerialNumber(deviceIndex));
                  LogTools.info("Tracker {} connected", getSerialNumber(deviceIndex));
               }
            }
            else if (event.eventType() == VR.EVREventType_VREvent_TrackedDeviceDeactivated)
            {
               if (trackers.containsKey(getSerialNumber(event.trackedDeviceIndex())))
               {
                  removedTrackerSerialNumber.add(getSerialNumber(deviceIndex));
                  LogTools.warn("Tracker {} disconnected", getSerialNumber(deviceIndex));
               }
            }
         }
      }

      for (var tracker : trackers.entrySet())
      {
         tracker.getValue().update(newTrackedDevicePosesParsed);
      }

      for (Consumer<RDXVRContext> vrPickCalculator : vrPickCalculators)
      {
         vrPickCalculator.accept(this);
      }
      for (RobotSide side : RobotSide.values)
      {
         controllers.get(side).updatePickResults();
      }
      for (Consumer<RDXVRContext> vrInputProcessor : vrInputProcessors)
      {
         vrInputProcessor.accept(this);
      }
   }

   /**
    * Completes rendering and submits the rendering surfaces to the
    * head mounted display.
    */
   public void renderEyes(RDX3DScene scene)
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
      for (RDXVREye eyeData : eyes)
         eyeData.getFrameBuffer().dispose();

      // TODO: Dispose models

      VR_ShutdownInternal();
   }

   public void teleport(Consumer<RigidBodyTransform> teleportIHMCZUpToIHMCZUpWorldConsumer)
   {
      teleportIHMCZUpToIHMCZUpWorldConsumer.accept(teleportIHMCZUpToIHMCZUpWorld);
      teleportFrameIHMCZUp.update();
   }

   public void addVRPickCalculator(Consumer<RDXVRContext> calculateVRPick)
   {
      vrPickCalculators.add(calculateVRPick);
   }

   public void addVRInputProcessor(Consumer<RDXVRContext> processVRInput)
   {
      vrInputProcessors.add(processVRInput);
   }

   public void addVRPickCalculator(Object ownerKey, Consumer<RDXVRContext> calculateVRPick)
   {
      vrPickCalculatorOwnerKeyMap.put(ownerKey, calculateVRPick);
      vrPickCalculators.add(calculateVRPick);
   }

   public void addVRInputProcessor(Object ownerKey, Consumer<RDXVRContext> processVRInput)
   {
      vrInputProcessorOwnerKeyMap.put(ownerKey, processVRInput);
      vrInputProcessors.add(processVRInput);
   }

   public void removeVRPickCalculator(Object ownerKey)
   {
      vrPickCalculators.remove(vrPickCalculatorOwnerKeyMap.remove(ownerKey));
   }

   public void removeVRInputProcessor(Object ownerKey)
   {
      vrInputProcessors.remove(vrInputProcessorOwnerKeyMap.remove(ownerKey));
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
         RDXVRController controller = controllers.get(side);
         if (controller.isConnected())
         {
            controller.getRenderables(renderables, pool);
         }
      }
   }

   public void getTrackerRenderables(String serialNumber, Array<Renderable> renderables, Pool<Renderable> pool)
   {
      ModelInstance modelInstance = trackers.get(serialNumber).getModelInstance();
      if (modelInstance != null)
      {
         modelInstance.getRenderables(renderables, pool);
      }
   }

   public void getTrackersRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (var tracker : trackers.entrySet())
      {
         ModelInstance modelInstance = tracker.getValue().getModelInstance();
         if (modelInstance != null)
         {
            modelInstance.getRenderables(renderables, pool);
         }
      }
   }

   public RDXVRController getController(RobotSide side)
   {
      return controllers.get(side);
   }

   public SideDependentList<RDXVRController> getControllers()
   {
      return controllers;
   }

   public RDXVRHeadset getHeadset()
   {
      return headset;
   }

   private String getSerialNumber(int index)
   {
      return VRSystem_GetStringTrackedDeviceProperty(index, VR.ETrackedDeviceProperty_Prop_SerialNumber_String, null);
   }

   public void setTrackerRole(String role, int index)
   {
      trackersRoleMap.put(role, getSerialNumber(index));
   }

   public RDXVRTracker getTracker(String role)
   {
      return trackers.get(trackersRoleMap.get(role));
   }

   public Map<String, RDXVRTracker> getTrackers()
   {
      return trackers;
   }

   public Set<String> getAssignedTrackerRoles()
   {
      return trackersRoleMap.keySet();
   }

   public SortedSet<String> getAvailableTrackerRoles()
   {
      return availableTrackerRoles;
   }

   public void setTrackerRoleAsUnavailable(String role)
   {
      availableTrackerRoles.remove(role);
   }

   public void setTrackerRoleAsAvailable(String role)
   {
      availableTrackerRoles.add(role);
      trackersRoleMap.remove(role);
   }

   public void resetTrackerRoles()
   {
      trackersRoleMap.clear();
      availableTrackerRoles.clear();
      availableTrackerRoles.add("Chest");
      availableTrackerRoles.add("Waist");
      availableTrackerRoles.add("Left Wrist");
      availableTrackerRoles.add("Right Wrist");
      availableTrackerRoles.add("Left Ankle");
      availableTrackerRoles.add("Right Ankle");
      rolesResetNotification.set();
   }

   public void saveTrackerRolesToFile()
   {
      if (trackerRolesFile.isFileAccessAvailable())
      {
         LogTools.info("Saving tracker roles to file ...");
         JSONFileTools.save(trackerRolesFile, jsonNode ->
         {
            for (String role : savedTrackersRoleMap.keySet())
            {
               jsonNode.put(role, trackersRoleMap.getOrDefault(role, ""));
            }
         });
         LogTools.info("Saved to file");
      }
   }

   public void loadTrackerRolesFromFile()
   {
      LogTools.info("Loading from file tracker roles");
      resetTrackerRoles();
      for (var roleMap : savedTrackersRoleMap.entrySet())
      {
         if (!roleMap.getValue().isEmpty() && trackers.containsKey(roleMap.getValue()))
         {
            trackersRoleMap.put(roleMap.getKey(), roleMap.getValue());
            availableTrackerRoles.remove(roleMap.getKey());
         }
      }
      loadingRolesNotification.set();
   }

   public Notification getRolesResetNotification()
   {
      return rolesResetNotification;
   }

   public Notification getLoadingRolesNotification()
   {
      return loadingRolesNotification;
   }

   public List<String> getNewTrackersSerialNumbers()
   {
      List<String> serialNumbers = new ArrayList<>(newTrackerSerialNumber);
      newTrackerSerialNumber.clear();
      return serialNumbers;
   }

   public List<String> getRemovedTrackersSerialNumbers()
   {
      List<String> serialNumbers = new ArrayList<>(removedTrackerSerialNumber);
      removedTrackerSerialNumber.clear();
      return serialNumbers;
   }

   public Map<String, String> getTrackersRoleMap()
   {
      return trackersRoleMap;
   }

   public Collection<RDXVRBaseStation> getBaseStations()
   {
      return baseStations.values();
   }

   public SideDependentList<RDXVREye> getEyes()
   {
      return eyes;
   }

   public ReferenceFrame getPlayAreaYUpFrame()
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

   public RDXVRControllerModel getControllerModel()
   {
      return controllerModel;
   }

   private static sun.misc.Unsafe getUnsafeInstance()
   {
      java.lang.reflect.Field[] fields = sun.misc.Unsafe.class.getDeclaredFields();

        /*
        Different runtimes use different names for the Unsafe singleton,
        so we cannot use .getDeclaredField and we scan instead. For example:

        Oracle: theUnsafe
        PERC : m_unsafe_instance
        Android: THE_ONE
        */
      for (java.lang.reflect.Field field : fields)
      {
         if (!field.getType().equals(sun.misc.Unsafe.class))
         {
            continue;
         }

         int modifiers = field.getModifiers();
         if (!(java.lang.reflect.Modifier.isStatic(modifiers) && java.lang.reflect.Modifier.isFinal(modifiers)))
         {
            continue;
         }

         try
         {
            field.setAccessible(true);
            return (sun.misc.Unsafe) field.get(null);
         }
         catch (Exception ignored)
         {
         }
         break;
      }

      throw new UnsupportedOperationException("LWJGL requires sun.misc.Unsafe to be available.");
   }
}