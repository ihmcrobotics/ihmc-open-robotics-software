package us.ihmc.gdx.simulation.environment;

import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.flag.ImGuiInputTextFlags;
import imgui.internal.ImGui;
import imgui.internal.flag.ImGuiItemFlags;
import imgui.type.ImBoolean;
import imgui.type.ImString;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.imgui.ImGui3DViewInput;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.tools.GDXModelLoader;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.vr.GDXVRContext;
import us.ihmc.gdx.vr.GDXVRManager;
import us.ihmc.gdx.vr.VRDeviceAdapter;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

import static us.ihmc.gdx.vr.GDXVRContext.VRControllerButtons.SteamVR_Trigger;

public class GDXEnvironmentBuilderPanel implements RenderableProvider
{
   private static String WINDOW_NAME = "Environment Builder";
   private ImString loadString = new ImString("terrain.yml", 100);
   private ImString saveString = new ImString("terrain.yml", 100);

   private GDXImGuiBasedUI baseUI;
   private GDXVRManager vrManager;

   private Model labFloorModel;
   private Model smallCinderBlockModel;
   private Model mediumCinderBlockModel;
   private Model largeCinderBlockModel;
   private Model sensorL515Model;
   private Model doorModel;
   private Model doorFrameModel;

   private FramePose3D tempFramePose = new FramePose3D();
   private RigidBodyTransform tempRigidBodyTransform = new RigidBodyTransform();

   private GDXModelInstance sensorModelInstance;

   private GDXModelInstance modelBeingPlaced;
   private final GDXModelInput modelInput = new GDXModelInput();
   private final ImBoolean editModeChecked = new ImBoolean(false);
   public void create(GDXImGuiBasedUI baseUI)
   {
      this.baseUI = baseUI;
      vrManager = baseUI.getVRManager();

      modelInput.setBaseUI(baseUI);

      labFloorModel = GDXModelLoader.loadG3DModel("labFloor.g3dj");
      smallCinderBlockModel = GDXModelLoader.loadG3DModel("SmallCinderBlockRough.g3dj");
      mediumCinderBlockModel = GDXModelLoader.loadG3DModel("MediumCinderBlockRough.g3dj");
      largeCinderBlockModel = GDXModelLoader.loadG3DModel("LargeCinderBlockRough.g3dj");
      doorModel = GDXModelLoader.loadG3DModel("DoorOnly.g3dj");
      doorFrameModel = GDXModelLoader.loadG3DModel("DoorFrame.g3dj");
      sensorL515Model = GDXModelLoader.loadG3DModel("sensor_l515.g3dj");

      modelInput.create();

      sensorModelInstance = new GDXModelInstance(sensorL515Model);
      modelInput.addInstance(sensorModelInstance);

      if (vrManager.isVREnabled())
      {
         vrManager.getContext().addListener(new VRDeviceAdapter()
         {
            @Override
            public void buttonPressed(GDXVRContext.VRDevice device, int button)
            {
               LogTools.info("Pressed: {}, {}", device, button);
               if (device == vrManager.getControllers().get(RobotSide.RIGHT) && button == SteamVR_Trigger)
               {
                  modelBeingPlaced = new GDXModelInstance(largeCinderBlockModel);
                  modelInput.addAndSelectInstance(modelBeingPlaced);
               }
               if (device == vrManager.getControllers().get(RobotSide.LEFT) && button == SteamVR_Trigger)
               {
                  modelInput.clear();
               }
            }

            @Override
            public void buttonReleased(GDXVRContext.VRDevice device, int button)
            {
               LogTools.info("Released: {}, {}", device, button);
               if (device == vrManager.getControllers().get(RobotSide.RIGHT) && button == SteamVR_Trigger)
               {
                  modelBeingPlaced = null;
               }
            }
         });
      }

      baseUI.addImGui3DViewInputProcessor(this::processImGui3DViewInput);
   }

   public void processImGui3DViewInput(ImGui3DViewInput input)
   {
      modelInput.updateSelections(input);
      modelInput.updateState(input);
      modelInput.updatePoseForSelections(input);
   }

   public void handleVREvents()
   {
      if (vrManager.isVREnabled() && modelBeingPlaced != null)
      {
         PoseReferenceFrame controllerFrame = vrManager.getControllers().get(RobotSide.RIGHT).getReferenceFrame();
         tempFramePose.setToZero(controllerFrame);
         tempFramePose.changeFrame(ReferenceFrame.getWorldFrame());
         tempFramePose.get(tempRigidBodyTransform);
         GDXTools.toGDX(tempRigidBodyTransform, modelBeingPlaced.transform);
      }
   }

   public void renderImGuiWindow()
   {
      ImGui.begin(WINDOW_NAME);

      int flags = ImGuiInputTextFlags.None;
      flags += ImGuiInputTextFlags.CallbackResize;
      ImGui.inputText("###loadText", loadString, flags);
      ImGui.sameLine();
      if (ImGui.button("Load"))
      {

      }

      ImGui.inputText("###saveText", saveString, flags);
      ImGui.sameLine();
      if (ImGui.button("Save"))
      {

      }

      boolean pushed = false;
      if (!modelInput.isDone())
      {
         pushed = true;
         ImGui.pushItemFlag(ImGuiItemFlags.Disabled, true);
      }
      if (ImGui.button("Place Cinder Block"))
      {
         modelInput.setState(GDXModelInput.State.PLACING_XY);
         modelBeingPlaced = new GDXModelInstance(largeCinderBlockModel);
         modelInput.addAndSelectInstance(modelBeingPlaced);
      }
      if (pushed)
      {
         ImGui.popItemFlag();
      }

      ImGui.checkbox("Edit Mode", editModeChecked);
      modelInput.setEditMode(editModeChecked.get());

      ImGui.end();

      ImGui.begin(ImGuiTools.uniqueLabel(this, "Planar Region Data Sets"));

//      ImGui.list

      ImGui.end();
   }

   public String getWindowName()
   {
      return WINDOW_NAME;
   }

   public GDXModelInstance getSensorModelInstance()
   {
      return sensorModelInstance;
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (GDXModelInstance placedModel : modelInput.getInstances())
      {
         placedModel.getRenderables(renderables, pool);
      }

      for(ModelInstance controlAxis : modelInput.getControlAxes())
      {
         controlAxis.getRenderables(renderables, pool);
      }
   }
}
