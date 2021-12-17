package us.ihmc.gdx.simulation.environment;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.flag.ImGuiInputTextFlags;
import imgui.internal.ImGui;
import imgui.internal.flag.ImGuiItemFlags;
import imgui.type.ImString;
import org.lwjgl.openvr.InputDigitalActionData;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.simulation.environment.object.objects.*;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.vr.GDXVRContext;
import us.ihmc.gdx.vr.GDXVRManager;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;

public class GDXEnvironmentBuilderPanel implements RenderableProvider
{
   private final static String WINDOW_NAME = "Environment Builder";
   private final ImString loadString = new ImString("terrain.yml", 100);
   private final ImString saveString = new ImString("terrain.yml", 100);

   private GDXEnvironmentObject modelBeingPlaced;
   private final GDXModelInput modelInput = new GDXModelInput();
   private final ArrayList<GDXEnvironmentObject> environmentObjects = new ArrayList<>();

//   private final GDXPose3DGizmo pose3DWidget = new GDXPose3DGizmo();

   public void create(GDXImGuiBasedUI baseUI)
   {
      GDXVRManager vrManager = baseUI.getVRManager();
      vrManager.getContext().addVRInputProcessor(this::handleVREvents);

      modelInput.create();

//      pose3DWidget.create(baseUI);
//      baseUI.addImGui3DViewInputProcessor(pose3DWidget::process3DViewInput);
//      baseUI.getSceneManager().addRenderableProvider(pose3DWidget);

      baseUI.addImGui3DViewInputProcessor(this::processImGui3DViewInput);
   }

   private void processImGui3DViewInput(ImGui3DViewInput input)
   {
      modelInput.updateSelections(input);
      modelInput.updateState(input);
      modelInput.updatePoseForSelections(input);
   }

   public void handleVREvents(GDXVRContext vrContext)
   {
      vrContext.getController(RobotSide.LEFT).runIfConnected(controller ->
      {
         InputDigitalActionData triggerClick = controller.getClickTriggerActionData();
         if (triggerClick.bChanged() && triggerClick.bState())
         {
            modelInput.clear();
         }
      });
      vrContext.getController(RobotSide.RIGHT).runIfConnected(controller ->
      {
         InputDigitalActionData triggerClick = controller.getClickTriggerActionData();
         if (triggerClick.bChanged() && triggerClick.bState())
         {
            modelBeingPlaced = new GDXLargeCinderBlockRoughed();
            modelInput.addAndSelectInstance(modelBeingPlaced);
            controller.getTransformZUpToWorld(modelBeingPlaced.getRealisticModelInstance().transform);
         }
         if (triggerClick.bChanged() && !triggerClick.bState())
         {
            modelBeingPlaced = null;
         }
      });
   }

   public void renderImGuiWindow()
   {
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
      boolean placeNewObject;
      if (placeNewObject = ImGui.button("Place Large Cinder Block"))
      {
         modelBeingPlaced = new GDXLargeCinderBlockRoughed();
      }
      else if (placeNewObject = ImGui.button("Place Medium Cinder Block"))
      {
         modelBeingPlaced = new GDXMediumCinderBlockRoughed();
      }
      else if (placeNewObject = ImGui.button("Place Small Cinder Block"))
      {
         modelBeingPlaced = new GDXSmallCinderBlockRoughed();
      }
      else if (placeNewObject = ImGui.button("Place Door Frame"))
      {
         modelBeingPlaced = new GDXDoorFrameObject();
      }
      else if (placeNewObject = ImGui.button("Place Door Only"))
      {
         modelBeingPlaced = new GDXPushHandleRightDoorObject();
      }
      else if (placeNewObject = ImGui.button("Place Floor"))
      {
         modelBeingPlaced = new GDXLabFloorObject();
      }
      if (placeNewObject)
      {
         modelInput.setState(GDXModelInput.State.PLACING_XY);
         modelInput.addAndSelectInstance(modelBeingPlaced);
      }
      if (pushed)
      {
         ImGui.popItemFlag();
      }

      modelInput.renderImGuiPanel();

//      pose3DWidget.render();
   }

   public String getWindowName()
   {
      return WINDOW_NAME;
   }

   public ModelInstance placeFloor()
   {
      GDXLabFloorObject floor = new GDXLabFloorObject();
      Pose3D pose = new Pose3D();
      RigidBodyTransform transform = new RigidBodyTransform();
      pose.set(new Point3D(0.0f, 0.0f, 0.0f), new YawPitchRoll(0.0, 0.0, Math.toRadians(90.0)));
      pose.get(transform);
      GDXTools.toGDX(transform, floor.getRealisticModelInstance().transform);
      return floor.getRealisticModelInstance();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (GDXEnvironmentObject placedModel : modelInput.getEnvironmentObjects())
      {
         placedModel.getRealisticModelInstance().getRenderables(renderables, pool);
      }

      for(ModelInstance controlAxis : modelInput.getControlAxes())
      {
         controlAxis.getRenderables(renderables, pool);
      }
   }

   public GDXModelInput getModelInput()
   {
      return modelInput;
   }
}
