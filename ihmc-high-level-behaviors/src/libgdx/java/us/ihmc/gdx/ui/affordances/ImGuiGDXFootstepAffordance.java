package us.ihmc.gdx.ui.affordances;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.flag.ImGuiMouseButton;
import imgui.flag.ImGuiStyleVar;
import imgui.internal.ImGui;
import imgui.internal.flag.ImGuiItemFlags;
import imgui.type.ImFloat;
import org.lwjgl.openvr.InputDigitalActionData;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.imgui.ImGuiLabelMap;
import us.ihmc.gdx.input.editor.GDXUIActionMap;
import us.ihmc.gdx.input.editor.GDXUITrigger;
import us.ihmc.gdx.simulation.environment.GDXModelInstance;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.vr.GDXVRManager;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.function.Consumer;

public class ImGuiGDXFootstepAffordance implements RenderableProvider
{
   private final ImGuiLabelMap labels = new ImGuiLabelMap();
   private final ImFloat goalZOffset = new ImFloat(0.0f);
   //private ModelInstance footstepModel;

   private GDXUIActionMap placeGoalActionMap;
   private boolean placingGoal = false;
   private boolean placingPosition = true;
   private Point3D lastObjectIntersection;
   private final Pose3D goalPoseForReading = new Pose3D();
   private final Point3D32 tempSpherePosition = new Point3D32();
   private final Vector3D32 tempRotationVector = new Vector3D32();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private PlanarRegionsList latestRegions;
   private Consumer<Pose3D> placedPoseConsumer;

   ReferenceFrame referenceFrameFootstep;
   FramePose3D footTextPose;
   private GDXModelInstance footstepModel;
   private float textheight = 12;

   public ArrayList<SingleFootstep> getFootstepArrayList()
   {
      return footstepArrayList;
   }

   ArrayList<SingleFootstep> footstepArrayList = new ArrayList<>();

   public void create(GDXImGuiBasedUI baseUI, Consumer<Pose3D> placedPoseConsumer, Color color)
   {
      this.placedPoseConsumer = placedPoseConsumer;

      placeGoalActionMap = new GDXUIActionMap(startAction ->
                                              {
                                                 placingGoal = true;
                                                 placingPosition = true;
                                              });
      placeGoalActionMap.mapAction(GDXUITrigger.POSITION_LEFT_CLICK, trigger ->
      {
         placingPosition = false;
      });

      placeGoalActionMap.mapAction(GDXUITrigger.RIGHT_CLICK, trigger ->
      {
         placingGoal = false;
      });

      clear();
   }

   public void addFootStep(SingleFootstep.FootstepSide Side)
   {

   }

   public void processImGui3DViewInput(ImGui3DViewInput input)
   {
      if (placingGoal && input.isWindowHovered())
      {
         Line3DReadOnly pickRayInWorld = input.getPickRayInWorld();
         PlanarRegionsList latestRegions = this.latestRegions;
         Point3D pickPoint = null;
         if (latestRegions != null)
         {
            for (PlanarRegion planarRegion : latestRegions.getPlanarRegionsAsList())
            {
               Point3D intersection = PlanarRegionTools.intersectRegionWithRay(planarRegion, pickRayInWorld.getPoint(), pickRayInWorld.getDirection());
               if (intersection != null)
               {
                  if (pickPoint == null)
                  {
                     pickPoint = intersection;
                     System.out.println("pickPoint " + pickPoint.toString());
                  }
                  else
                  {
                     if (intersection.distance(pickRayInWorld.getPoint()) < pickPoint.distance(pickRayInWorld.getPoint()))
                     {
                        pickPoint = intersection;
                        System.out.println("pickPoint " + pickPoint.toString());
                     }
                  }
                  lastObjectIntersection = pickPoint;
               }
            }
         }

         if (pickPoint == null)
         {
            pickPoint = EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(lastObjectIntersection != null
                                                                                      ? lastObjectIntersection : EuclidCoreTools.origin3D,
                                                                                Axis3D.Z,
                                                                                pickRayInWorld.getPoint(),
                                                                                pickRayInWorld.getDirection());
         }

         double z = (lastObjectIntersection != null ? lastObjectIntersection.getZ() : 0.0) + goalZOffset.get();
         if (placingPosition)
         {
            if (ImGui.getIO().getKeyCtrl())
            {
               goalZOffset.set(goalZOffset.get() - (input.getMouseWheelDelta() / 30.0f));
            }

            footstepModel.transform.setTranslation(pickPoint.getX32(), pickPoint.getY32(), (float) z);

            // when left button clicked and released.
            if (input.mouseReleasedWithoutDrag(ImGuiMouseButton.Left))
            {
               placeGoalActionMap.triggerAction(GDXUITrigger.POSITION_LEFT_CLICK);
               placingGoal = false;
               placingPosition = true;
            }
         }

         if (input.mouseReleasedWithoutDrag(ImGuiMouseButton.Right))
         {
            placeGoalActionMap.triggerAction(GDXUITrigger.RIGHT_CLICK);
         }
      }
   }

   public void handleVREvents(GDXVRManager vrManager)
   {
      vrManager.getContext().getController(RobotSide.LEFT).runIfConnected(controller ->
      {
         InputDigitalActionData triggerClick = controller.getClickTriggerActionData();
         if (triggerClick.bChanged() && triggerClick.bState())
         {
            placingGoal = true;
         }
         if (triggerClick.bChanged() && !triggerClick.bState())
         {
            placingGoal = false;
            placedPoseConsumer.accept(goalPoseForReading);
         }

         controller.getTransformZUpToWorld(footstepModel.transform);

      });
   }

   public void renderFootStepModeButton()
   {
      boolean pushedFlags = false;
      if (placingGoal)
      {
         ImGui.pushItemFlag(ImGuiItemFlags.Disabled, true);
         ImGui.pushStyleVar(ImGuiStyleVar.Alpha, 0.6f);
         pushedFlags = true;
      }
      if (ImGui.button(labels.get(pushedFlags ? "Placing" : "Place goal")))
      {
         placeGoalActionMap.start();
      }
      if (pushedFlags)
      {
         ImGui.popItemFlag();
         ImGui.popStyleVar();
      }
      if (ImGui.isItemHovered())
      {
         ImGui.setTooltip("Hold Ctrl and scroll the mouse wheel while placing to adjust Z.");
      }
      ImGui.sameLine();
      if (!isPlaced())
      {
         ImGui.text("Not placed.");
      }
      else
      {
         if (ImGui.button(labels.get("Clear")))
         {
            clear();
         }
         ImGui.sameLine();
         ImGui.pushItemWidth(50.0f);
         ImGui.dragFloat("Goal Z Offset", goalZOffset.getData(), 0.01f);
         ImGui.popItemWidth();
      }


   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (isPlaced())
      {
         footstepModel.getRenderables(renderables, pool);

      }
   }

   public boolean isPlaced()
   {
      return !Float.isNaN(footstepModel.transform.val[Matrix4.M03]);
   }

   public boolean getPlacingGoal()
   {
      return placingGoal;
   }

   public void setPlacingGoal(boolean placingGoal)
   {
      this.placingGoal = placingGoal;
   }

   public void clear()
   {
      placingGoal = false;
      placingPosition = true;
      if (footstepModel != null)
         footstepModel.transform.val[Matrix4.M03] = Float.NaN;
      goalZOffset.set(0.0f);
   }

   public void setLatestRegions(PlanarRegionsList latestRegions)
   {
      this.latestRegions = latestRegions;
   }

   public Pose3DReadOnly getGoalPose()
   {
      return goalPoseForReading;
   }

   public void setGoalPoseAndPassOn(Pose3DReadOnly pose)
   {
      setGoalPoseNoCallbacks(pose);
      placedPoseConsumer.accept(goalPoseForReading);
   }

   public void setGoalPoseNoCallbacks(Pose3DReadOnly pose)
   {
      if (pose == null)
      {
         clear();
      }
      else
      {
         GDXTools.toGDX(pose.getPosition(), footstepModel.transform);
         goalZOffset.set((float) pose.getZ());
      }
      goalPoseForReading.set(pose);
   }

   private void createNewFootStep(GDXImGuiBasedUI baseUI, SingleFootstep.FootstepSide footstepSide)
   {
      footstepArrayList.add(new SingleFootstep(baseUI.getPrimaryScene(),footstepSide));
   }

}
