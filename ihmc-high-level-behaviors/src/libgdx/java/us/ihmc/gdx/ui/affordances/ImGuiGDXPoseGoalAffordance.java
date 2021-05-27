package us.ihmc.gdx.ui.affordances;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.flag.ImGuiMouseButton;
import imgui.internal.ImGui;
import imgui.internal.flag.ImGuiItemFlags;
import imgui.type.ImFloat;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.gdx.imgui.ImGui3DViewInput;
import us.ihmc.gdx.imgui.ImGuiLabelMap;
import us.ihmc.gdx.input.editor.GDXUIActionMap;
import us.ihmc.gdx.input.editor.GDXUITrigger;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.function.Consumer;

public class ImGuiGDXPoseGoalAffordance implements RenderableProvider
{
   private final ImGuiLabelMap labels = new ImGuiLabelMap();
   private final ImFloat goalZ = new ImFloat(0.0f);
   private ModelInstance sphere;
   private ModelInstance arrow;
   private GDXUIActionMap placeGoalActionMap;
   private boolean placingGoal = false;
   private boolean placingPosition = true;
   private final Pose3D goalPose = new Pose3D();
   private final Point3D32 spherePosition = new Point3D32();
   private final Vector3D32 rotationVector = new Vector3D32();
   private final RotationMatrix arrowRotationMatrix = new RotationMatrix();
   private PlanarRegionsList latestRegions;

   public void create(Consumer<Pose3D> placedPoseConsumer)
   {
      float sphereRadius = 0.03f;
      sphere = GDXModelPrimitives.createSphere(sphereRadius, Color.CYAN);
      arrow = GDXModelPrimitives.createArrow(sphereRadius * 6.0, Color.CYAN);

      placeGoalActionMap = new GDXUIActionMap(startAction ->
                                              {
                                                 placingGoal = true;
                                                 placingPosition = true;
                                              });
      placeGoalActionMap.mapAction(GDXUITrigger.POSITION_LEFT_CLICK, trigger ->
      {
         placingPosition = false;
      });
      placeGoalActionMap.mapAction(GDXUITrigger.ORIENTATION_LEFT_CLICK, trigger ->
      {
         placedPoseConsumer.accept(goalPose);

         placingGoal = false;
      });
      placeGoalActionMap.mapAction(GDXUITrigger.RIGHT_CLICK, trigger ->
      {
         placingGoal = false;
      });
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
                  }
                  else
                  {
                     if (intersection.distance(pickRayInWorld.getPoint()) < pickPoint.distance(pickRayInWorld.getPoint()))
                     {
                        pickPoint = intersection;
                     }
                  }
               }
            }
         }

         if (pickPoint == null)
         {
            pickPoint = EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(EuclidCoreTools.origin3D,
                                                                                Axis3D.Z,
                                                                                pickRayInWorld.getPoint(),
                                                                                pickRayInWorld.getDirection());
         }

         if (placingPosition)
         {
            if (ImGui.getIO().getKeyCtrl())
            {
               goalZ.set(goalZ.get() - (input.getMouseWheelDelta() / 30.0f));
            }

            sphere.transform.setTranslation(pickPoint.getX32(), pickPoint.getY32(), (float) goalZ.get());

            if (input.mouseReleasedWithoutDrag(ImGuiMouseButton.Left))
            {
               placeGoalActionMap.triggerAction(GDXUITrigger.POSITION_LEFT_CLICK);
            }
         }
         else // placing orientation
         {
            GDXTools.toEuclid(sphere.transform, spherePosition);
            spherePosition.setZ(goalZ.get());
            GDXTools.toGDX(spherePosition, arrow.transform);

            rotationVector.set(pickPoint);
            rotationVector.sub(spherePosition);

            double yaw = Math.atan2(rotationVector.getY(), rotationVector.getX());
            arrowRotationMatrix.setToYawOrientation(yaw);
            GDXTools.toGDX(arrowRotationMatrix, arrow.transform);

            goalPose.set(spherePosition, arrowRotationMatrix);

            if (input.mouseReleasedWithoutDrag(ImGuiMouseButton.Left))
            {
               placeGoalActionMap.triggerAction(GDXUITrigger.ORIENTATION_LEFT_CLICK);
            }
         }

         if (input.mouseReleasedWithoutDrag(ImGuiMouseButton.Right))
         {
            placeGoalActionMap.triggerAction(GDXUITrigger.RIGHT_CLICK);
         }
      }
   }

   public void renderPlaceGoalButton()
   {
      boolean pushed = false;
      if (placingGoal)
      {
         pushed = true;
         ImGui.pushItemFlag(ImGuiItemFlags.Disabled, true);
      }
      if (ImGui.button(labels.get("Place goal")))
      {
         placeGoalActionMap.start();
      }
      if (pushed)
      {
         ImGui.popItemFlag();
      }
      if (ImGui.isItemHovered())
      {
         ImGui.setTooltip("Hold Ctrl and scroll the mouse wheel while placing to adjust Z.");
      }
      ImGui.sameLine();
      ImGui.pushItemWidth(50.0f);
      ImGui.dragFloat("Goal Z", goalZ.getData(), 0.01f);
      ImGui.popItemWidth();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      sphere.getRenderables(renderables, pool);
      arrow.getRenderables(renderables, pool);
   }

   public void setLatestRegions(PlanarRegionsList latestRegions)
   {
      this.latestRegions = latestRegions;
   }
}
