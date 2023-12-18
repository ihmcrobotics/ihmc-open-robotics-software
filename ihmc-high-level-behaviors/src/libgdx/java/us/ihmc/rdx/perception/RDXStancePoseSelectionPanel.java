package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiMouseButton;
import us.ihmc.behaviors.activeMapping.StancePoseCalculator;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.graphics.RDXFootstepGraphic;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;

public class RDXStancePoseSelectionPanel implements RenderableProvider
{
   private ModelInstance pickPointSphere;
   private final FramePose3D latestPose = new FramePose3D(ReferenceFrame.getWorldFrame());
   private final SideDependentList<RDXFootstepGraphic> footstepGraphics;

   private StancePoseCalculator stancePoseCalculator;
   private TerrainMapData terrainMapData;
   private ImGui3DViewInput latestInput;

   private boolean selectionActive = false;

   public RDXStancePoseSelectionPanel(StancePoseCalculator stancePoseCalculator)
   {
      this.stancePoseCalculator = stancePoseCalculator;

      SegmentDependentList<RobotSide, ArrayList<Point2D>> contactPoints = new SideDependentList<>();
      contactPoints.set(RobotSide.LEFT, PlannerTools.createFootContactPoints(0.2, 0.1, 0.08));
      contactPoints.set(RobotSide.RIGHT, PlannerTools.createFootContactPoints(0.2, 0.1, 0.08));

      footstepGraphics = new SideDependentList<>(new RDXFootstepGraphic(contactPoints, RobotSide.LEFT), new RDXFootstepGraphic(contactPoints, RobotSide.RIGHT));

      footstepGraphics.get(RobotSide.LEFT).create();
      footstepGraphics.get(RobotSide.RIGHT).create();

      pickPointSphere = RDXModelBuilder.createSphere(0.05f, Color.CYAN);
   }

   public void update(FramePose3D goalPose, TerrainMapData terrainMapData)
   {
      this.terrainMapData = terrainMapData;

      //SideDependentList<FramePose3D> poses = stancePoseCalculator.getStancePoses(goalPose, terrainMapData);
      //footstepGraphics.get(RobotSide.LEFT).setPose(poses.get(RobotSide.LEFT));
      //footstepGraphics.get(RobotSide.RIGHT).setPose(poses.get(RobotSide.RIGHT));
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      footstepGraphics.get(RobotSide.LEFT).getRenderables(renderables, pool);
      footstepGraphics.get(RobotSide.RIGHT).getRenderables(renderables, pool);

      if (selectionActive)
      {
         pickPointSphere.getRenderables(renderables, pool);
      }
   }

   public void renderImGuiWidgets()
   {
      boolean panel3DIsHovered = latestInput != null && latestInput.isWindowHovered();
      if (panel3DIsHovered && ImGui.isKeyPressed('P'))
      {
         selectionActive = true;
      }
      if (ImGui.isKeyPressed(ImGuiTools.getEscapeKey()))
      {
         selectionActive = false;
      }
   }

   public void processImGui3DViewInput(ImGui3DViewInput input)
   {
      latestInput = input;

      Point3DReadOnly pickPointInWorld = input.getGroundPickPointInWorld(0.0f);
      latestPose.getTranslation().set(pickPointInWorld);

      if (terrainMapData != null)
      {
         double height = terrainMapData.getHeightInWorld(pickPointInWorld.getX32(), pickPointInWorld.getY32());
         latestPose.getTranslation().setZ(height);
      }

      LibGDXTools.toLibGDX(latestPose.getPosition(), pickPointSphere.transform);

      // Adjust footstep yaw while placing with Ctrl + Mouse Scroll Up/Down
      double deltaYaw = 0.0;
      boolean ctrlHeld = ImGui.getIO().getKeyCtrl();
      if (ctrlHeld)
      {
         float dScroll = input.getMouseWheelDelta();
         if (dScroll > 0.0)
         {
            deltaYaw = 0.03 * Math.PI;
         }
         else if (dScroll < 0.0)
         {
            deltaYaw = -0.03 * Math.PI;
         }
         if (deltaYaw != 0.0)
         {
            double latestFootstepYaw = latestPose.getRotation().getYaw();
            latestPose.getOrientation().setToYawOrientation(latestFootstepYaw + deltaYaw);
         }
      }

      if (input.isWindowHovered() & input.mouseReleasedWithoutDrag(ImGuiMouseButton.Left))
      {
         placeFootstep();
         selectionActive = false;
      }

      if (input.isWindowHovered() && input.mouseReleasedWithoutDrag(ImGuiMouseButton.Right))
      {
         selectionActive = false;
      }
   }

   private void placeFootstep()
   {

   }
}
