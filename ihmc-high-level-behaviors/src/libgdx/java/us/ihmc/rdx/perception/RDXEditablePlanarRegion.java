package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiMouseButton;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.input.ImGui3DViewPickResult;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.affordances.RDXSelectablePose3DGizmo;
import us.ihmc.rdx.visualizers.RDXPlanarRegionGraphic;
import us.ihmc.robotics.geometry.PlanarRegion;

import java.util.ArrayList;
import java.util.Set;

public class RDXEditablePlanarRegion
{
   private PlanarRegion planarRegion = new PlanarRegion();
   private boolean planarRegionChanged = false;
   private RDXSelectablePose3DGizmo originGizmo = new RDXSelectablePose3DGizmo();
   private Point3D mouseRegionIntersection = new Point3D();
   private RDXPlanarRegionGraphic planarRegionGraphic;
   private final ImGui3DViewPickResult pickResult = new ImGui3DViewPickResult();
   private final ArrayList<RDXEditablePlanarRegionVertex> editableVertices = new ArrayList<>();

   public RDXEditablePlanarRegion(RDX3DPanel panel3D)
   {
      originGizmo.createAndSetupDefault(panel3D);
//      originGizmo.
      planarRegionGraphic = new RDXPlanarRegionGraphic(Color.GREEN);
//      planarRegionsGraphic.
   }

   public void addVertex()
   {
      RDXEditablePlanarRegionVertex vertex = new RDXEditablePlanarRegionVertex();
      FramePoint3D vertexInitialPoint = new FramePoint3D(originGizmo.getPoseGizmo().getGizmoFrame());
      vertexInitialPoint.setX(0.4);
      vertexInitialPoint.changeFrame(ReferenceFrame.getWorldFrame());
      vertex.setPositionInWorld(vertexInitialPoint);
      addVertex(vertex);
   }

   public void addVertex(RDXEditablePlanarRegionVertex vertex)
   {
      editableVertices.add(vertex);
   }

   public void update()
   {
      for (RDXEditablePlanarRegionVertex editableVertex : editableVertices)
      {
         editableVertex.update();
      }

      if (planarRegionChanged)
      {
         planarRegionGraphic.generateMesh(planarRegion);
      }

      if (planarRegionGraphic.getModelInstance() != null)
      {
         planarRegionGraphic.getModelInstance().setPoseInWorldFrame(originGizmo.getPoseGizmo().getPose());
      }
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      for (RDXEditablePlanarRegionVertex editableVertex : editableVertices)
      {
         editableVertex.calculate3DViewPick(input);
      }

      originGizmo.calculate3DViewPick(input);

      if (planarRegion.getConcaveHullSize() >= 3)
      {
         boolean lineIsARay = true;
         boolean intersects = planarRegion.intersectWithLine(input.getPickRayInWorld(), mouseRegionIntersection, lineIsARay);

         if (intersects)
         {
            pickResult.setDistanceToCamera(mouseRegionIntersection.distance(input.getPickRayInWorld().getPoint()));
            input.addPickResult(pickResult);
         }
      }
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      for (RDXEditablePlanarRegionVertex editableVertex : editableVertices)
      {
         editableVertex.process3DViewInput(input);

         if (editableVertex.isHovered() && ImGui.isMouseDown(ImGuiMouseButton.Left))
         {
            editableVertex.setPositionInWorld(positionToPack -> planarRegion.getPlane().intersectionWith(input.getPickRayInWorld(), positionToPack));
         }
      }

//      originGizmo.getSelected().set(input.getClosestPick() == pickResult);

      originGizmo.process3DViewInput(input);
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (sceneLevels.contains(RDXSceneLevel.VIRTUAL))
      {
         for (RDXEditablePlanarRegionVertex editableVertex : editableVertices)
         {
            editableVertex.getRenderables(renderables, pool, sceneLevels);
         }

         originGizmo.getVirtualRenderables(renderables, pool);
      }
      if (sceneLevels.contains(RDXSceneLevel.MODEL))
      {
         planarRegionGraphic.getRenderables(renderables, pool);
      }
   }

   public RDXSelectablePose3DGizmo getOriginGizmo()
   {
      return originGizmo;
   }
}
