package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.input.ImGui3DViewPickResult;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.affordances.RDXSelectablePose3DGizmo;
import us.ihmc.rdx.visualizers.RDXPlanarRegionGraphic;
import us.ihmc.robotics.geometry.PlanarRegion;

import java.util.Set;

public class RDXEditablePlanarRegion
{
   private PlanarRegion planarRegion = new PlanarRegion();
   private RDXSelectablePose3DGizmo originGizmo = new RDXSelectablePose3DGizmo();
   private Point3D mouseRegionIntersection = new Point3D();
   private RDXPlanarRegionGraphic planarRegionGraphic;
   private final ImGui3DViewPickResult pickResult = new ImGui3DViewPickResult();

   public RDXEditablePlanarRegion(RDX3DPanel panel3D)
   {
      originGizmo.createAndSetupDefault(panel3D);
//      originGizmo.
      planarRegionGraphic = new RDXPlanarRegionGraphic(Color.GREEN);
//      planarRegionsGraphic.
   }

   public void update()
   {
//      planarRegionGraphic.
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      originGizmo.calculate3DViewPick(input);

      boolean lineIsARay = true;
      boolean intersects = planarRegion.intersectWithLine(input.getPickRayInWorld(), mouseRegionIntersection, lineIsARay);

      if (intersects)
      {
         pickResult.setDistanceToCamera(mouseRegionIntersection.distance(input.getPickRayInWorld().getPoint()));
         input.addPickResult(pickResult);
      }
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      originGizmo.getSelected().set(input.getClosestPick() == pickResult);

      originGizmo.process3DViewInput(input);
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (sceneLevels.contains(RDXSceneLevel.VIRTUAL))
      {
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
