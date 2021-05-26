package us.ihmc.gdx.simulation.environment;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import us.ihmc.commons.nio.BasicPathVisitor;
import us.ihmc.commons.nio.PathTools;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.visualizers.GDXPlanarRegionsGraphic;
import us.ihmc.robotics.PlanarRegionFileTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.nio.file.FileVisitResult;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;

public class GDXPlanarRegionEnvironmentPanel implements RenderableProvider
{
   private final String windowName = ImGuiTools.uniqueLabel("Planar Region Data Sets");
   private final HashMap<String, GDXPlanarRegionsGraphic> planarRegionGraphics = new HashMap<>();
   private final ArrayList<Path> pathPlanningDataSetPaths = new ArrayList<>();
   private final ArrayList<Path> reaDataSetPaths = new ArrayList<>();
   private boolean loadedDatasetsOnce = false;

   public void renderImGuiWindow()
   {
      ImGui.begin(windowName);

      /**
       * Folders:
       * ihmc-path-planning/src/data-sets/resources/us/ihmc/pathPlanning/dataSets/20001201_205030_SingleSquare
       */
      boolean reindexClicked = ImGui.button(ImGuiTools.uniqueLabel(this, "Reindex datasets"));
      if (!loadedDatasetsOnce || reindexClicked)
      {
         loadedDatasetsOnce = true;
         Path openRoboticsSoftwarePath = PathTools.findDirectoryInline("ihmc-open-robotics-software");
         pathPlanningDataSetPaths.clear();
         Path pathPlanningDataSetsPath = openRoboticsSoftwarePath.resolve("ihmc-path-planning/src/data-sets/resources/us/ihmc/pathPlanning/dataSets");
         PathTools.walkFlat(pathPlanningDataSetsPath, (path, pathType) ->
         {
            if (pathType == BasicPathVisitor.PathType.DIRECTORY)
            {
               pathPlanningDataSetPaths.add(path.resolve("PlanarRegions"));
            }
            return FileVisitResult.CONTINUE;
         });
         reaDataSetPaths.clear();
         Path reaDataSetsPath = openRoboticsSoftwarePath.resolve("robot-environment-awareness/Data/PlanarRegion");
         PathTools.walkFlat(reaDataSetsPath, (path, pathType) ->
         {
            if (pathType == BasicPathVisitor.PathType.DIRECTORY)
            {
               reaDataSetPaths.add(path);
            }
            return FileVisitResult.CONTINUE;
         });
      }

      ImGui.text("Path planning data sets:");
      renderDataSetPathWidgets(pathPlanningDataSetPaths);
      ImGui.text("REA data sets:");
      renderDataSetPathWidgets(reaDataSetPaths);

      //      ImGui.list

      ImGui.end();

      for (GDXPlanarRegionsGraphic planarRegionsGraphic : planarRegionGraphics.values())
      {
         if (planarRegionsGraphic != null)
         {
            planarRegionsGraphic.render();
         }
      }
   }

   private void renderDataSetPathWidgets(ArrayList<Path> dataSetPaths)
   {
      for (Path dataSetPath : dataSetPaths)
      {
         String dataSetName = dataSetPath.getFileName().toString();
         if (dataSetName.equals("PlanarRegions"))
         {
            dataSetName = dataSetPath.getParent().getFileName().toString();
         }
         GDXPlanarRegionsGraphic graphic = planarRegionGraphics.get(dataSetName);
         if (ImGui.checkbox(ImGuiTools.uniqueLabel(this, dataSetName), graphic != null))
         {
            if (graphic == null)
            {
               PlanarRegionsList planarRegionsList = PlanarRegionFileTools.importPlanarRegionData(dataSetPath.toFile());
               GDXPlanarRegionsGraphic planarRegionsGraphic = new GDXPlanarRegionsGraphic();
               planarRegionsGraphic.generateMeshesAsync(planarRegionsList);
               planarRegionGraphics.put(dataSetName, planarRegionsGraphic);
            }
            else
            {
               graphic.destroy();
               planarRegionGraphics.put(dataSetName, null);
            }
         }
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (GDXPlanarRegionsGraphic planarRegionsGraphic : planarRegionGraphics.values())
      {
         if (planarRegionsGraphic != null)
         {
            planarRegionsGraphic.getRenderables(renderables, pool);
         }
      }
   }

   public String getWindowName()
   {
      return windowName;
   }
}
