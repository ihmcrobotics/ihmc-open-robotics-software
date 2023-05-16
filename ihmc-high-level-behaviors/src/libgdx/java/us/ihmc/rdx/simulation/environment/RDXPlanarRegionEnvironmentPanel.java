package us.ihmc.rdx.simulation.environment;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import us.ihmc.commons.nio.BasicPathVisitor;
import us.ihmc.commons.nio.PathTools;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;
import us.ihmc.robotics.PlanarRegionFileTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.nio.file.FileVisitResult;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.TreeSet;

public class RDXPlanarRegionEnvironmentPanel implements RenderableProvider
{
   private final String windowName = ImGuiTools.uniqueLabel("Planar Region Data Sets");
   private final HashMap<String, RDXPlanarRegionsGraphic> planarRegionGraphics = new HashMap<>();
   private final TreeSet<Path> pathPlanningDataSetPaths = new TreeSet<>(this::alphabetizePlanarRegionFolders);
   private final TreeSet<Path> reaDataSetPaths = new TreeSet<>(this::alphabetizePlanarRegionFolders);

   private boolean loadedDatasetsOnce = false;

   public void renderImGuiWindow()
   {
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
   }

   public void update()
   {
      for (RDXPlanarRegionsGraphic planarRegionsGraphic : planarRegionGraphics.values())
      {
         if (planarRegionsGraphic != null)
         {
            planarRegionsGraphic.update();
         }
      }
   }

   private void renderDataSetPathWidgets(TreeSet<Path> dataSetPaths)
   {
      for (Path dataSetPath : dataSetPaths)
      {
         String dataSetName = dataSetPath.getFileName().toString();
         if (dataSetName.equals("PlanarRegions"))
         {
            dataSetName = dataSetPath.getParent().getFileName().toString();
         }
         RDXPlanarRegionsGraphic graphic = planarRegionGraphics.get(dataSetName);
         if (ImGui.checkbox(ImGuiTools.uniqueLabel(this, dataSetName), graphic != null))
         {
            if (graphic == null)
            {
               PlanarRegionsList planarRegionsList = PlanarRegionFileTools.importPlanarRegionData(dataSetPath.toFile());
               RDXPlanarRegionsGraphic planarRegionsGraphic = new RDXPlanarRegionsGraphic();
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
      for (RDXPlanarRegionsGraphic planarRegionsGraphic : planarRegionGraphics.values())
      {
         if (planarRegionsGraphic != null)
         {
            planarRegionsGraphic.getRenderables(renderables, pool);
         }
      }
   }

   private int alphabetizePlanarRegionFolders(Path path1, Path path2)
   {
      return getPlanarRegionFolderName(path1).compareTo(getPlanarRegionFolderName(path2));
   }

   private String getPlanarRegionFolderName(Path path1)
   {
      String path1FileName = path1.getFileName().toString();
      return path1FileName.equals("PlanarRegions") ? path1.getParent().getFileName().toString() : path1FileName;
   }

   public String getWindowName()
   {
      return windowName;
   }
}
