package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.SteppableRegionsListCollectionMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.perception.steppableRegions.SteppableRegion;
import us.ihmc.perception.steppableRegions.SteppableRegionMessageConverter;
import us.ihmc.perception.steppableRegions.SteppableRegionsListCollection;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.List;

public class RDXSteppableRegionGraphic implements RenderableProvider
{
   private final ResettableExceptionHandlingExecutorService executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);

   private boolean renderHeightMap = true;
   private boolean renderPlanes = false;
   private boolean inPaintHeight = true;
   private boolean renderGroundCells = false;
   private boolean renderGroundPlane = false;

   private final RecyclingArrayList<RDXGridMapGraphic> gridMapGraphics = new RecyclingArrayList<>(RDXGridMapGraphic::new);
   private final RDXPlanarRegionsGraphic regionGraphics = new RDXPlanarRegionsGraphic();
   {
      regionGraphics.setDrawNormal(true);
   }

   public void clear()
   {
      for (int i = 0; i < gridMapGraphics.size(); i++)
         gridMapGraphics.get(i).clear();

      regionGraphics.clear();
      gridMapGraphics.clear();
   }

   public void update()
   {
      if (renderPlanes)
         regionGraphics.update();

      if (renderHeightMap)
      {
         for (RDXGridMapGraphic gridMapGraphic : gridMapGraphics)
            gridMapGraphic.update();
      }
   }

   public void setRenderPlanes(boolean renderPlanes)
   {
      this.renderPlanes = renderPlanes;
   }

   public void setRenderHeightMap(boolean renderHeightMap)
   {
      this.renderHeightMap = renderHeightMap;
   }

   public void setInPaintHeight(boolean inPaintHeight)
   {
      this.inPaintHeight = inPaintHeight;
   }

   public void setRenderGroundPlane(boolean renderGroundPlane)
   {
      this.renderGroundPlane = renderGroundPlane;
   }

   public void setRenderGroundCells(boolean renderGroundCells)
   {
      this.renderGroundCells = renderGroundCells;
   }

   public void generateMeshesAsync(SteppableRegionsListCollectionMessage steppableRegions, int yawToShow)
   {
      executorService.clearQueueAndExecute(() -> generateMeshes(steppableRegions, yawToShow));
   }

   private void generateMeshes(SteppableRegionsListCollectionMessage message, int yawToShow)
   {
      SteppableRegionsListCollection steppableRegionsCollection = SteppableRegionMessageConverter.convertToSteppableRegionsListCollection(message);
      List<SteppableRegion> steppableRegionsToView = steppableRegionsCollection.getSteppableRegions(yawToShow).getSteppableRegionsAsList();

      clear();
      if (renderHeightMap)
         steppableRegionsToView.forEach(this::generateMeshes);
      if (renderPlanes)
         generatePlanarRegionMeshes(steppableRegionsToView);
   }

   private void generatePlanarRegionMeshes(List<SteppableRegion> steppableRegionsToView)
   {
      List<PlanarRegion> planarRegions = steppableRegionsToView.stream().map(region ->
                                                                             {
                                                                                PlanarRegion planarRegion = new PlanarRegion();
                                                                                region.toPlanarRegion(planarRegion);
                                                                                return planarRegion;
                                                                             }).toList();
      regionGraphics.generateMeshes(new PlanarRegionsList(planarRegions));
   }

   private void generateMeshes(SteppableRegion steppableRegion)
   {
      RDXGridMapGraphic gridMapGraphic = gridMapGraphics.add();
      HeightMapMessage heightMapMessage = HeightMapMessageTools.toMessage(steppableRegion.getLocalHeightMap());
      heightMapMessage.setSequenceId(steppableRegion.getRegionId());
      gridMapGraphic.setInPaintHeight(inPaintHeight);
      gridMapGraphic.colorFromHeight(false);
      gridMapGraphic.setRenderGroundCells(renderGroundCells);
      gridMapGraphic.setRenderGroundPlane(renderGroundPlane);
      gridMapGraphic.generateMeshes(heightMapMessage);
   }


   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (renderPlanes)
         regionGraphics.getRenderables(renderables, pool);

      if (renderHeightMap)
      {
         for (RDXGridMapGraphic gridMapGraphic : gridMapGraphics)
         {
            if (gridMapGraphic == null)
               continue;

            gridMapGraphic.getRenderables(renderables, pool);
         }
      }
   }

   public void destroy()
   {
      executorService.destroy();
   }

}
