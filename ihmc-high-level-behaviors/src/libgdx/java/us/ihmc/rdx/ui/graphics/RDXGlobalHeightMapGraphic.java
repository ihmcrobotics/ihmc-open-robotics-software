package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.IntMap;
import com.badlogic.gdx.utils.Pool;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.perception.tools.PerceptionDebugTools;

import java.nio.ShortBuffer;
import java.util.Arrays;

/**
 * Creates a graphic for the GPU Height Map to be visualized in the RDX UI. Each height value from the height map is turned into a 2cm polygon that is then
 * visualized on the UI. The height value location will be in the center of the 2cm polygon that is visualized.
 */
public class RDXGlobalHeightMapGraphic implements RenderableProvider
{
   private static class TileRenderer
   {
      private final GlobalTileInfo globalMapTiles;
      private final RDXHeightMapRenderer globalMapTileRenderers;

      public TileRenderer(Mat heightMapMat, float latestHeightMapOffset, float latestCellSizeInMeters, Point3D heightMapCenter, int cellsPerAxis)
      {
         globalMapTileRenderers = new RDXHeightMapRenderer();
         globalMapTiles = new GlobalTileInfo(heightMapMat, latestHeightMapOffset, latestCellSizeInMeters, heightMapCenter, cellsPerAxis);
      }

      public GlobalTileInfo getGlobalMapTiles()
      {
         return globalMapTiles;
      }

      public RDXHeightMapRenderer getGlobalMapTileRenderers()
      {
         return globalMapTileRenderers;
      }
   }

   private static class GlobalTileInfo
   {
      private Mat heightMapMat;
      private float latestHeightMapOffset;
      private float latestCellSizeInMeters;
      private Point3D heightMapCenter;
      private int cellsPerAxis;

      public GlobalTileInfo(Mat heightMapMat, float latestHeightMapOffset, float latestCellSizeInMeters, Point3D heightMapCenter, int cellsPerAxis)
      {
//         By default this array is filled with zeros
//         int totalCells = cellsPerAxis * cellsPerAxis;
//         short[] heightsAsFloats = new short[totalCells];

//         Arrays.fill(heightsAsFloats, (short) 32768);
//         ShortBuffer buffer = this.heightMapMat.createBuffer();
//         buffer.put(heightsAsFloats);

//         PerceptionDebugTools.printMat("s", heightMapMat, 10);

//         this.heightMapMat = heightMapMat;
         this.latestHeightMapOffset = latestHeightMapOffset;
         this.latestCellSizeInMeters = latestCellSizeInMeters;
         this.heightMapCenter = heightMapCenter;
         this.cellsPerAxis = cellsPerAxis;
      }

      public Mat getHeightMapMat()
      {
         return heightMapMat;
      }

      public float getLatestHeightMapOffset()
      {
         return latestHeightMapOffset;
      }

      public float getLatestCellSizeInMeters()
      {
         return latestCellSizeInMeters;
      }

      public Point3D getHeightMapCenter()
      {
         return heightMapCenter;
      }

      public int getCellsPerAxis()
      {
         return cellsPerAxis;
      }

      public void setHeightMapMat(Mat heightMapMat)
      {
         this.heightMapMat = heightMapMat;
      }

      public void setLatestHeightMapOffset(float latestHeightMapOffset)
      {
         this.latestHeightMapOffset = latestHeightMapOffset;
      }

      public void setLatestCellSizeInMeters(float latestCellSizeInMeters)
      {
         this.latestCellSizeInMeters = latestCellSizeInMeters;
      }

      public void setHeightMapCenter(Point3D heightMapCenter)
      {
         this.heightMapCenter = heightMapCenter;
      }

      public void setCellsPerAxis(int cellsPerAxis)
      {
         this.cellsPerAxis = cellsPerAxis;
      }
   }

   private final IntMap<TileRenderer> globalMapRenderables = new IntMap<>();

   public RDXGlobalHeightMapGraphic()
   {
   }

   public void create()
   {
      for (TileRenderer globalMapTileRenderer : globalMapRenderables.values())
      {
         if (!globalMapTileRenderer.getGlobalMapTileRenderers().isHasBeenCreated())
         {
            int cellsPerAxis = globalMapTileRenderer.getGlobalMapTiles().getCellsPerAxis();
            globalMapTileRenderer.getGlobalMapTileRenderers().create(cellsPerAxis * cellsPerAxis);
         }
      }
   }

   public void update()
   {
      for (TileRenderer globalMapTileRenderer : globalMapRenderables.values())
      {
         RDXHeightMapRenderer heightMapRenderer = globalMapTileRenderer.getGlobalMapTileRenderers();
         if (heightMapRenderer.isHasBeenCreated())
         {
            GlobalTileInfo mapTile = globalMapTileRenderer.getGlobalMapTiles();
            if (mapTile != null)
            {
               if (mapTile.getHeightMapMat() != null && mapTile.getHeightMapMat().ptr(0) != null)
               {
                  float pixelScalingFactor = 10000.0f;
                  heightMapRenderer.update(mapTile.getHeightMapMat(),
                                           mapTile.getLatestHeightMapOffset(),
                                           mapTile.getHeightMapCenter().getX32(),
                                           mapTile.getHeightMapCenter().getY32(),
                                           mapTile.getCellsPerAxis() / 2,
                                           mapTile.getLatestCellSizeInMeters(),
                                           pixelScalingFactor);
               }
            }
         }
      }
   }

   public void addHeightMap(Mat heightMapMat, float latestHeightMapOffset, float latestCellSizeInMeters, Point3D heightMapCenter, int cellsPerAxis, int hash)
   {
      TileRenderer currentMapTile = globalMapRenderables.get(hash);
      if (currentMapTile == null)
      {
         currentMapTile = new TileRenderer(heightMapMat, latestHeightMapOffset, latestCellSizeInMeters, heightMapCenter, cellsPerAxis);
         globalMapRenderables.put(hash, currentMapTile);
      }

      GlobalTileInfo globalMapTiles = currentMapTile.getGlobalMapTiles();
      globalMapTiles.setHeightMapMat(heightMapMat);
      globalMapTiles.setLatestHeightMapOffset(latestHeightMapOffset);
      globalMapTiles.setLatestCellSizeInMeters(latestCellSizeInMeters);
      globalMapTiles.setHeightMapCenter(heightMapCenter);
      globalMapTiles.setCellsPerAxis(cellsPerAxis);
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (TileRenderer currentRenderer : globalMapRenderables.values())
      {
         // We can't guarantee that this has been created by the time this is called, so need to check
         if (currentRenderer.getGlobalMapTileRenderers().isHasBeenCreated())
            currentRenderer.getGlobalMapTileRenderers().getRenderables(renderables, pool);
      }
   }

   public void destroy()
   {
      for (TileRenderer currentRenderer : globalMapRenderables.values())
      {
         currentRenderer.getGlobalMapTileRenderers().dispose();
      }
   }
}

