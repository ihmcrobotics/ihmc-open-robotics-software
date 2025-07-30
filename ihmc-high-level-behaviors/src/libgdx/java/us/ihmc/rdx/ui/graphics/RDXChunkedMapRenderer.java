package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.IntMap;
import com.badlogic.gdx.utils.Pool;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.euclid.tuple3D.Point3D;

public class RDXChunkedMapRenderer implements RenderableProvider
{
   private static class TileRenderer
   {
      private final GlobalTileInfo globalMapTiles;
      private final RDXHeightMapRenderer globalMapTileRenderers;

      public TileRenderer(float latestHeightMapOffset, float latestCellSizeInMeters, Point3D heightMapCenter, int cellsPerAxis)
      {
         globalMapTileRenderers = new RDXHeightMapRenderer();
         globalMapTiles = new GlobalTileInfo(latestHeightMapOffset, latestCellSizeInMeters, heightMapCenter, cellsPerAxis);
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

      public GlobalTileInfo(float latestHeightMapOffset, float latestCellSizeInMeters, Point3D heightMapCenter, int cellsPerAxis)
      {
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

   public RDXChunkedMapRenderer()
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
         currentMapTile = new TileRenderer(latestHeightMapOffset, latestCellSizeInMeters, heightMapCenter, cellsPerAxis);
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

