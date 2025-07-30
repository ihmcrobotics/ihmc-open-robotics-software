package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.IntMap;
import com.badlogic.gdx.utils.Pool;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.HeightMapMessage;
import us.ihmc.perception.gpuHeightMap.worldModel.Chunk;
import us.ihmc.perception.heightMap.HeightMapMessageTools;

public class RDXChunkedMapRenderer implements RenderableProvider
{
   private final IntMap<ChunkRenderer> chunkRenderers = new IntMap<>();

   public RDXChunkedMapRenderer()
   {
   }

   public void create()
   {
      for (ChunkRenderer chunkRenderer : chunkRenderers.values())
      {
         if (!chunkRenderer.getRenderer().isHasBeenCreated())
         {
            int cellsPerAxis = chunkRenderer.getChunk().getCellsPerAxis();
            chunkRenderer.getRenderer().create(cellsPerAxis * cellsPerAxis);
         }
      }
   }

   public void update()
   {
      for (ChunkRenderer chunkRenderer : chunkRenderers.values())
      {
         RDXHeightMapRenderer heightMapRenderer = chunkRenderer.getRenderer();
         if (heightMapRenderer.isHasBeenCreated())
         {
            Chunk chunk = chunkRenderer.getChunk();
            if (chunk != null)
            {
               if (chunk.getChunk() != null && chunk.getChunk().ptr(0) != null)
               {
                  heightMapRenderer.update(chunk.getChunk(),
                                           chunk.getHeightMapOffset(),
                                           (float) chunk.getCenterX(),
                                           (float) chunk.getCenterY(),
                                           chunk.getCellsPerAxis() / 2,
                                           (float) chunk.getCellSize(),
                                           chunk.getScalingFactor());
               }
            }
         }
      }
   }

   public void addHeightMap(HeightMapMessage heightMapMessage, int hash)
   {
      ChunkRenderer chunkRenderer = chunkRenderers.get(hash);
      if (chunkRenderer == null)
      {
         chunkRenderer = new ChunkRenderer(heightMapMessage);
         chunkRenderers.put(hash, chunkRenderer);
      }

      Mat latestChunk = HeightMapMessageTools.unpackMessageToMat(heightMapMessage);

      Chunk chunk = chunkRenderer.getChunk();
      chunk.setChunk(latestChunk);
      chunk.setCellSize(heightMapMessage.getCellSizeInMeters());
      chunk.setHeightMapOffset((float) heightMapMessage.getHeightOffset());
      chunk.setCenterX(heightMapMessage.getGridCenterX());
      chunk.setCenterY(heightMapMessage.getGridCenterY());
      chunk.setCellsPerAxis(heightMapMessage.getCellsPerAxis());
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (ChunkRenderer currentRenderer : chunkRenderers.values())
      {
         // We can't guarantee that this has been created by the time this is called, so need to check
         if (currentRenderer.getRenderer().isHasBeenCreated())
            currentRenderer.getRenderer().getRenderables(renderables, pool);
      }
   }

   public void destroy()
   {
      for (ChunkRenderer currentRenderer : chunkRenderers.values())
      {
         currentRenderer.getRenderer().dispose();
      }
   }

   private static class ChunkRenderer
   {
      private final Chunk chunk;
      private final RDXHeightMapRenderer renderer;

      public ChunkRenderer(HeightMapMessage heightMapMessage)
      {
         renderer = new RDXHeightMapRenderer();
         chunk = new Chunk(heightMapMessage);
      }

      public Chunk getChunk()
      {
         return chunk;
      }

      public RDXHeightMapRenderer getRenderer()
      {
         return renderer;
      }
   }
}

