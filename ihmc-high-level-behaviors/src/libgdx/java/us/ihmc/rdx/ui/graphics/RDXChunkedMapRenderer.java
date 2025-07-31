package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.IntMap;
import com.badlogic.gdx.utils.Pool;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ChunkMessage;
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
         RDXChunkRenderer heightMapRenderer = chunkRenderer.getRenderer();
         if (heightMapRenderer.isHasBeenCreated())
         {
            Chunk chunk = chunkRenderer.getChunk();
            if (chunk != null)
            {
               if (chunk.getChunk() != null && chunk.getChunk().ptr(0) != null)
               {
                     heightMapRenderer.update(chunk.getChunk(),
                                           (float) chunk.getHeightMapOffset(),
                                           (float) chunk.getOriginX(),
                                           (float) chunk.getOriginY(),
                                           chunk.getCellsPerAxis(),
                                           (float) chunk.getCellSize(),
                                           (float) chunk.getScalingFactor());
               }
            }
         }
      }
   }

   public void addHeightMap(ChunkMessage chunkMessage, int hash)
   {
      ChunkRenderer chunkRenderer = chunkRenderers.get(hash);
      if (chunkRenderer == null)
      {
         chunkRenderer = new ChunkRenderer(chunkMessage);
         chunkRenderers.put(hash, chunkRenderer);
      }

      Mat latestChunk = HeightMapMessageTools.unpackMessageToMat(chunkMessage);

      Chunk chunk = chunkRenderer.getChunk();
      chunk.setChunk(latestChunk);
      chunk.setCellSize(chunkMessage.getCellSizeInMeters());
      chunk.setHeightMapOffset((float) chunkMessage.getHeightOffset());
      chunk.setOriginX(chunkMessage.getOriginX());
      chunk.setOriginY(chunkMessage.getOriginY());
      chunk.setCellsPerAxis(chunkMessage.getCellsPerAxis());
      chunk.setScalingFactor(chunkMessage.getHeightScaleFactor());
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
      private final RDXChunkRenderer renderer;

      public ChunkRenderer(ChunkMessage chunkMessage)
      {
         renderer = new RDXChunkRenderer();
         chunk = new Chunk(chunkMessage);
      }

      public Chunk getChunk()
      {
         return chunk;
      }

      public RDXChunkRenderer getRenderer()
      {
         return renderer;
      }
   }
}

