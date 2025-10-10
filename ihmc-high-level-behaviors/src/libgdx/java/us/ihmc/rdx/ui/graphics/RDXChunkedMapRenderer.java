package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.IntMap;
import com.badlogic.gdx.utils.Pool;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ChunkMessage;
import us.ihmc.perception.gpuMapping.worldModel.Chunk;
import us.ihmc.perception.gpuMapping.HeightMapMessageTools;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;

public class RDXChunkedMapRenderer implements RenderableProvider
{
   private static final int MAX_ENTREES = 100;

   private final IntMap<ChunkRenderer> chunkRenderers = new IntMap<>();
   private final Queue<Integer> queueOfRenderers = new ArrayDeque<>();
   private final List<ChunkRenderer> renderersToRemove = new ArrayList<>();

   public RDXChunkedMapRenderer()
   {
   }

   public void removeOldRenderers()
   {
      for (int i = 0; i < renderersToRemove.size(); i++)
      {
         ChunkRenderer renderer = renderersToRemove.remove(i);
         renderer.getRenderer().dispose();
      }
   }

   public void create()
   {
      for (ChunkRenderer chunkRenderer : chunkRenderers.values())
      {
         if (chunkRenderer == null)
         {
            // Different threads means this may be removed already which would make it null
            continue;
         }

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
         if (chunkRenderer == null)
         {
            // Different threads means this may be removed already which would make it null
            continue;
         }

         RDXChunkRenderer heightMapRenderer = chunkRenderer.getRenderer();
         if (heightMapRenderer.isHasBeenCreated())
         {
            Chunk chunk = chunkRenderer.getChunk();
            if (chunk != null)
            {
               if (chunk.getChunk() != null && chunk.getChunk().ptr(0) != null)
               {
                  heightMapRenderer.update(chunk.getChunk(),
                                           (float) chunk.getOriginX(),
                                           (float) chunk.getOriginY(),
                                           chunk.getCellsPerAxis(),
                                           (float) chunk.getCellSize());
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
         if (chunkRenderers.size > MAX_ENTREES && !queueOfRenderers.isEmpty())
         {
            int oldestRenderer = queueOfRenderers.poll();
            ChunkRenderer oldestChunkRenderer = chunkRenderers.remove(oldestRenderer);
            renderersToRemove.add(oldestChunkRenderer);
         }

         chunkRenderer = new ChunkRenderer(chunkMessage);
         chunkRenderers.put(hash, chunkRenderer);
         queueOfRenderers.add(hash);
      }

      Mat latestChunk = HeightMapMessageTools.unpackMessageToMat(chunkMessage);

      Chunk chunk = chunkRenderer.getChunk();
      chunk.setChunk(latestChunk);
      chunk.setOriginX(chunkMessage.getOriginX());
      chunk.setOriginY(chunkMessage.getOriginY());
      chunk.setCellSize(chunkMessage.getCellSizeInMeters());
      chunk.setCellsPerAxis(chunkMessage.getCellsPerAxis());

      latestChunk.close();
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

