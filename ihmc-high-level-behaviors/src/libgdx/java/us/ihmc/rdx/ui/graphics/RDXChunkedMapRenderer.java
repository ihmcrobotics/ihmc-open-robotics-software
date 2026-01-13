package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.IntMap;
import com.badlogic.gdx.utils.Pool;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ChunkMessage;
import us.ihmc.perception.gpuMapping.worldModel.Chunk;
import us.ihmc.perception.gpuMapping.worldModel.ChunkMessageTools;
import us.ihmc.perception.gpuMapping.worldModel.ChunkTools;

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
            if (chunkRenderer.getMap().ptr(0) != null)
            {
               heightMapRenderer.update(chunkRenderer.getMap(),
                                        (float) chunk.getOriginX(),
                                        (float) chunk.getOriginY(),
                                        chunk.getCellsPerAxis(),
                                        (float) chunk.getCellSize());
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

         Chunk latestChunk = new Chunk(chunkMessage.getOriginX(),
                                       chunkMessage.getOriginY(),
                                       chunkMessage.getCellSizeInMeters(),
                                       chunkMessage.getCellsPerAxis(),
                                       (float) chunkMessage.getWidthInMeters());
         ChunkMessageTools.unpackMessageToChunk(chunkMessage, latestChunk);

         chunkRenderer = new ChunkRenderer(latestChunk);
         chunkRenderers.put(hash, chunkRenderer);
         queueOfRenderers.add(hash);
      }

      ChunkMessageTools.unpackMessageToChunk(chunkMessage, chunkRenderer.getChunk());
      ChunkTools.convertToMat(chunkRenderer.getMap(), chunkRenderer.getChunk());
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
      private final Mat map;
      private final Chunk chunk;
      private final RDXChunkRenderer renderer;

      public ChunkRenderer(Chunk chunk)
      {
         renderer = new RDXChunkRenderer();
         this.chunk = new Chunk(chunk.getOriginX(), chunk.getOriginY(), chunk.getCellSize(), chunk.getCellsPerAxis(), chunk.getWidthInMeters());
         map = new Mat(chunk.getCellsPerAxis(), chunk.getCellsPerAxis(), opencv_core.CV_32FC1);

         ChunkTools.convertToMat(map, chunk);
      }

      public Chunk getChunk()
      {
         return chunk;
      }

      public Mat getMap()
      {
         return map;
      }

      public RDXChunkRenderer getRenderer()
      {
         return renderer;
      }
   }
}