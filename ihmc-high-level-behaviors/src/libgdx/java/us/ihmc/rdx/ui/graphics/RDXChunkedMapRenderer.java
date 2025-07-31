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
         RDXChunkMapRenderer heightMapRenderer = chunkRenderer.getRenderer();
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

   public void addHeightMap(HeightMapMessage heightMapMessage, int hash)
   {
      ChunkRenderer chunkRenderer = chunkRenderers.get(hash);
      if (chunkRenderer == null)
      {
         chunkRenderer = new ChunkRenderer(heightMapMessage);
         chunkRenderers.put(hash, chunkRenderer);
      }

      Mat latestChunk = HeightMapMessageTools.unpackMessageToMatNotCentered(heightMapMessage);

      Chunk chunk = chunkRenderer.getChunk();
      chunk.setChunk(latestChunk);
      chunk.setCellSize(heightMapMessage.getCellSizeInMeters());
      //TODO should create a seperate chunk message, basically the same as the height map but now it would be independent
      chunk.setHeightMapOffset((float) heightMapMessage.getHeightOffset());
      chunk.setOriginX(heightMapMessage.getGridCenterX());
      chunk.setOriginY(heightMapMessage.getGridCenterY());
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
      private final RDXChunkMapRenderer renderer;

      public ChunkRenderer(HeightMapMessage heightMapMessage)
      {
         renderer = new RDXChunkMapRenderer();
         chunk = new Chunk(heightMapMessage);
      }

      public Chunk getChunk()
      {
         return chunk;
      }

      public RDXChunkMapRenderer getRenderer()
      {
         return renderer;
      }
   }
}

