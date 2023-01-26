package us.ihmc.rdx.visualizers;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.graphics.g3d.model.MeshPart;
import com.badlogic.gdx.graphics.g3d.utils.MeshBuilder;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import org.lwjgl.opengl.GL41;
import perception_msgs.msg.dds.HeightMapMessage;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.ihmcPerception.steppableRegions.SteppableRegion;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.mesh.RDXMultiColorMeshBuilder;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.IntFunction;
import java.util.function.IntToDoubleFunction;

public class RDXSteppableRegionGraphic implements RenderableProvider
{
   private final ResettableExceptionHandlingExecutorService executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);

   private boolean inPaintHeight = true;
   private boolean renderGroundCells = false;
   private boolean renderGroundPlane = false;

   private final RecyclingArrayList<RDXGridMapGraphic> gridMapGraphics = new RecyclingArrayList<>(RDXGridMapGraphic::new);


   public void clear()
   {
      gridMapGraphics.clear();
   }

   public void update()
   {
      for (RDXGridMapGraphic gridMapGraphic : gridMapGraphics)
         gridMapGraphic.update();
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

   public void generateMeshesAsync(List<SteppableRegion> steppableRegions)
   {
      executorService.clearQueueAndExecute(() -> generateMeshes(steppableRegions));
   }

   private void generateMeshes(List<SteppableRegion> steppableRegions)
   {
      gridMapGraphics.clear();
      steppableRegions.forEach(this::generateMeshes);
   }

   private void generateMeshes(SteppableRegion steppableRegion)
   {
      RDXGridMapGraphic gridMapGraphic = gridMapGraphics.add();
      HeightMapMessage heightMapMessage = HeightMapMessageTools.toMessage(steppableRegion.getLocalHeightMap());
      heightMapMessage.setSequenceId(steppableRegion.getRegionId());
      gridMapGraphic.setInPaintHeight(inPaintHeight);
      gridMapGraphic.setRenderGroundCells(renderGroundCells);
      gridMapGraphic.setRenderGroundPlane(renderGroundPlane);
      gridMapGraphic.generateMeshes(heightMapMessage);
   }


   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (RDXGridMapGraphic gridMapGraphic : gridMapGraphics)
         gridMapGraphic.getRenderables(renderables, pool);
   }

   public void destroy()
   {
      executorService.destroy();
   }

}
