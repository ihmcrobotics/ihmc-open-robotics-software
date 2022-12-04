package us.ihmc.rdx.mesh;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

public class RDXAsyncMeshBuilding
{
   private volatile Runnable buildMeshAndCreateModelInstance = null;
   private final ResettableExceptionHandlingExecutorService executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
   private TypedNotification<ModelInstance> newlyBuiltModelInstance = new TypedNotification<>();

   public void update()
   {
      if (buildMeshAndCreateModelInstance != null)
      {
         buildMeshAndCreateModelInstance.run();
         buildMeshAndCreateModelInstance = null;
      }
   }

   public void generateMeshesAsync(PlanarRegionsList planarRegionsList)
   {
//      executorService.clearQueueAndExecute(() -> generateMeshes(planarRegionsList));
   }

   public TypedNotification<ModelInstance> getNewlyBuiltModelInstanceNotification()
   {
      return newlyBuiltModelInstance;
   }
}
