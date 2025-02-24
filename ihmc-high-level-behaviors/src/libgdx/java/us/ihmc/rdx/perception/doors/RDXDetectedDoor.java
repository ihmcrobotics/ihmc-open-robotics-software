package us.ihmc.rdx.perception.doors;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import perception_msgs.msg.dds.DetectedDoorMessage;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.perception.detections.doors.DetectedDoor;
import us.ihmc.rdx.sceneManager.RDXRenderableProvider;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.Set;

public class RDXDetectedDoor implements RDXRenderableProvider
{
   private final RDXReferenceFrameGraphic openingMechanismFrameGraphic;
   private final RDXReferenceFrameGraphic panelFrameGraphic;
   private final RDXPlanarRegionsGraphic planarRegionGraphic;

   public RDXDetectedDoor()
   {
      openingMechanismFrameGraphic = new RDXReferenceFrameGraphic(0.2, Color.RED);
      panelFrameGraphic = new RDXReferenceFrameGraphic(0.2, Color.BLUE);
      planarRegionGraphic = new RDXPlanarRegionsGraphic();
      planarRegionGraphic.setBlendOpacity(0.6f);
   }

   public void update(DetectedDoor detectedDoor)
   {
      update(detectedDoor.getOpeningMechanism().getPose(),
             detectedDoor.getPanelPose(),
             detectedDoor.getPanelPlanarRegion());
   }

   public void update(DetectedDoorMessage detectedDoorMessage)
   {
      update(detectedDoorMessage.getOpeningMechanism().getPose(),
             detectedDoorMessage.getPanelPose(),
             PlanarRegionMessageConverter.convertToPlanarRegion(detectedDoorMessage.getPanelPlanarRegion()));
   }

   public void update(Pose3DReadOnly openingMechanismPose, Pose3DReadOnly panelPose, PlanarRegion panelPlanarRegion)
   {
      openingMechanismFrameGraphic.setPoseInWorldFrame(openingMechanismPose);
      panelFrameGraphic.setPoseInWorldFrame(panelPose);
      planarRegionGraphic.generateMeshes(new PlanarRegionsList(panelPlanarRegion));
      planarRegionGraphic.update();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (sceneLevels.contains(RDXSceneLevel.VIRTUAL))
      {
         openingMechanismFrameGraphic.getRenderables(renderables, pool);
         panelFrameGraphic.getRenderables(renderables, pool);
         planarRegionGraphic.getRenderables(renderables, pool);
      }
   }

   public void dispose()
   {
      openingMechanismFrameGraphic.dispose();
      panelFrameGraphic.dispose();
      planarRegionGraphic.destroy();
   }
}
