package us.ihmc.rdx.perception.doors;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import perception_msgs.msg.dds.DetectedDoorMessage;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.perception.detections.doors.DetectedDoor;
import us.ihmc.rdx.sceneManager.RDXRenderableProvider;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.Set;

public class RDXDetectedDoor implements RDXRenderableProvider
{
   private DetectedDoor detection;

   private final ModelInstance openingMechanismPositionSphere;
   private final RDXReferenceFrameGraphic openingMechanismFrameGraphic;
   private final ModelInstance panelPositionSphere;
   private final RDXReferenceFrameGraphic panelFrameGraphic;
   private final RDXPlanarRegionsGraphic planarRegionGraphic;

   public RDXDetectedDoor()
   {
      openingMechanismPositionSphere = RDXModelBuilder.createSphere(0.025f, Color.GREEN);
      openingMechanismFrameGraphic = new RDXReferenceFrameGraphic(0.2);
      panelPositionSphere = RDXModelBuilder.createSphere(0.025f, Color.BLUE);
      panelFrameGraphic = new RDXReferenceFrameGraphic(0.2, Color.BLUE);
      planarRegionGraphic = new RDXPlanarRegionsGraphic();
      planarRegionGraphic.setBlendOpacity(0.6f);
   }

   public void update(DetectedDoorMessage detectedDoorMessage)
   {
      update(new DetectedDoor(detectedDoorMessage));
   }

   public void update(DetectedDoor detectedDoor)
   {
      detection = detectedDoor;

      Tuple3DReadOnly openingMechanismTranslation = detection.getOpeningMechanism().getTransformToWorld().getTranslation();
      openingMechanismPositionSphere.transform.setTranslation(openingMechanismTranslation.getX32(),
                                                              openingMechanismTranslation.getY32(),
                                                              openingMechanismTranslation.getZ32());
      openingMechanismFrameGraphic.setToReferenceFrame(detection.getOpeningMechanism().getFrame());

      Tuple3DReadOnly panelTranslation = detection.getPanelTransformToWorld().getTranslation();
      panelPositionSphere.transform.setTranslation(panelTranslation.getX32(), panelTranslation.getY32(), panelTranslation.getZ32());
      panelFrameGraphic.setToReferenceFrame(detection.getPanelFrame());

      planarRegionGraphic.generateMeshes(new PlanarRegionsList(detection.getPanelPlanarRegion()));
      planarRegionGraphic.update();
   }

   public DetectedDoor getDetection()
   {
      return detection;
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (sceneLevels.contains(RDXSceneLevel.VIRTUAL))
      {
         openingMechanismPositionSphere.getRenderables(renderables, pool);
         openingMechanismFrameGraphic.getRenderables(renderables, pool);
         panelPositionSphere.getRenderables(renderables, pool);
         panelFrameGraphic.getRenderables(renderables, pool);
         planarRegionGraphic.getRenderables(renderables, pool);
      }
   }

   public void dispose()
   {
      openingMechanismPositionSphere.model.dispose();
      openingMechanismFrameGraphic.dispose();
      panelPositionSphere.model.dispose();
      panelFrameGraphic.dispose();
      planarRegionGraphic.destroy();
   }
}
