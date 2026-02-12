package us.ihmc.rdx.behaviorTree.scene;

import behavior_msgs.msg.dds.PersistentDetectionStatusMessage;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.rdx.RDX3DSituatedText;
import us.ihmc.rdx.behaviorTree.RDXCRDTTools;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;

import java.util.Random;

public class RDXBehaviorTreeSceneDetection
{
   private static final Random random = new Random();
   private final RDXBaseUI baseUI;
   private final PersistentDetectionStatusMessage message = new PersistentDetectionStatusMessage();
   private final ModelInstance frameGraphic;
   private final RDX3DSituatedText textLabel = new RDX3DSituatedText();

   public RDXBehaviorTreeSceneDetection(RDXBaseUI baseUI)
   {
      this.baseUI = baseUI;

      frameGraphic = RDXModelBuilder.createCoordinateFrameInstance(0.12, LibGDXTools.toLibGDX(YoAppearance.randomColor(random)));
      textLabel.setTextHeight(0.05f);
   }

   public void update(PersistentDetectionStatusMessage status)
   {
      message.set(status);

      RDXCRDTTools.toLibGDX(status.getTransformToWorld(), frameGraphic.transform);

      textLabel.setTextWithoutCache(status.getObjectClassAsString() + " " + status.getId());
      textLabel.setPositionFacingCamera(baseUI.getPrimary3DPanel().getCamera3D(),
                                        status.getTransformToWorld().getX(),
                                        status.getTransformToWorld().getY(),
                                        status.getTransformToWorld().getZ());
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      frameGraphic.getRenderables(renderables, pool);
      textLabel.getRenderables(renderables, pool);
   }

   public PersistentDetectionStatusMessage getMessage()
   {
      return message;
   }
}
