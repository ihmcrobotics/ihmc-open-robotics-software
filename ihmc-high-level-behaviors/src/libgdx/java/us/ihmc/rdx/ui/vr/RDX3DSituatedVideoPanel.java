package us.ihmc.rdx.ui.vr;

import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.graphics.RDX3DSituatedImagePanel;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.Set;

import static us.ihmc.rdx.ui.vr.RDX3DSituatedVideoPanelMode.*;

public class RDX3DSituatedVideoPanel
{
   private static final double FOLLOW_HEADSET_OFFSET_Y = 0.0;
   private static final double FOLLOW_HEADSET_OFFSET_Z = 0.17;

   private final RDXVRModeManager vrModeManager;
   private RDX3DSituatedVideoPanelMode placementMode;
   private final RDX3DSituatedImagePanel floatingVideoPanel = new RDX3DSituatedImagePanel();
   private final ModifiableReferenceFrame floatingPanelFrame = new ModifiableReferenceFrame(ReferenceFrame.getWorldFrame());
   private final FramePose3D floatingPanelFramePose = new FramePose3D();
   private final RigidBodyTransform gripOffsetTransform = new RigidBodyTransform();
   private double lastTouchpadFloatingPanelY = Double.NaN;
   private double panelZoom = 0;
   private double panelDistanceFromHeadset = 0.5;
   private boolean grippedLastTime = false;
   private boolean justShown;
   private boolean isShowing;

   public RDX3DSituatedVideoPanel(RDXVRContext context, RDXVRModeManager vrModeManager)
   {
      this.vrModeManager = vrModeManager;
      context.addVRInputProcessor(this::addVRInputProcessor);
   }

   public void update(Texture imageTexture)
   {
      placementMode = vrModeManager.getVideoPanelPlacementMode();
      isShowing = vrModeManager.getShowFloatingVideoPanel().get();
      justShown = vrModeManager.getShowFloatVideoPanelNotification().poll();

      if (isShowing && imageTexture != floatingVideoPanel.getTexture())
      {
         boolean flipY = false;
         float multiplier = 2.0f;
         float halfWidth = imageTexture.getWidth() / 1000.0f * multiplier;
         float halfHeight = imageTexture.getHeight() / 1000.0f * multiplier;
         floatingVideoPanel.create(imageTexture,
                                   new Vector3[] {new Vector3(0.0f, halfWidth, halfHeight),
                                                  new Vector3(0.0f, halfWidth, -halfHeight),
                                                  new Vector3(0.0f, -halfWidth, -halfHeight),
                                                  new Vector3(0.0f, -halfWidth, halfHeight)},
                                   floatingPanelFrame.getReferenceFrame(),
                                   flipY);
      }

      floatingVideoPanel.setPoseToReferenceFrame(floatingPanelFrame.getReferenceFrame());
   }

   public void addVRInputProcessor(RDXVRContext context)
   {
      context.getHeadset().runIfConnected(headset ->
      {
         if (placementMode == FOLLOW_HEADSET || (placementMode == MANUAL_PLACEMENT && justShown))
         {
            if (floatingVideoPanel.getModelInstance() != null)
            {
               floatingPanelFramePose.setToZero(headset.getXForwardZUpHeadsetFrame());
               floatingPanelFramePose.getPosition().set(panelDistanceFromHeadset, FOLLOW_HEADSET_OFFSET_Y, FOLLOW_HEADSET_OFFSET_Z);
               floatingPanelFramePose.changeFrame(ReferenceFrame.getWorldFrame());
               floatingPanelFramePose.get(floatingPanelFrame.getTransformToParent());
               floatingPanelFrame.getReferenceFrame().update();
            }
         }
      });

      for (RobotSide side : RobotSide.values)
      {
         context.getController(side).runIfConnected(controller ->
         {
            if (placementMode == MANUAL_PLACEMENT)
            {
               if (floatingVideoPanel.getModelInstance() != null)
               {
                  floatingPanelFramePose.setToZero(floatingPanelFrame.getReferenceFrame());
                  floatingPanelFramePose.changeFrame(ReferenceFrame.getWorldFrame());
                  boolean controllerIsCloseToPanel = controller.getXForwardZUpPose().getPosition().distance(floatingPanelFramePose.getPosition()) < 0.05;
                  boolean isGripping = controller.getGripActionData().x() > 0.9;
                  if ((grippedLastTime || controllerIsCloseToPanel) && isGripping)
                  {
                     if (!grippedLastTime) // set up offset
                     {
                        floatingPanelFramePose.changeFrame(controller.getXForwardZUpControllerFrame());
                        floatingPanelFramePose.get(gripOffsetTransform);
                        floatingPanelFramePose.changeFrame(ReferenceFrame.getWorldFrame());
                     }
                     floatingPanelFrame.getTransformToParent().set(gripOffsetTransform);
                     controller.getXForwardZUpControllerFrame().getTransformToWorldFrame().transform(floatingPanelFrame.getTransformToParent());
                     floatingPanelFrame.getReferenceFrame().update();

                     grippedLastTime = true;
                  }
                  else
                  {
                     grippedLastTime = false;
                  }
               }
            }
            // TODO: Make this context based, by pointing at the panel
            else if (placementMode == FOLLOW_HEADSET && side == RobotSide.LEFT)
            {
               if (controller.getTouchpadTouchedActionData().bState())
               {
                  double y = controller.getTouchpadActionData().y();
                  if (!Double.isNaN(lastTouchpadFloatingPanelY))
                  {
                     panelZoom = y - lastTouchpadFloatingPanelY;
                  }
                  lastTouchpadFloatingPanelY = y;
                  panelDistanceFromHeadset = panelDistanceFromHeadset + panelZoom;
               }
               else

               {
                  lastTouchpadFloatingPanelY = Double.NaN;
               }
            }
         });
      }
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (vrModeManager.getShowFloatingVideoPanel().get() && sceneLevels.contains(RDXSceneLevel.VIRTUAL))
         floatingVideoPanel.getRenderables(renderables, pool);
   }

   public RDX3DSituatedImagePanel getFloatingVideoPanel()
   {
      return floatingVideoPanel;
   }

   public ModifiableReferenceFrame getFloatingPanelFrame()
   {
      return floatingPanelFrame;
   }
}
