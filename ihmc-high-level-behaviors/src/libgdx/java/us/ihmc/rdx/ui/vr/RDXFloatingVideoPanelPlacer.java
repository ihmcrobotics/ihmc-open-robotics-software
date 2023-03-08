package us.ihmc.rdx.ui.vr;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.ui.graphics.RDX3DSituatedImagePanel;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class RDXFloatingVideoPanelPlacer
{
   private RDXPanelPlacementMode mode = RDXPanelPlacementMode.MANUAL_PLACEMENT;
   private static final double yPanel = 0.0;
   private static final double zPanel = 0.17;
   private final RDX3DSituatedImagePanel floatingVideoPanel = new RDX3DSituatedImagePanel();
   private double lastTouchpadFloatingPanelY = Double.NaN;
   private double panelZoom = 0;
   private double panelDistanceFromHeadset = 0.5;
   private final ModifiableReferenceFrame floatingPanelFrame = new ModifiableReferenceFrame(ReferenceFrame.getWorldFrame());
   private final FramePose3D floatingPanelFramePose = new FramePose3D();
   private final RigidBodyTransform gripOffsetTransform = new RigidBodyTransform();
   private boolean grippedLastTime = false;
   private boolean justActivated = true;


   public void update(RDXVRContext context)
   {
      context.addVRInputProcessor(vrContext ->
      {
         vrContext.getController(RobotSide.LEFT).runIfConnected(controller ->
         {
            if (mode == RDXPanelPlacementMode.FOLLOW_HEADSET)
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
         vrContext.getHeadset().runIfConnected(headset ->
         {
            if (mode == RDXPanelPlacementMode.FOLLOW_HEADSET || (mode == RDXPanelPlacementMode.MANUAL_PLACEMENT && justActivated))
            {
               if (floatingVideoPanel.getModelInstance() != null)
               {
                  floatingPanelFramePose.setToZero(headset.getXForwardZUpHeadsetFrame());
                  floatingPanelFramePose.getPosition().set(panelDistanceFromHeadset, yPanel, zPanel);
                  floatingPanelFramePose.changeFrame(ReferenceFrame.getWorldFrame());
                  floatingPanelFramePose.get(floatingPanelFrame.getTransformToParent());
                  floatingPanelFrame.getReferenceFrame().update();
               }
            }
         });
         vrContext.getController(RobotSide.RIGHT).runIfConnected(controller ->
         {
            if (mode == RDXPanelPlacementMode.MANUAL_PLACEMENT)
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
         });
      });
   }

   public void checkNotification(boolean activatedPanel)
   {
      justActivated = activatedPanel;
   }

   public void setMode(RDXPanelPlacementMode mode)
   {
      if (this.mode != mode)
         this.mode = mode;
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
