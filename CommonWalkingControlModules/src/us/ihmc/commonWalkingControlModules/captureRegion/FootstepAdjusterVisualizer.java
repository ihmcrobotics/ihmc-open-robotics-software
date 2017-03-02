package us.ihmc.commonWalkingControlModules.captureRegion;

import java.awt.Color;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FootstepAdjusterVisualizer
{
   private static final Color colorDefault = Color.BLUE;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name + "Registry");
   private final YoArtifactPolygon nextFootstepPolygonArtifact;

   private final FootstepAdjustor footstepAdjustor;

   private YoFrameConvexPolygon2d yoNextFootstepPolygon;
   private FrameConvexPolygon2d nextFootstepPolygon;

   public FootstepAdjusterVisualizer(FootstepAdjustor footstepAdjustor, YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this.footstepAdjustor = footstepAdjustor;

      String nextFootstepCaption = "DesiredTouchdown";

      yoNextFootstepPolygon = new YoFrameConvexPolygon2d(nextFootstepCaption, "", worldFrame, 8, registry);
      nextFootstepPolygon = new FrameConvexPolygon2d(worldFrame);

      nextFootstepPolygonArtifact = new YoArtifactPolygon(nextFootstepCaption, yoNextFootstepPolygon, colorDefault, false);
      nextFootstepPolygonArtifact.setVisible(false);
      yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), nextFootstepPolygonArtifact);

      parentRegistry.addChild(registry);
   }

   public void hide()
   {
      yoNextFootstepPolygon.hide();
   }

   public void update()
   {
      nextFootstepPolygon.setIncludingFrameAndUpdate(footstepAdjustor.getTouchdownFootPolygon());
      nextFootstepPolygon.changeFrameAndProjectToXYPlane(worldFrame);

      try
      {
         yoNextFootstepPolygon.setFrameConvexPolygon2d(nextFootstepPolygon);
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }
}
