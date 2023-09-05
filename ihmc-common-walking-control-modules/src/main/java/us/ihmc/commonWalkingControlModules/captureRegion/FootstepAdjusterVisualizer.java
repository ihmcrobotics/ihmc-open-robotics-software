package us.ihmc.commonWalkingControlModules.captureRegion;

import java.awt.Color;

import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class FootstepAdjusterVisualizer implements SCS2YoGraphicHolder
{
   private static final Color colorDefault = Color.BLUE;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name + "Registry");
   private final YoArtifactPolygon nextFootstepPolygonArtifact;

   private final FootstepAdjustor footstepAdjustor;

   private YoFrameConvexPolygon2D yoNextFootstepPolygon;
   private FrameConvexPolygon2D nextFootstepPolygon;

   public FootstepAdjusterVisualizer(FootstepAdjustor footstepAdjustor, YoGraphicsListRegistry yoGraphicsListRegistry, YoRegistry parentRegistry)
   {
      this.footstepAdjustor = footstepAdjustor;

      String nextFootstepCaption = "DesiredTouchdown";

      yoNextFootstepPolygon = new YoFrameConvexPolygon2D(nextFootstepCaption, "", worldFrame, 8, registry);
      nextFootstepPolygon = new FrameConvexPolygon2D(worldFrame);

      nextFootstepPolygonArtifact = new YoArtifactPolygon(nextFootstepCaption, yoNextFootstepPolygon, colorDefault, false);
      nextFootstepPolygonArtifact.setVisible(false);
      yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), nextFootstepPolygonArtifact);

      parentRegistry.addChild(registry);
   }

   public void hide()
   {
      yoNextFootstepPolygon.clear();
   }

   public void update()
   {
      nextFootstepPolygon.setIncludingFrame(footstepAdjustor.getTouchdownFootPolygon());
      nextFootstepPolygon.changeFrameAndProjectToXYPlane(worldFrame);

      try
      {
         yoNextFootstepPolygon.set(nextFootstepPolygon);
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicPolygon2D("DesiredTouchdown", yoNextFootstepPolygon, ColorDefinitions.argb(colorDefault.getRGB())));
      return group;
   }
}
