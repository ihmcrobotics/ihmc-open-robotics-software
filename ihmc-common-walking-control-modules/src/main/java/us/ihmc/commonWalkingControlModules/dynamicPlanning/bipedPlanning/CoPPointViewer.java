package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

public class CoPPointViewer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final double size = 0.01;
   private static final int maxPoints = 20;

   private final List<YoFramePoint3D> copStartPoints = new ArrayList<>();
   private final List<YoFramePoint3D> copEndPoints = new ArrayList<>();

   private static final String name = "Corner Points";

   public CoPPointViewer(YoRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      for (int i = 0; i < maxPoints; i++)
      {
         YoFramePoint3D copStartPoint = new YoFramePoint3D("copStartPoint" + i, worldFrame, registry);
         YoFramePoint3D copEndPoint = new YoFramePoint3D("copEndPoint" + i, worldFrame, registry);

         copStartPoint.setToNaN();
         copEndPoint.setToNaN();

         copStartPoints.add(copStartPoint);
         copEndPoints.add(copEndPoint);

         YoGraphicPosition copStartPointGraphic = new YoGraphicPosition("copStartPoint" + i, copStartPoint, size, YoAppearance.DarkRed(), GraphicType.BALL);
         YoGraphicPosition copEndPointGraphic = new YoGraphicPosition("copEndPoint" + i, copEndPoint, size, YoAppearance.DarkRed(), GraphicType.SOLID_BALL);

         yoGraphicsListRegistry.registerYoGraphic(name, copStartPointGraphic);
         yoGraphicsListRegistry.registerYoGraphic(name, copEndPointGraphic);

         yoGraphicsListRegistry.registerArtifact(name, copStartPointGraphic.createArtifact());
         yoGraphicsListRegistry.registerArtifact(name, copEndPointGraphic.createArtifact());
      }

   }

   public void updateWaypoints(List<? extends ContactStateProvider> vrpSegments)
   {
      int i = 0;

      for (;i < Math.min(vrpSegments.size(), maxPoints); i++)
      {
         copStartPoints.get(i).set(vrpSegments.get(i).getCopStartPosition());
         copEndPoints.get(i).set(vrpSegments.get(i).getCopEndPosition());
      }
      for (;i < maxPoints; i++)
      {
         copStartPoints.get(i).setToNaN();
         copEndPoints.get(i).setToNaN();
      }
   }
}
