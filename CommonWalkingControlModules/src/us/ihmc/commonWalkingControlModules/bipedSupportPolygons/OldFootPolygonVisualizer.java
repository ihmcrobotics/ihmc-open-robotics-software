package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import java.awt.Color;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.graphics.plotting.YoArtifactPolygon;
import us.ihmc.yoUtilities.math.frames.YoFrameConvexPolygon2d;


public class OldFootPolygonVisualizer implements Updatable
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final SideDependentList<? extends BipedFootInterface> bipedFeet;
   private final SideDependentList<YoFrameConvexPolygon2d> yoFootPolygons = new SideDependentList<YoFrameConvexPolygon2d>();
   private static final SideDependentList<Color> colors = new SideDependentList<Color>(new Color(53, 184, 144), new Color(202, 119, 11));

   public OldFootPolygonVisualizer(SideDependentList<? extends BipedFootInterface> bipedFeet, YoGraphicsListRegistry yoGraphicsListRegistry,
                                YoVariableRegistry parentRegistry)
   {
      this.bipedFeet = bipedFeet;

      if (yoGraphicsListRegistry != null)
      {       
        for (RobotSide robotSide : RobotSide.values)
        {
           YoFrameConvexPolygon2d yoFootPolygon = new YoFrameConvexPolygon2d(robotSide.getSideNameFirstLetter() + "foot", "", ReferenceFrame.getWorldFrame(), 30, registry);
           yoFootPolygons.put(robotSide, yoFootPolygon);
           Color color = colors.get(robotSide);
           
           YoArtifactPolygon dynamicGraphicYoPolygonArtifact = new YoArtifactPolygon(robotSide.getSideNameFirstLetter() + " Foot", yoFootPolygon,
                 color, false);
           yoGraphicsListRegistry.registerArtifact(robotSide.getSideNameFirstLetter() + " Foot", dynamicGraphicYoPolygonArtifact);
        }
      }


      parentRegistry.addChild(registry);
   }

   public void update(double t)
   {
      update();
   }
   
   public void update()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         YoFrameConvexPolygon2d yoFootPolygon = yoFootPolygons.get(robotSide);
         if (yoFootPolygon != null)
         {
          BipedFootInterface bipedFoot = bipedFeet.get(robotSide);
          yoFootPolygon.setFrameConvexPolygon2d(bipedFoot.getFootPolygonInUseInAnkleZUpCopy().changeFrameAndProjectToXYPlaneCopy(ReferenceFrame.getWorldFrame()));           
         }
      }
   }
}
