package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import java.awt.Color;

import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.plotting.DynamicGraphicYoPolygonArtifact;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameConvexPolygon2d;

public class FootPolygonVisualizer
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final SideDependentList<? extends BipedFootInterface> bipedFeet;
   private final SideDependentList<YoFrameConvexPolygon2d> yoFootPolygons = new SideDependentList<YoFrameConvexPolygon2d>();
   private static final SideDependentList<Color> colors = new SideDependentList<Color>(new Color(53, 184, 144), new Color(202, 119, 11));

   public FootPolygonVisualizer(SideDependentList<? extends BipedFootInterface> bipedFeet, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry,
                                YoVariableRegistry parentRegistry)
   {
      this.bipedFeet = bipedFeet;

      for (RobotSide robotSide : RobotSide.values())
      {
         YoFrameConvexPolygon2d yoFootPolygon = new YoFrameConvexPolygon2d(robotSide + "foot", "", ReferenceFrame.getWorldFrame(), 30, registry);
         yoFootPolygons.put(robotSide, yoFootPolygon);
         Color color = colors.get(robotSide);

         DynamicGraphicYoPolygonArtifact dynamicGraphicYoPolygonArtifact = new DynamicGraphicYoPolygonArtifact(robotSide + " Foot", yoFootPolygon,
                                                                              color, false);
         dynamicGraphicObjectsListRegistry.registerArtifact(robotSide + " Foot", dynamicGraphicYoPolygonArtifact);
      }


      parentRegistry.addChild(registry);
   }

   public void update()
   {
      for (RobotSide robotSide : RobotSide.values())
      {
         YoFrameConvexPolygon2d yoFootPolygon = yoFootPolygons.get(robotSide);
         BipedFootInterface bipedFoot = bipedFeet.get(robotSide);
         yoFootPolygon.setFrameConvexPolygon2d(bipedFoot.getFootPolygonInUseInAnkleZUp().changeFrameAndProjectToXYPlaneCopy(ReferenceFrame.getWorldFrame()));
      }
   }
}
