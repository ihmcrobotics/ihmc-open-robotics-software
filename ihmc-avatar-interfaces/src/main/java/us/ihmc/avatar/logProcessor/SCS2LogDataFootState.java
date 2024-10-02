package us.ihmc.avatar.logProcessor;

import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.ArrayList;

public class SCS2LogDataFootState
{
   private final RobotSide side;
   private final SCS2LogDataEnum<ConstraintType> yoFootState;
   private boolean newStep = false;
   private double fullSupportTime = Double.NaN;
   private YoVariable footPolygon_0_x ;
   private YoVariable footPolygon_0_y ;
   private YoVariable footPolygon_1_x ;
   private YoVariable footPolygon_1_y ;
   private YoVariable footPolygon_2_x ;
   private YoVariable footPolygon_2_y ;
   private YoVariable footPolygon_3_x ;
   private YoVariable footPolygon_3_y ;
   private final ArrayList<SCS2LogDataFootstep> footsteps = new ArrayList<>();
   private final double comPlotProximityToFootsteps = 5.0;

   public SCS2LogDataFootState(RobotSide side, SCS2LogDataEnum<ConstraintType> yoFootState, YoRegistry rootRegistry)
   {
      this.side = side;
      this.yoFootState = yoFootState;

      String highLevelController = "root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.";
      String footPolygonPrefix = highLevelController + "HighLevelHumanoidControllerToolbox.BipedSupportPolygons.";
      footPolygon_0_x  = rootRegistry.findVariable(footPolygonPrefix + "%sFootPolygon_0_x".formatted(side.getLowerCaseName()));
      footPolygon_0_y  = rootRegistry.findVariable(footPolygonPrefix + "%sFootPolygon_0_y".formatted(side.getLowerCaseName()));
      footPolygon_1_x  = rootRegistry.findVariable(footPolygonPrefix + "%sFootPolygon_1_x".formatted(side.getLowerCaseName()));
      footPolygon_1_y  = rootRegistry.findVariable(footPolygonPrefix + "%sFootPolygon_1_y".formatted(side.getLowerCaseName()));
      footPolygon_2_x  = rootRegistry.findVariable(footPolygonPrefix + "%sFootPolygon_2_x".formatted(side.getLowerCaseName()));
      footPolygon_2_y  = rootRegistry.findVariable(footPolygonPrefix + "%sFootPolygon_2_y".formatted(side.getLowerCaseName()));
      footPolygon_3_x  = rootRegistry.findVariable(footPolygonPrefix + "%sFootPolygon_3_x".formatted(side.getLowerCaseName()));
      footPolygon_3_y  = rootRegistry.findVariable(footPolygonPrefix + "%sFootPolygon_3_y".formatted(side.getLowerCaseName()));
   }

   public boolean afterRead(double currentTime)
   {
      if (yoFootState.changedTo(ConstraintType.FULL))
      {
         newStep = true;
         fullSupportTime = currentTime;
      }
      yoFootState.postUpdate();

      if (newStep && currentTime - fullSupportTime > 0.1)
      {
         LogTools.info("%s step at %s".formatted(side.getPascalCaseName(), new Point2D(footPolygon_0_x.getValueAsDouble(), footPolygon_0_y.getValueAsDouble())));
         footsteps.add(new SCS2LogDataFootstep(side, new double[] {footPolygon_0_x.getValueAsDouble(),
                                                                   footPolygon_1_x.getValueAsDouble(),
                                                                   footPolygon_2_x.getValueAsDouble(),
                                                                   footPolygon_3_x.getValueAsDouble(),
                                                                   footPolygon_0_y.getValueAsDouble(),
                                                                   footPolygon_1_y.getValueAsDouble(),
                                                                   footPolygon_2_y.getValueAsDouble(),
                                                                   footPolygon_3_y.getValueAsDouble()}));
         newStep = false;
      }

      // recent footstep
      return !Double.isNaN(fullSupportTime) && currentTime - fullSupportTime < comPlotProximityToFootsteps;
   }

   public ArrayList<SCS2LogDataFootstep> getFootsteps()
   {
      return footsteps;
   }
}
