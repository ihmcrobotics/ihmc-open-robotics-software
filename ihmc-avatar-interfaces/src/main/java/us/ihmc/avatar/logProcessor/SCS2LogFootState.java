package us.ihmc.avatar.logProcessor;

import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.ArrayList;

/** Data holder for YoVariable information relating to foot state for data post-processing. */
public class SCS2LogFootState
{
   private final RobotSide side;
   private final SCS2LogEnum<ConstraintType> yoFootState;
   private boolean newStep = false;
   private double fullSupportTime = Double.NaN;
   private final YoVariable footPolygon_0_x;
   private final YoVariable footPolygon_0_y;
   private final YoVariable footPolygon_1_x;
   private final YoVariable footPolygon_1_y;
   private final YoVariable footPolygon_2_x;
   private final YoVariable footPolygon_2_y;
   private final YoVariable footPolygon_3_x;
   private final YoVariable footPolygon_3_y;
   private final ArrayList<SCS2LogFootstep> footsteps = new ArrayList<>();
   private final TypedNotification<ConstraintType> stateChanged = new TypedNotification<>();
   private final double comPlotProximityToFootsteps = 5.0;
   private double timeStartedSwing = Double.NaN;
   private final TypedNotification<Double> swingCompleted = new TypedNotification<>();

   public SCS2LogFootState(RobotSide side, SCS2LogEnum<ConstraintType> yoFootState, YoRegistry rootRegistry)
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

   public void afterRead(double currentTime)
   {
      if (yoFootState.changed())
      {
         stateChanged.set(yoFootState.getValue());
      }
      if (yoFootState.changedTo(ConstraintType.FULL))
      {
         newStep = true;
         fullSupportTime = currentTime;
      }
      if (yoFootState.changedTo(ConstraintType.SWING))
      {
         timeStartedSwing = currentTime;
      }
      if (yoFootState.changedFrom(ConstraintType.SWING))
      {
         double swingDuration = currentTime - timeStartedSwing;
         swingCompleted.set(swingDuration);
      }
      yoFootState.postUpdate();

      if (newStep && currentTime - fullSupportTime > 0.1)
      {
         LogTools.info("%s step at %s".formatted(side.getPascalCaseName(), new Point2D(footPolygon_0_x.getValueAsDouble(), footPolygon_0_y.getValueAsDouble())));
         footsteps.add(new SCS2LogFootstep(currentTime, side, new double[] {footPolygon_0_x.getValueAsDouble(),
                                                                            footPolygon_1_x.getValueAsDouble(),
                                                                            footPolygon_2_x.getValueAsDouble(),
                                                                            footPolygon_3_x.getValueAsDouble(),
                                                                            footPolygon_0_y.getValueAsDouble(),
                                                                            footPolygon_1_y.getValueAsDouble(),
                                                                            footPolygon_2_y.getValueAsDouble(),
                                                                            footPolygon_3_y.getValueAsDouble()}));
         newStep = false;
      }
   }

   public RobotSide getSide()
   {
      return side;
   }

   public ArrayList<SCS2LogFootstep> getFootsteps()
   {
      return footsteps;
   }

   public TypedNotification<ConstraintType> getStateChanged()
   {
      return stateChanged;
   }

   public TypedNotification<Double> getSwingCompleted()
   {
      return swingCompleted;
   }
}
