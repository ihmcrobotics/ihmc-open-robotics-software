package us.ihmc.avatar.logProcessor;

import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.session.log.LogSession;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.ArrayList;

public class SCS2LogLocomotionData
{
   private YoRegistry rootRegistry;
   private YoVariable yoCurrentNumberOfFootsteps;
   private YoVariable leftFootState;
   private YoVariable rightFootState;
   private ConstraintType leftLastFootState = ConstraintType.SWING;
   private ConstraintType rightLastFootState = ConstraintType.SWING;
   private Double leftFullSupportTime = Double.NaN;
   private Double rightFullSupportTime = Double.NaN;
   private boolean newLeftStep = false;
   private boolean newRightStep = false;
   private YoVariable leftFootPolygon_0_x ;
   private YoVariable leftFootPolygon_0_y ;
   private YoVariable leftFootPolygon_1_x ;
   private YoVariable leftFootPolygon_1_y ;
   private YoVariable leftFootPolygon_2_x ;
   private YoVariable leftFootPolygon_2_y ;
   private YoVariable leftFootPolygon_3_x ;
   private YoVariable leftFootPolygon_3_y ;
   private YoVariable rightFootPolygon_0_x;
   private YoVariable rightFootPolygon_0_y;
   private YoVariable rightFootPolygon_1_x;
   private YoVariable rightFootPolygon_1_y;
   private YoVariable rightFootPolygon_2_x;
   private YoVariable rightFootPolygon_2_y;
   private YoVariable rightFootPolygon_3_x;
   private YoVariable rightFootPolygon_3_y;
   private final ArrayList<double[]> leftFootsteps = new ArrayList<>();
   private final ArrayList<double[]> rightFootsteps = new ArrayList<>();

   private final Point2D robotStartLocation = new Point2D();
   private final Point2D currentCenterOfMass = new Point2D();
   private final Point2D lastCenterOfMass = new Point2D(Double.NaN, Double.NaN);
   private YoVariable yoCenterOfMassX;
   private YoVariable yoCenterOfMassY;
   private final double comPlotProximityToFootsteps = 5.0;
   private final double comPlotResolution = 0.1;
   private double lastCoMPlotTime = Double.NaN;
   private final RecyclingArrayList<Point2D> coms = new RecyclingArrayList<>(Point2D::new);
   private boolean requestStopProcessing = false;

   public void setup(LogSession logSession)
   {
      rootRegistry = logSession.getRootRegistry();

      String highLevelController = "root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.";

      String momentumRateControl = highLevelController + "WalkingControllerState.LinearMomentumRateControlModule.";
      yoCenterOfMassX = rootRegistry.findVariable(momentumRateControl + "centerOfMassX");
      yoCenterOfMassY = rootRegistry.findVariable(momentumRateControl + "centerOfMassY");

      String walkingMessageHandler = highLevelController + "HighLevelHumanoidControllerFactory.WalkingMessageHandler.";
      yoCurrentNumberOfFootsteps = rootRegistry.findVariable(walkingMessageHandler + "currentNumberOfFootsteps");
      String feetManager = highLevelController + "HighLevelHumanoidControllerFactory.HighLevelControlManagerFactory.FeetManager.";
      leftFootState = rootRegistry.findVariable(feetManager + "leftFootControlModule.leftFootCurrentState");
      rightFootState = rootRegistry.findVariable(feetManager + "rightFootControlModule.rightFootCurrentState");
      String footPolygonPrefix = highLevelController + "HighLevelHumanoidControllerToolbox.BipedSupportPolygons.";
      leftFootPolygon_0_x  = rootRegistry.findVariable(footPolygonPrefix + "leftFootPolygon_0_x");
      leftFootPolygon_0_y  = rootRegistry.findVariable(footPolygonPrefix + "leftFootPolygon_0_y");
      leftFootPolygon_1_x  = rootRegistry.findVariable(footPolygonPrefix + "leftFootPolygon_1_x");
      leftFootPolygon_1_y  = rootRegistry.findVariable(footPolygonPrefix + "leftFootPolygon_1_y");
      leftFootPolygon_2_x  = rootRegistry.findVariable(footPolygonPrefix + "leftFootPolygon_2_x");
      leftFootPolygon_2_y  = rootRegistry.findVariable(footPolygonPrefix + "leftFootPolygon_2_y");
      leftFootPolygon_3_x  = rootRegistry.findVariable(footPolygonPrefix + "leftFootPolygon_3_x");
      leftFootPolygon_3_y  = rootRegistry.findVariable(footPolygonPrefix + "leftFootPolygon_3_y");
      rightFootPolygon_0_x = rootRegistry.findVariable(footPolygonPrefix + "rightFootPolygon_0_x");
      rightFootPolygon_0_y = rootRegistry.findVariable(footPolygonPrefix + "rightFootPolygon_0_y");
      rightFootPolygon_1_x = rootRegistry.findVariable(footPolygonPrefix + "rightFootPolygon_1_x");
      rightFootPolygon_1_y = rootRegistry.findVariable(footPolygonPrefix + "rightFootPolygon_1_y");
      rightFootPolygon_2_x = rootRegistry.findVariable(footPolygonPrefix + "rightFootPolygon_2_x");
      rightFootPolygon_2_y = rootRegistry.findVariable(footPolygonPrefix + "rightFootPolygon_2_y");
      rightFootPolygon_3_x = rootRegistry.findVariable(footPolygonPrefix + "rightFootPolygon_3_x");
      rightFootPolygon_3_y = rootRegistry.findVariable(footPolygonPrefix + "rightFootPolygon_3_y");

      logSession.addAfterReadCallback(this::afterRead);
   }

   private void afterRead(double currentTime)
   {
      if (requestStopProcessing)
         return;

      currentCenterOfMass.set(yoCenterOfMassX.getValueAsDouble(), yoCenterOfMassY.getValueAsDouble());

      if (leftLastFootState != ConstraintType.FULL && leftFootState.getValueAsString().equals(ConstraintType.FULL.name()))
      {
         newLeftStep = true;
         leftFullSupportTime = currentTime;
      }
      leftLastFootState = leftFootState.getValueAsString().equals("null") ? null : ConstraintType.valueOf(leftFootState.getValueAsString());

      if (rightLastFootState != ConstraintType.FULL && rightFootState.getValueAsString().equals(ConstraintType.FULL.name()))
      {
         newRightStep = true;
         rightFullSupportTime = currentTime;
      }
      rightLastFootState = rightFootState.getValueAsString().equals("null") ? null : ConstraintType.valueOf(rightFootState.getValueAsString());

      if (newLeftStep && currentTime - leftFullSupportTime > 0.1)
      {
         LogTools.info("Left step at {}", new Point2D(leftFootPolygon_0_x.getValueAsDouble(), leftFootPolygon_0_y.getValueAsDouble()));
         leftFootsteps.add(new double[] {leftFootPolygon_0_x.getValueAsDouble(),
                                         leftFootPolygon_1_x.getValueAsDouble(),
                                         leftFootPolygon_2_x.getValueAsDouble(),
                                         leftFootPolygon_3_x.getValueAsDouble(),
                                         leftFootPolygon_0_y.getValueAsDouble(),
                                         leftFootPolygon_1_y.getValueAsDouble(),
                                         leftFootPolygon_2_y.getValueAsDouble(),
                                         leftFootPolygon_3_y.getValueAsDouble()});
         newLeftStep = false;
      }

      if (newRightStep && currentTime - rightFullSupportTime > 0.1)
      {
         LogTools.info("Right step at {}", new Point2D(rightFootPolygon_0_x.getValueAsDouble(), rightFootPolygon_0_y.getValueAsDouble()));
         rightFootsteps.add(new double[] {rightFootPolygon_0_x.getValueAsDouble(),
                                          rightFootPolygon_1_x.getValueAsDouble(),
                                          rightFootPolygon_2_x.getValueAsDouble(),
                                          rightFootPolygon_3_x.getValueAsDouble(),
                                          rightFootPolygon_0_y.getValueAsDouble(),
                                          rightFootPolygon_1_y.getValueAsDouble(),
                                          rightFootPolygon_2_y.getValueAsDouble(),
                                          rightFootPolygon_3_y.getValueAsDouble()});
         newRightStep = false;
      }

      boolean recentLeftStep = !Double.isNaN(leftFullSupportTime) && currentTime - leftFullSupportTime < comPlotProximityToFootsteps;
      boolean recentRightStep = !Double.isNaN(rightFullSupportTime) && currentTime - rightFullSupportTime < comPlotProximityToFootsteps;

      if (recentLeftStep || recentRightStep)
      {
         if (Double.isNaN(lastCoMPlotTime) || currentTime - lastCoMPlotTime > comPlotResolution)
         {
            if (coms.isEmpty())
            {
               robotStartLocation.set(currentCenterOfMass.getX(), currentCenterOfMass.getY());
               LogTools.info("Robot start location: {}", robotStartLocation);
            }

            coms.add().set(currentCenterOfMass);

            lastCenterOfMass.set(currentCenterOfMass);
            lastCoMPlotTime = currentTime;
         }
      }

      // TODO:
      // # Falls
      // # Runs of action (split by 30 seconds of inactivity)
      // Timestamps where runs start
      // Arm motions
   }

   public void requestStopProcessing()
   {
      requestStopProcessing = true;
   }

   public Point2D getRobotStartLocation()
   {
      return robotStartLocation;
   }

   public ArrayList<double[]> getLeftFootsteps()
   {
      return leftFootsteps;
   }

   public ArrayList<double[]> getRightFootsteps()
   {
      return rightFootsteps;
   }

   public RecyclingArrayList<Point2D> getComs()
   {
      return coms;
   }
}
