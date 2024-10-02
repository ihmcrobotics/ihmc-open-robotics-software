package us.ihmc.avatar.logProcessor;

import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.session.log.LogSession;
import us.ihmc.yoVariables.euclid.YoPoint3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class SCS2LogLocomotionData
{
   private YoRegistry rootRegistry;
   private SCS2LogDataEnum<HighLevelControllerName> controllerState;
   private final SideDependentList<SCS2LogDataFootstep> footStates = new SideDependentList<>();

   private final Point2D robotStartLocation = new Point2D();
   private final Point2D lastCenterOfMass = new Point2D(Double.NaN, Double.NaN);
   private YoPoint3D yoCenterOfMass;
   private final double comPlotProximityToFootsteps = 5.0;
   private final double comPlotResolution = 0.1;
   private double lastCoMPlotTime = Double.NaN;
   private final RecyclingArrayList<Point2D> coms = new RecyclingArrayList<>(Point2D::new);
   private boolean requestStopProcessing = false;

   public void setup(LogSession logSession)
   {
      rootRegistry = logSession.getRootRegistry();

      String highLevelController = "root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.";

      if (rootRegistry.findVariable(highLevelController + "highLevelControllerNameCurrentState") instanceof YoEnum<?> yoEnum)
         controllerState = new SCS2LogDataEnum<>(yoEnum, HighLevelControllerName.class);

      String momentumRateControl = highLevelController + "WalkingControllerState.LinearMomentumRateControlModule.";
      if (rootRegistry.findVariable(momentumRateControl + "centerOfMassX") instanceof YoDouble xVariable
       && rootRegistry.findVariable(momentumRateControl + "centerOfMassY") instanceof YoDouble yVariable
       && rootRegistry.findVariable(momentumRateControl + "centerOfMassX") instanceof YoDouble zVariable)
         yoCenterOfMass = new YoPoint3D(xVariable, yVariable, zVariable);
      
      String feetManager = highLevelController + "HighLevelHumanoidControllerFactory.HighLevelControlManagerFactory.FeetManager.";
      for (RobotSide side : RobotSide.values)
         if (rootRegistry.findVariable(feetManager + "%1$sFootControlModule.%1$sFootCurrentState".formatted(side.getLowerCaseName())) instanceof YoEnum<?> yoEnum)
            footStates.set(side, new SCS2LogDataFootstep(side, new SCS2LogDataEnum<>(yoEnum, ConstraintType.class), rootRegistry));

      logSession.addAfterReadCallback(this::afterRead);
   }

   private void afterRead(double currentTime)
   {
      if (requestStopProcessing)
         return;

      boolean recentSteps = false;
      for (RobotSide side : RobotSide.values)
      {
         recentSteps |= footStates.get(side).afterRead(currentTime);
      }

      if (recentSteps)
      {
         if (Double.isNaN(lastCoMPlotTime) || currentTime - lastCoMPlotTime > comPlotResolution)
         {
            if (coms.isEmpty())
            {
               robotStartLocation.set(yoCenterOfMass.getX(), yoCenterOfMass.getY());
               LogTools.info("Robot start location: {}", robotStartLocation);
            }

            coms.add().set(yoCenterOfMass);

            lastCenterOfMass.set(yoCenterOfMass);
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

   public SideDependentList<SCS2LogDataFootstep> getFootStates()
   {
      return footStates;
   }

   public RecyclingArrayList<Point2D> getComs()
   {
      return coms;
   }
}
