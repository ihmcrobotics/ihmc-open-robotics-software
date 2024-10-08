package us.ihmc.avatar.logProcessor;

import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.session.log.LogSession;
import us.ihmc.yoVariables.euclid.YoPoint2D;
import us.ihmc.yoVariables.euclid.YoPoint3D;
import us.ihmc.yoVariables.euclid.YoPose3D;
import us.ihmc.yoVariables.euclid.YoQuaternion;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;

public class SCS2LogLocomotionData
{
   private double dt;
   private YoRegistry rootRegistry;
   private int initialWorkingCounterMismatch = -1;
   private YoInteger workingCounterMismatch;
   private YoBoolean isRobotFalling;
   private SCS2LogEnum<HighLevelControllerName> controllerState;
   private final Point2D robotStartLocation = new Point2D(Double.NaN, Double.NaN);
   private final SideDependentList<SCS2LogFootState> footStates = new SideDependentList<>();
   private final ArrayList<SCS2LogWalk> logWalks = new ArrayList<>();
   private final Point2D lastCenterOfMass = new Point2D(Double.NaN, Double.NaN);
   private YoPose3D pelvisPose;
   private YoPoint3D centerOfMass;
   private YoPoint2D capturePoint;
   private final double plotTimeResolution = 0.1;
   private double lastCoMPlotTime = Double.NaN;
   private SideDependentList<ArrayList<SCS2LogJointTracker>> armJointPositions = new SideDependentList<>(new ArrayList<>(), new ArrayList<>());
   private boolean requestStopProcessing = false;

   public void setup(LogSession logSession)
   {
      rootRegistry = logSession.getRootRegistry();

      dt = logSession.getLogDataReader().getParser().getDt();

      if (rootRegistry.findVariable("root.main.DRCEstimatorThread.NadiaEtherCATRealtimeThread.workingCounterMismatch") instanceof YoInteger yoInteger)
         workingCounterMismatch = yoInteger;

      String highLevelController = "root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.";

      if (rootRegistry.findVariable(highLevelController + "highLevelControllerNameCurrentState") instanceof YoEnum<?> yoEnum)
         controllerState = new SCS2LogEnum<>(yoEnum, HighLevelControllerName.class);

      if (rootRegistry.findVariable(highLevelController + "HighLevelHumanoidControllerToolbox.WalkingFailureDetectionControlModule.isRobotFalling") instanceof YoBoolean yoBoolean)
         isRobotFalling = yoBoolean;

      // TODO: These are specific to nadia, how to make general?
      if (rootRegistry.findVariable("root.nadia.q_PELVIS_LINK_x") instanceof YoDouble xVariable
       && rootRegistry.findVariable("root.nadia.q_PELVIS_LINK_y") instanceof YoDouble yVariable
       && rootRegistry.findVariable("root.nadia.q_PELVIS_LINK_z") instanceof YoDouble zVariable
       && rootRegistry.findVariable("root.nadia.q_PELVIS_LINK_qx") instanceof YoDouble qxVariable
       && rootRegistry.findVariable("root.nadia.q_PELVIS_LINK_qy") instanceof YoDouble qyVariable
       && rootRegistry.findVariable("root.nadia.q_PELVIS_LINK_qz") instanceof YoDouble qzVariable
       && rootRegistry.findVariable("root.nadia.q_PELVIS_LINK_qs") instanceof YoDouble qsVariable)
         pelvisPose = new YoPose3D(new YoPoint3D(xVariable, yVariable, zVariable),
                                   new YoQuaternion(qxVariable, qyVariable, qzVariable, qsVariable));

      String momentumRateControl = highLevelController + "WalkingControllerState.LinearMomentumRateControlModule.";
      if (rootRegistry.findVariable(momentumRateControl + "centerOfMassX") instanceof YoDouble xVariable
       && rootRegistry.findVariable(momentumRateControl + "centerOfMassY") instanceof YoDouble yVariable
       && rootRegistry.findVariable(momentumRateControl + "centerOfMassX") instanceof YoDouble zVariable)
         centerOfMass = new YoPoint3D(xVariable, yVariable, zVariable);

      if (rootRegistry.findVariable(momentumRateControl + "capturePointX") instanceof YoDouble xVariable
       && rootRegistry.findVariable(momentumRateControl + "capturePointY") instanceof YoDouble yVariable)
         capturePoint = new YoPoint2D(xVariable, yVariable);
      
      String feetManager = highLevelController + "HighLevelHumanoidControllerFactory.HighLevelControlManagerFactory.FeetManager.";
      for (RobotSide side : RobotSide.values)
         if (rootRegistry.findVariable(feetManager + "%1$sFootControlModule.%1$sFootCurrentState".formatted(side.getLowerCaseName())) instanceof YoEnum<?> yoEnum)
            footStates.set(side, new SCS2LogFootState(side, new SCS2LogEnum<>(yoEnum, ConstraintType.class), rootRegistry));

      String sensorProcessing = "root.main.DRCEstimatorThread.NadiaSensorReader.SensorProcessing.";
      // TODO: These are specific to the robot version. How would you know these generally?
      String[] armJointNames = new String[] { "SHOULDER_Y", "SHOULDER_X", "SHOULDER_Z", "ELBOW_Y", "WRIST_Z", "WRIST_X", "GRIPPER_Z" };
      for (RobotSide side : RobotSide.values)
      {
         for (String armJoint : armJointNames)
         {
            if (rootRegistry.findVariable(sensorProcessing + "raw_q_%s_%s".formatted(side.getSideNameInAllCaps(), armJoint)) instanceof YoDouble yoDouble)
            {
               armJointPositions.get(side).add(new SCS2LogJointTracker(yoDouble));
            }
         }
      }

      logSession.addAfterReadCallback(this::afterRead);
   }

   private void afterRead(double currentTime)
   {
      if (requestStopProcessing)
         return;

      long tick = (long) (currentTime / dt);

      if (initialWorkingCounterMismatch < 0)
         initialWorkingCounterMismatch = workingCounterMismatch.getIntegerValue();

      if (controllerState.changedTo(HighLevelControllerName.WALKING))
      {
         SCS2LogWalk logWalk = new SCS2LogWalk();
         logWalks.add(logWalk);
         logWalk.getWalkStart().set(centerOfMass);
         logWalk.setWalkStartTick(tick);
      }
      if (controllerState.changedFrom(HighLevelControllerName.WALKING) && controllerState.changedTo(HighLevelControllerName.FREEZE_STATE))
      {
         if (isRobotFalling.getBooleanValue())
         {
            getCurrentLogWalk().setEndedWithFall(true);
         }
      }
      controllerState.postUpdate();

      if (!logWalks.isEmpty() && controllerState.getValue() == HighLevelControllerName.WALKING)
      {
         SCS2LogWalk logWalk = getCurrentLogWalk();
         logWalk.update(currentTime, tick, workingCounterMismatch);

         boolean recentSteps = false;
         for (RobotSide side : RobotSide.values)
         {
            recentSteps |= footStates.get(side).afterRead(currentTime);
            logWalk.getFootsteps().addAll(footStates.get(side).getFootsteps());
            footStates.get(side).getFootsteps().clear();
         }

         if (robotStartLocation.containsNaN())
         {
            robotStartLocation.set(centerOfMass.getX(), centerOfMass.getY());
            LogTools.info("Robot start location: {}", robotStartLocation);
         }

         if (lastCenterOfMass.containsNaN())
         {
            recordEntry(currentTime, logWalk);
         }
         else if (centerOfMass.distanceXY(lastCenterOfMass) > 0.001 && currentTime - lastCoMPlotTime > plotTimeResolution)
         {
            recordEntry(currentTime, logWalk);
         }
      }
   }

   private void recordEntry(double currentTime, SCS2LogWalk logWalk)
   {
      logWalk.getTimes().add(currentTime);
      logWalk.getPelvisPoses().add().set(pelvisPose);
      logWalk.getComs().add().set(centerOfMass);
      logWalk.getIcps().add().set(capturePoint);
      lastCenterOfMass.set(centerOfMass);
      lastCoMPlotTime = currentTime;
   }

   public void writeJSON(ObjectNode rootNode)
   {
      rootNode.put("numberOfWalks", logWalks.size());
      rootNode.put("numberOfFalls", getFalls());
      rootNode.put("numberOfFootsteps", getNumberOfFootsteps());
      rootNode.put("numberOfComs", getNumberOfComs());
      rootNode.put("workingCounterMismatch", getWorkingCounterMismatch());
   }

   private SCS2LogWalk getCurrentLogWalk()
   {
      return logWalks.get(logWalks.size() - 1);
   }

   public void requestStopProcessing()
   {
      requestStopProcessing = true;
   }

   public Point2D getRobotStartLocation()
   {
      return robotStartLocation;
   }

   public int getNumberOfFootsteps()
   {
      int numberOfFootsteps = 0;
      for (SCS2LogWalk logWalk : logWalks)
      {
         numberOfFootsteps += logWalk.getFootsteps().size();
      }
      return numberOfFootsteps;
   }

   public int getNumberOfComs()
   {
      int numberOfComs = 0;
      for (SCS2LogWalk logWalk : logWalks)
      {
         numberOfComs += logWalk.getComs().size();
      }
      return numberOfComs;
   }

   public ArrayList<SCS2LogWalk> getLogWalks()
   {
      return logWalks;
   }

   public int getFalls()
   {
      int falls = 0;
      for (SCS2LogWalk logWalk : logWalks)
      {
         if (logWalk.isEndedWithFall())
            ++falls;
      }
      return falls;
   }

   public int getWorkingCounterMismatch()
   {
      if (workingCounterMismatch == null)
         return -1;
      else
         return workingCounterMismatch.getIntegerValue() - initialWorkingCounterMismatch;
   }
}
