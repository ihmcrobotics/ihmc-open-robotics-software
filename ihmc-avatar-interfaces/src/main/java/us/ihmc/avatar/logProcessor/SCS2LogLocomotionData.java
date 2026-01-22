package us.ihmc.avatar.logProcessor;

import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.avatar.logProcessor.SCS2LogWalk.DoubleSupportDuration;
import us.ihmc.avatar.logProcessor.SCS2LogWalk.FootStateChange;
import us.ihmc.avatar.logProcessor.SCS2LogWalk.FootSwing;
import us.ihmc.avatar.logProcessor.SCS2LogWalk.ICPErrorEntry;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.session.log.LogSession;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimRigidBodyBasics;
import us.ihmc.yoVariables.euclid.YoPoint2D;
import us.ihmc.yoVariables.euclid.YoPoint3D;
import us.ihmc.yoVariables.euclid.YoPose3D;
import us.ihmc.yoVariables.euclid.YoQuaternion;
import us.ihmc.yoVariables.euclid.YoVector2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;

/**
 * Scrubs a log and loads locomotion data into data structures that support processing
 * the data for presentation in scientific publications.
 */
public class SCS2LogLocomotionData
{
   private double dt;
   private YoRegistry rootRegistry;
   private int initialWorkingCounterMismatch = -1;
   private YoInteger workingCounterMismatch;
   private YoBoolean isRobotFalling;
   private SCS2LogEnum<HighLevelControllerName> controllerState;
   private final Point2D robotStartLocation = new Point2D(Double.NaN, Double.NaN);
   private YoDouble controllerSwingDuration;
   private SideDependentList<YoDouble> controllerTransferDuration = new SideDependentList<>();
   private final SideDependentList<SCS2LogFootState> footStates = new SideDependentList<>();
   private final ArrayList<SCS2LogWalk> logWalks = new ArrayList<>();
   private final Point2D lastCenterOfMass = new Point2D(Double.NaN, Double.NaN);
   private YoPose3D pelvisPose;
   private YoPoint3D centerOfMass;
   private YoPoint2D capturePoint;
   private YoVector2D capturePointError;
   private final double plotTimeResolution = 0.1;
   private double lastCoMPlotTime = Double.NaN;
   private final SideDependentList<ArrayList<SCS2LogJointTracker>> armJointPositions = new SideDependentList<>(new ArrayList<>(), new ArrayList<>());
   private final SideDependentList<ReferenceFrame> handFrames = new SideDependentList<>();
   private final SideDependentList<ReferenceFrame> footFrames = new SideDependentList<>();
   private boolean requestStopProcessing = false;
   private Robot robot;
   private final SideDependentList<Boolean> isInSupport = new SideDependentList<>(true, true);
   private double timeInDoubleSupport = Double.NaN;

   public void setup(LogSession logSession)
   {
      rootRegistry = logSession.getRootRegistry();

      dt = logSession.getLogDataReader().getDt();

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
      if (rootRegistry.findVariable(momentumRateControl + "ICPController.controllerICPErrorX") instanceof YoDouble xVariable
       && rootRegistry.findVariable(momentumRateControl + "ICPController.controllerICPErrorY") instanceof YoDouble yVariable)
         capturePointError = new YoVector2D(xVariable, yVariable);

      String balanceManager = highLevelController + "HighLevelHumanoidControllerFactory.HighLevelControlManagerFactory.BalanceManager.";
      if (rootRegistry.findVariable(balanceManager + "ErrorBasedStepAdjustmentController.controllerSwingDuration") instanceof YoDouble yoDouble)
         controllerSwingDuration = yoDouble;

      String walkingController = highLevelController + "WalkingControllerState.WalkingHighLevelHumanoidController.";
      for (RobotSide side : RobotSide.values)
      {
         String currentTransferDuration = walkingController + "ToWalking%sSupport.CurrentTransferDuration".formatted(side.getPascalCaseName());
         if (rootRegistry.findVariable(currentTransferDuration) instanceof YoDouble yoDouble)
            controllerTransferDuration.put(side, yoDouble);
      }

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

      robot = logSession.getRobots().get(0);
      for (RobotSide side : RobotSide.values)
      {
         SimRigidBodyBasics handLink = robot.getRigidBody("%s_GRIPPER_YAW_LINK".formatted(side.getSideNameInAllCaps()));
         if (handLink != null) // Robot might not have a hand on this side
         {
            MovingReferenceFrame handFrame = handLink.getParentJoint().getFrameAfterJoint();
            handFrames.set(side, handFrame);
         }

         SimRigidBodyBasics footLink = robot.getRigidBody("%s_FOOT_LINK".formatted(side.getSideNameInAllCaps()));
         MovingReferenceFrame footFrame = footLink.getParentJoint().getFrameAfterJoint();
         footFrames.set(side, footFrame);
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

         logWalk.getICPErrors().add(new ICPErrorEntry(currentTime, new Vector2D(capturePointError)));

         for (SCS2LogFootState footState : footStates.values())
         {
            footState.afterRead(currentTime);
            ConstraintType changedTo = footState.getStateChanged().peek();
            logWalk.getFootsteps().addAll(footState.getFootsteps());
            footState.getFootsteps().clear();

            if (footState.getStateChanged().poll())
               logWalk.getFootStateChanges().get(footState.getSide()).add(new FootStateChange(currentTime, footState.getStateChanged().read()));

            if (footState.getSwingCompleted().poll())
               logWalk.getFootSwings().get(footState.getSide()).add(new FootSwing(currentTime,
                                                                                  footState.getSwingCompleted().read(),
                                                                                  controllerSwingDuration.getDoubleValue()));

            if (changedTo != null)
            {
               isInSupport.put(footState.getSide(), changedTo == ConstraintType.FULL || changedTo == ConstraintType.TOES);

               if (isInSupport.get(RobotSide.LEFT) && isInSupport.get(RobotSide.RIGHT))
               {
                  timeInDoubleSupport = currentTime;
               }
               else
               {
                  double duration = currentTime - timeInDoubleSupport;
                  logWalk.getDoubleSupportDurations().add(new DoubleSupportDuration(currentTime,
                                                                                    duration,
                                                                                    controllerTransferDuration.get(footState.getSide()).getDoubleValue()));
                  timeInDoubleSupport = Double.NaN;
               }
            }
         }

         if (robotStartLocation.containsNaN())
         {
            robotStartLocation.set(centerOfMass.getX(), centerOfMass.getY());
            LogTools.info("Robot start location: {}", robotStartLocation);
         }

         if (lastCenterOfMass.containsNaN() ||
            (centerOfMass.distanceXY(lastCenterOfMass) > 0.001 && currentTime - lastCoMPlotTime > plotTimeResolution))
         {
            recordEntry(currentTime, logWalk);
         }
      }
   }

   private void recordEntry(double currentTime, SCS2LogWalk logWalk)
   {
      robot.updateFrames();

      logWalk.getTimes().add(currentTime);
      logWalk.getPelvisPoses().add().set(pelvisPose);
      for (RobotSide side : handFrames.sides())
         logWalk.getHandPoses().get(side).add().set(handFrames.get(side).getTransformToDesiredFrame(footFrames.get(side)));
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

   public SideDependentList<ReferenceFrame> getHandFrames()
   {
      return handFrames;
   }
}
