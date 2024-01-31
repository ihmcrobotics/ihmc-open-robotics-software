package us.ihmc.commonWalkingControlModules.controlModules.pelvis;

import controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OneDoFJointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.pidGains.PDGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.PDGains;
import us.ihmc.robotics.controllers.pidGains.implementations.ParameterizedPDGains;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class HeightThroughKneeControlState implements PelvisAndCenterOfMassHeightControlState
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final DoubleParameter heightFromKneeKp = new DoubleParameter("heightFromKneeKp", registry, 0.0);
   private final DoubleParameter desiredKneeAngleForHeightControl = new DoubleParameter("desiredKneeAngleForHeightControl", registry, 0.0);
   private final YoDouble straightestKneeAngle = new YoDouble("straightestKneeAngle", registry);
   private final YoDouble desiredHeightFromKneeControl = new YoDouble("desiredHeightFromKneeControl", registry);
   private final YoDouble currentHeightFromKneeControl = new YoDouble("currentHeightFromKneeControl", registry);
   private final DoubleParameter maximumHeightChangeFromKneeControl = new DoubleParameter("maximumHeightChangeFromKneeControl", registry, 0.02);
   private final YoDouble heightChangeFromKneeControl = new YoDouble("heightChangeFromKneeControl", registry);

   private final YoEnum<RobotSide> kneeSideToControl = new YoEnum<>("kneeSideToControl", registry, RobotSide.class);
   private final YoEnum<RobotSide> supportLegSide = new YoEnum<>("kneeControlSupportLegSide", registry, RobotSide.class);

   private final MovingReferenceFrame pelvisFrame;

   private final FeedbackControlCommandList feedbackCommandList = new FeedbackControlCommandList();
   private final PointFeedbackControlCommand pelvisHeightControlCommand = new PointFeedbackControlCommand();
   private final SideDependentList<OneDoFJointFeedbackControlCommand> kneeControlCommands = new SideDependentList<>();

   private final DoubleParameter supportKneeWeight;
   private final DoubleParameter nonSupportKneeWeight;
   private final ParameterizedPDGains kneeGains;

   private Vector3DReadOnly pelvisTaskpaceFeedbackWeight;
   private final FullHumanoidRobotModel fullRobotModel;

   private final SideDependentList<OneDoFJointBasics> kneeJoints = new SideDependentList<>();
   private final SideDependentList<YoDouble> desiredSupportKneeAngles = new SideDependentList<>();

   private final FramePoint3D desiredPosition = new FramePoint3D();
   private final FrameVector3D desiredVelocity = new FrameVector3D();
   private final FrameVector3D desiredAcceleration = new FrameVector3D();
   private PDGainsReadOnly pelvisHeightGains;
   // Temporary objects to reduce garbage collection.

   private final FramePoint3D pelvisPosition = new FramePoint3D();

   public HeightThroughKneeControlState(HighLevelHumanoidControllerToolbox controllerToolbox,
                                        WalkingControllerParameters walkingControllerParameters,
                                        YoRegistry parentRegistry)
   {
      CommonHumanoidReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
      fullRobotModel = controllerToolbox.getFullRobotModel();

      PDGains tempPDGains = new PDGains();
      tempPDGains.setKp(100.0);
      tempPDGains.setZeta(0.7);
      kneeGains = new ParameterizedPDGains("kneeHeightControlGains", tempPDGains, registry);
      supportKneeWeight = new DoubleParameter("supportKneeHeightControlWeight", registry, 10.0);
      nonSupportKneeWeight = new DoubleParameter("nonSupportKneeHeightControlWeight", registry, 1.0);

      for (RobotSide robotSide : RobotSide.values)
      {
         OneDoFJointBasics kneeJoint = fullRobotModel.getLegJoint(robotSide, LegJointName.KNEE_PITCH);
         kneeJoints.put(robotSide, kneeJoint);

         YoDouble desiredSupportKneeAngle = new YoDouble(robotSide.getCamelCaseName() + "DesiredSupportKneeAngle", registry);
         desiredSupportKneeAngle.set(0.5);
         desiredSupportKneeAngles.put(robotSide, desiredSupportKneeAngle);

         OneDoFJointFeedbackControlCommand kneeControlCommand = new OneDoFJointFeedbackControlCommand();
         kneeControlCommand.setJoint(kneeJoint);
         kneeControlCommand.setControlMode(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);

         kneeControlCommands.put(robotSide, kneeControlCommand);
      }

      pelvisFrame = referenceFrames.getPelvisFrame();

      SelectionMatrix3D selectionMatrix = new SelectionMatrix3D(worldFrame, false, false, true);
      pelvisHeightControlCommand.set(fullRobotModel.getElevator(), fullRobotModel.getPelvis());
      FramePoint3D pelvisPoint = new FramePoint3D(pelvisFrame);
      pelvisPoint.changeFrame(fullRobotModel.getPelvis().getBodyFixedFrame());
      pelvisHeightControlCommand.setBodyFixedPointToControl(pelvisPoint);
      pelvisHeightControlCommand.setSelectionMatrix(selectionMatrix);


      parentRegistry.addChild(registry);
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public void initializeDesiredHeightToCurrent()
   {
   }

   @Override
   public void goHome(double trajectoryTime)
   {
   }

   @Override
   public void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command)
   {
   }

   public void setSupportLeg(RobotSide supportLeg)
   {
      supportLegSide.set(supportLeg);
   }





   @Override
   public void computeCoMHeightCommand(FrameVector2DReadOnly desiredICPVelocity,
                                       FrameVector2DReadOnly desiredCoMVelocity,
                                       boolean isInDoubleSupport,
                                       double omega0,
                                       FeetManager feetManager)
   {


      if (isInDoubleSupport)
      {
         OneDoFJointBasics leftKnee = kneeJoints.get(RobotSide.LEFT);
         OneDoFJointBasics rightKnee = kneeJoints.get(RobotSide.RIGHT);

         double leftJointAngle = leftKnee.getQ();
         double rightJointAngle = rightKnee.getQ();

         if (leftJointAngle < rightJointAngle)
            kneeSideToControl.set(RobotSide.LEFT);
         else
            kneeSideToControl.set(RobotSide.RIGHT);

         straightestKneeAngle.set(Math.min(leftJointAngle, rightJointAngle));
      }
      else
      {
         OneDoFJointBasics supportKnee = kneeJoints.get(supportLegSide.getValue());
         straightestKneeAngle.set(supportKnee.getQ());
         kneeSideToControl.set(supportLegSide.getValue());
      }

      pelvisPosition.setToZero(pelvisFrame);
      pelvisPosition.changeFrame(worldFrame);
      currentHeightFromKneeControl.set(pelvisPosition.getZ());

      // FIXME This never actually gets used. So the control is being done to achieve the current position, and zero velocity, which is just viscous damping.
      double control = -heightFromKneeKp.getValue() * (desiredKneeAngleForHeightControl.getValue() - straightestKneeAngle.getValue());
      heightChangeFromKneeControl.set(MathTools.clamp(control, maximumHeightChangeFromKneeControl.getValue()));

      desiredHeightFromKneeControl.set(currentHeightFromKneeControl.getValue() + heightChangeFromKneeControl.getValue());

      desiredPosition.set(0.0, 0.0, desiredHeightFromKneeControl.getValue());
      desiredVelocity.setToZero();
      desiredAcceleration.setToZero();



      updateGains();
      // FIXME this never gets weights. So it doesn't do anything?
      pelvisHeightControlCommand.setInverseDynamics(desiredPosition, desiredVelocity, desiredAcceleration);

      OneDoFJointFeedbackControlCommand supportKneeControlCommand = kneeControlCommands.get(kneeSideToControl.getValue());
      supportKneeControlCommand.setWeightForSolver(supportKneeWeight.getValue());
      supportKneeControlCommand.setInverseDynamics(desiredSupportKneeAngles.get(kneeSideToControl.getValue()).getValue(), 0.0, 0.0);
      supportKneeControlCommand.setGains(kneeGains);

      feedbackCommandList.clear();
      feedbackCommandList.addCommand(supportKneeControlCommand);

      if (isInDoubleSupport)
      {
         OneDoFJointFeedbackControlCommand nonSupportKneeControlCommand = kneeControlCommands.get(kneeSideToControl.getValue().getOppositeSide());
         nonSupportKneeControlCommand.setGains(kneeGains);

         nonSupportKneeControlCommand.setWeightForSolver(nonSupportKneeWeight.getValue());
         nonSupportKneeControlCommand.setInverseDynamics(desiredSupportKneeAngles.get(kneeSideToControl.getValue().getOppositeSide()).getValue(), 0.0, 0.0);

         feedbackCommandList.addCommand(nonSupportKneeControlCommand);
      }
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return feedbackCommandList;
   }

   @Override
   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      FeedbackControlCommandList list = new FeedbackControlCommandList();
      for (RobotSide robotSide : RobotSide.values)
         list.addCommand(kneeControlCommands.get(robotSide));

      return list;
   }

   @Override
   public FeedbackControlCommand<?> getHeightControlCommand()
   {
      return pelvisHeightControlCommand;
   }

   @Override
   public boolean getControlHeightWithMomentum()
   {
      return false;
   }

   @Override
   public void doAction(double timeInState)
   {
   }

   public void setPelvisHeightGains(PDGainsReadOnly pelvisHeightGains)
   {
      this.pelvisHeightGains = pelvisHeightGains;
   }

   private final DefaultPID3DGains pelvisGainsTemp = new DefaultPID3DGains();

   public void updateGains()
   {
      pelvisGainsTemp.setProportionalGains(0.0, 0.0, pelvisHeightGains.getKp());
      pelvisGainsTemp.setDerivativeGains(0.0, 0.0, pelvisHeightGains.getKd());
      pelvisGainsTemp.setMaxFeedbackAndFeedbackRate(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);

      pelvisHeightControlCommand.setGains(pelvisGainsTemp);
      pelvisHeightControlCommand.getSpatialAccelerationCommand().getWeightMatrix().setWeightFrames(null, worldFrame);
      pelvisHeightControlCommand.getSpatialAccelerationCommand().getWeightMatrix().setAngularWeights(0.0, 0.0, 0.0);
      pelvisHeightControlCommand.getSpatialAccelerationCommand().getWeightMatrix().getLinearPart().set(pelvisTaskpaceFeedbackWeight);
   }

   @Override
   public TaskspaceTrajectoryStatusMessage pollStatusToReport()
   {
      return null;
   }

   public void setWeights(Vector3DReadOnly weight)
   {
      this.pelvisTaskpaceFeedbackWeight = weight;
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      return null;
   }
}
