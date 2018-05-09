package us.ihmc.quadrupedRobotics.inverseKinematics;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.YoPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultYoPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.PID3DConfiguration;
import us.ihmc.robotics.math.filters.RateLimitedYoFramePoint;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

public class QuadrupedInverseKinematicsController implements RobotController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final WholeBodyControlCoreToolbox controlCoreToolbox;
   private final WholeBodyControllerCore controllerCore;

   private final ControllerCoreCommand controllerCoreCommand;

   private final RobotQuadrant[] quadrants;

   private final QuadrantDependentList<PointFeedbackControlCommand> feedbackControlCommands = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoFramePoint3D> desiredSolePositions = new QuadrantDependentList<>();
   private final QuadrantDependentList<RateLimitedYoFramePoint> limitedDesiredSolePositions = new QuadrantDependentList<>();
   private final YoDouble maxRate = new YoDouble("maxRate", registry);

   private final QuadrupedReferenceFrames referenceFrames;

   private final FramePoint3D desiredPosition = new FramePoint3D();
   private final FrameVector3D desiredVelocity = new FrameVector3D();

   private final YoPID3DGains gains;
   private final YoDouble weight = new YoDouble("footWeight", registry);
   private final YoBoolean setToCurrent = new YoBoolean("setToCurrent", registry);

   private boolean initialized = false;

   public QuadrupedInverseKinematicsController(FullQuadrupedRobotModel fullRobotModel, RobotQuadrant[] quadrants, QuadrupedReferenceFrames referenceFrames,
                                               JointDesiredOutputList lowLevelJointOutputList, ControllerCoreOptimizationSettings optimizationSettings,
                                               double controlDT, double gravityZ, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.quadrants = quadrants;
      this.referenceFrames = referenceFrames;

      for (RobotQuadrant robotQuadrant : quadrants)
      {
         RigidBody foot = fullRobotModel.getFoot(robotQuadrant);
         ReferenceFrame soleFrame = referenceFrames.getSoleFrame(robotQuadrant);
         RigidBody body = fullRobotModel.getBody();

         FramePoint3D currentPosition = new FramePoint3D(soleFrame);
         currentPosition.changeFrame(foot.getBodyFixedFrame());

         PointFeedbackControlCommand feedbackControlCommand = new PointFeedbackControlCommand();
         feedbackControlCommand.set(body, foot);
         feedbackControlCommand.setBodyFixedPointToControl(currentPosition);
         feedbackControlCommands.put(robotQuadrant, feedbackControlCommand);

         RigidBody rigidBody = foot;
         while (rigidBody.getParentJoint().getPredecessor() != body)
            rigidBody = rigidBody.getParentJoint().getPredecessor();
         ReferenceFrame shoulderBaseFrame = rigidBody.getParentJoint().getFrameBeforeJoint();
         YoFramePoint3D desiredPosition = new YoFramePoint3D(robotQuadrant.getCamelCaseName() + "_Foot_DesiredPosition", shoulderBaseFrame, registry);
         RateLimitedYoFramePoint limitedDesiredPosition = new RateLimitedYoFramePoint(robotQuadrant.getCamelCaseName() + "_LimitedFoot_DesiredPosition", "",
                                                                                      registry, maxRate, controlDT, desiredPosition);
         desiredSolePositions.put(robotQuadrant, desiredPosition);
         limitedDesiredSolePositions.put(robotQuadrant, limitedDesiredPosition);
      }

      PID3DConfiguration gainConfiguration = new PID3DConfiguration(GainCoupling.XY, false);
      gains = new DefaultYoPID3DGains("footGains", gainConfiguration, registry);
      gains.setProportionalGains(10.0);
      gains.setDerivativeGains(1.0);

      weight.set(1.0);
      maxRate.set(0.1);

      controlCoreToolbox = new WholeBodyControlCoreToolbox(controlDT, gravityZ, null, fullRobotModel.getControllableOneDoFJoints(),
                                                           referenceFrames.getCenterOfMassFrame(), optimizationSettings, graphicsListRegistry, registry);
      controlCoreToolbox.setupForInverseKinematicsSolver();
      controllerCore = new WholeBodyControllerCore(controlCoreToolbox, getFeedbackCommandTemplate(), lowLevelJointOutputList, registry);
      controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.INVERSE_KINEMATICS);
   }

   @Override
   public void doControl()
   {
      referenceFrames.updateFrames();

      if (!initialized)
      {
         initialize();
         initialized = true;
      }

      if (setToCurrent.getBooleanValue())
         setToCurrent();

      controllerCoreCommand.clear();

      for (RobotQuadrant robotQuadrant : quadrants)
      {
         limitedDesiredSolePositions.get(robotQuadrant).update();
         desiredPosition.setIncludingFrame(limitedDesiredSolePositions.get(robotQuadrant));
         desiredPosition.changeFrame(ReferenceFrame.getWorldFrame());

         PointFeedbackControlCommand feedbackControlCommand = feedbackControlCommands.get(robotQuadrant);
         feedbackControlCommand.set(desiredPosition, desiredVelocity);
         feedbackControlCommand.setGains(gains);
         feedbackControlCommand.setWeightForSolver(weight.getDoubleValue());

         controllerCoreCommand.addFeedbackControlCommand(feedbackControlCommand);
      }

      controllerCore.submitControllerCoreCommand(controllerCoreCommand);
      controllerCore.compute();
   }

   @Override
   public void initialize()
   {
      controllerCore.initialize();

      setToCurrent();
   }

   private void setToCurrent()
   {
      setToCurrent.set(false);

      for (RobotQuadrant robotQuadrant : quadrants)
      {
         desiredPosition.setToZero(referenceFrames.getSoleFrame(robotQuadrant));
         desiredSolePositions.get(robotQuadrant).setMatchingFrame(desiredPosition);
         limitedDesiredSolePositions.get(robotQuadrant).setMatchingFrame(desiredPosition);
      }
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return getClass().getSimpleName();
   }

   @Override
   public String getDescription()
   {
      return getName();
   }

   private FeedbackControlCommandList getFeedbackCommandTemplate()
   {
      FeedbackControlCommandList feedbackControlCommandList = new FeedbackControlCommandList();
      for (RobotQuadrant robotQuadrant : quadrants)
      {
         feedbackControlCommandList.addCommand(feedbackControlCommands.get(robotQuadrant));
      }
      return feedbackControlCommandList;
   }
}
