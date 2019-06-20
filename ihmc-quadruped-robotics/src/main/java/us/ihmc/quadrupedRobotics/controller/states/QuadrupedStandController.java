package us.ihmc.quadrupedRobotics.controller.states;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedBalanceManager;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedBodyOrientationManager;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedControlManagerFactory;
import us.ihmc.quadrupedRobotics.controlModules.foot.QuadrupedFeetManager;
import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.robotics.stateMachine.extra.EventState;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class QuadrupedStandController implements EventState
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ReferenceFrame supportFrame;

   private final QuadrupedBodyOrientationManager bodyOrientationManager;
   private final QuadrupedFeetManager feetManager;
   private final QuadrupedBalanceManager balanceManager;
   private final QuadrupedControllerToolbox controllerToolbox;

   public QuadrupedStandController(QuadrupedControllerToolbox controllerToolbox, QuadrupedControlManagerFactory controlManagerFactory,
                                   YoVariableRegistry parentRegistry)
   {
      this.controllerToolbox = controllerToolbox;
      // frames
      QuadrupedReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
      supportFrame = referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();

      // feedback controllers
      feetManager = controlManagerFactory.getOrCreateFeetManager();
      bodyOrientationManager = controlManagerFactory.getOrCreateBodyOrientationManager();
      balanceManager = controlManagerFactory.getOrCreateBalanceManager();

      parentRegistry.addChild(registry);
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public void doAction(double timeInState)
   {
      // update desired horizontal com forces
      balanceManager.compute();
      controllerToolbox.updateSupportPolygon();

      // update desired body orientation, angular velocity, and torque
      bodyOrientationManager.compute();

      // update desired contact state and sole forces
      feetManager.compute();
   }

   @Override
   public ControllerEvent fireEvent(double timeInState)
   {
      return null;
   }

   @Override
   public void onEntry()
   {
      bodyOrientationManager.setDesiredFrameToHoldPosition(supportFrame);
      bodyOrientationManager.enableBodyPitchOscillation();

      // initialize feedback controllers
      balanceManager.initializeForStanding();
      balanceManager.enableBodyXYControl();
      bodyOrientationManager.initialize();

      feetManager.requestFullContact();
      this.controllerToolbox.getRuntimeEnvironment().getRobotMotionStatusHolder().setCurrentRobotMotionStatus(RobotMotionStatus.STANDING);
   }

   @Override
   public void onExit()
   {
      bodyOrientationManager.disableBodyPitchOscillation();
      balanceManager.disableBodyXYControl();
      this.controllerToolbox.getRuntimeEnvironment().getRobotMotionStatusHolder().setCurrentRobotMotionStatus(RobotMotionStatus.STANDING);
   }
}
