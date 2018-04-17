package us.ihmc.quadrupedRobotics.controller.states;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedBalanceManager;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedBodyOrientationManager;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedControlManagerFactory;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedJointSpaceManager;
import us.ihmc.quadrupedRobotics.controlModules.foot.QuadrupedFeetManager;
import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.toolbox.QuadrupedTaskSpaceEstimates;
import us.ihmc.quadrupedRobotics.estimator.GroundPlaneEstimator;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class QuadrupedStandController implements QuadrupedController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   // frames
   private final ReferenceFrame supportFrame;

   // feedback controllers
   private final QuadrupedBodyOrientationManager bodyOrientationManager;
   private final QuadrupedFeetManager feetManager;
   private final QuadrupedBalanceManager balanceManager;
   private final QuadrupedJointSpaceManager jointSpaceManager;

   // task space controller
   private final QuadrantDependentList<ContactState> contactStates = new QuadrantDependentList<>();

   // planning
   private final GroundPlaneEstimator groundPlaneEstimator;

   private final QuadrupedControllerToolbox controllerToolbox;

   private final QuadrantDependentList<FramePoint3D> solePositions;

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
      jointSpaceManager = controlManagerFactory.getOrCreateJointSpaceManager();

      // planning
      groundPlaneEstimator = controllerToolbox.getGroundPlaneEstimator();

      solePositions = controllerToolbox.getTaskSpaceEstimates().getSolePositions();

      parentRegistry.addChild(registry);
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public void doAction(double timeInState)
   {
      controllerToolbox.update();
      feetManager.updateSupportPolygon();

      // update ground plane estimate
      groundPlaneEstimator.compute(solePositions);

      // update desired dcm, com position
      balanceManager.compute(contactStates);

      // update desired body orientation and angular rate
      bodyOrientationManager.compute();

      jointSpaceManager.compute();

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
      // update task space estimates
      controllerToolbox.update();
      QuadrupedTaskSpaceEstimates taskSpaceEstimates = controllerToolbox.getTaskSpaceEstimates();

      // update ground plane estimate
      groundPlaneEstimator.compute(solePositions);

      bodyOrientationManager.setDesiredFrameToHoldPosition(supportFrame);

      // initialize feedback controllers
      balanceManager.initializeForStanding();
      bodyOrientationManager.initialize(taskSpaceEstimates.getBodyOrientation());

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         contactStates.put(robotQuadrant, ContactState.IN_CONTACT);

      feetManager.requestFullContact();
   }

   @Override
   public void onExit()
   {
   }
}
