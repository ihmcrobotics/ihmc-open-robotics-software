package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.states;

import java.util.Map;

import us.ihmc.commonWalkingControlModules.controlModules.flight.CentroidalMomentumManager;
import us.ihmc.commonWalkingControlModules.controlModules.flight.FeetJumpManager;
import us.ihmc.commonWalkingControlModules.controlModules.flight.GravityCompensationManager;
import us.ihmc.commonWalkingControlModules.controlModules.flight.JumpMessageHandler;
import us.ihmc.commonWalkingControlModules.controlModules.flight.WholeBodyMotionPlanner;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlManager;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;

public class FlightState extends AbstractJumpState
{
   private static final JumpStateEnum stateEnum = JumpStateEnum.FLIGHT;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final CentroidalMomentumManager wholeBodyMomentumManager;
   private final GravityCompensationManager gravityCompensationManager;
   private final FeetJumpManager feetManager;
   private final SideDependentList<RigidBodyControlManager> handManagers;
   private final RigidBodyControlManager chestManager;
   private final RigidBodyControlManager headManager;
   private final SideDependentList<FootSwitchInterface> footSwitches;
   private final YoInteger groundContactForceAboveThresholdCount;
   private final int glitchFilterWindowSize = 5;
   private final double contactThreshold = 400.0;
   private final boolean[] glitchCount = new boolean[glitchFilterWindowSize];

   public FlightState(WholeBodyMotionPlanner motionPlanner, JumpMessageHandler messageHandler, HighLevelHumanoidControllerToolbox controllerToolbox,
                      WholeBodyControlCoreToolbox controlCoreToolbox, CentroidalMomentumManager centroidalMomentumManager,
                      GravityCompensationManager gravityCompensationManager, SideDependentList<RigidBodyControlManager> handManagers,
                      FeetJumpManager feetManager, Map<String, RigidBodyControlManager> bodyManagerMap, YoVariableRegistry registry)
   {
      super(stateEnum, motionPlanner, messageHandler, controllerToolbox);
      this.controllerToolbox = controllerToolbox;
      this.wholeBodyMomentumManager = centroidalMomentumManager;
      this.gravityCompensationManager = gravityCompensationManager;
      this.feetManager = feetManager;
      this.handManagers = handManagers;
      this.chestManager = bodyManagerMap.get(controllerToolbox.getFullRobotModel().getChest().getName());
      this.headManager = bodyManagerMap.get(controllerToolbox.getFullRobotModel().getHead().getName());
      this.footSwitches = controllerToolbox.getFootSwitches();

      this.groundContactForceAboveThresholdCount = new YoInteger(getClass().getSimpleName() + "GroundForceAboveThresholdCount", registry);
   }

   @Override
   public boolean isDone()
   {
      return checkIfFeetHaveCollidedWithGround();
   }

   @Override
   public void doAction()
   {
      wholeBodyMomentumManager.computeMomentumRateOfChangeForFreeFall();
      gravityCompensationManager.setRootJointAccelerationForFreeFall();
      feetManager.makeFeetFullyUnconstrained();

      for (RobotSide side : RobotSide.values)
         handManagers.get(side).compute();
      chestManager.compute();
      headManager.compute();
   }

   private boolean checkIfFeetHaveCollidedWithGround()
   {
      double totalGroundReactionZ = feetManager.getGroundReactionForceZ();
      updateGlitchBooleans();
      glitchCount[glitchFilterWindowSize - 1] = totalGroundReactionZ > contactThreshold;
      return getGlitchFilteredOutput();
   }

   private void updateGlitchBooleans()
   {
      for (int i = 0; i < glitchCount.length - 1; i++)
         glitchCount[i] = glitchCount[i + 1];
   }

   private void resetGlitchBooleans()
   {
      for (int i = 0; i < glitchCount.length; i++)
         glitchCount[i] = false;
   }

   private boolean getGlitchFilteredOutput()
   {
      int numberOfTrues = 0;
      for(int i = 0; i < glitchCount.length; i++)
         numberOfTrues += glitchCount[i] ? 1 : 0;
      groundContactForceAboveThresholdCount.set(numberOfTrues);
      if(numberOfTrues > 1)
         return true;
      else
         return false;
   }
   
   @Override
   public void doStateSpecificTransitionIntoAction()
   {
      resetGlitchBooleans();
      controllerToolbox.clearContacts();
      for (RobotSide side : RobotSide.values)
      {
         RigidBodyControlManager handManager = handManagers.get(side);
         handManager.holdInTaskspace();
      }
      headManager.holdInJointspace();
      chestManager.holdInJointspace();
   }

   @Override
   public void doTransitionOutOfAction()
   {
      // TODO Auto-generated method stub

   }

}
