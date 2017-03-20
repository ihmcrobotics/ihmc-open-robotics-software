package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidBehaviors.behaviors.coactiveElements.PickUpBallBehaviorCoactiveElement.PickUpBallBehaviorState;
import us.ihmc.humanoidBehaviors.behaviors.coactiveElements.PickUpBallBehaviorCoactiveElementBehaviorSide;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.SearchFarForSphereBehavior.SearchFarState;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SphereDetectionBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.WaitForUserValidationBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataFilterParameters;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataStateCommand.LidarState;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class SearchFarForSphereBehavior extends StateMachineBehavior<SearchFarState>
{
   public enum SearchFarState
   {
      ENABLE_LIDAR, SETUP_LIDAR, CLEAR_LIDAR, SEARCHING_FOR_SPHERE, VALIDATING
   }

   private final SphereDetectionBehavior initialSphereDetectionBehavior;
   private final WaitForUserValidationBehavior waitForUserValidationBehavior;
   private final PickUpBallBehaviorCoactiveElementBehaviorSide coactiveElement;
   private final boolean requireUserValidation;
   private final AtlasPrimitiveActions atlasPrimitiveActions;

   public SearchFarForSphereBehavior(DoubleYoVariable yoTime, PickUpBallBehaviorCoactiveElementBehaviorSide coactiveElement,
         HumanoidReferenceFrames referenceFrames, CommunicationBridge outgoingCommunicationBridge, boolean requireUserValidation,
         AtlasPrimitiveActions atlasPrimitiveActions)
   {
      super("SearchForSpehereFar", SearchFarState.class, yoTime, outgoingCommunicationBridge);
      this.atlasPrimitiveActions = atlasPrimitiveActions;
      this.coactiveElement = coactiveElement;
      this.requireUserValidation = requireUserValidation;

      
      initialSphereDetectionBehavior = new SphereDetectionBehavior(outgoingCommunicationBridge, referenceFrames);


      waitForUserValidationBehavior = new WaitForUserValidationBehavior(outgoingCommunicationBridge, coactiveElement.validClicked,
            coactiveElement.validAcknowledged);
      setupStateMachine();
   }

   private void setupStateMachine()
   {

      //ENABLE LIDAR
      BehaviorAction<SearchFarState> enableLidarTask = new BehaviorAction<SearchFarState>(SearchFarState.ENABLE_LIDAR,
            atlasPrimitiveActions.enableLidarBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            atlasPrimitiveActions.enableLidarBehavior.setLidarState(LidarState.ENABLE_BEHAVIOR_ONLY);
         }
      };

      //REDUCE LIDAR RANGE *******************************************

      BehaviorAction<SearchFarState> setLidarMediumRangeTask = new BehaviorAction<SearchFarState>(SearchFarState.SETUP_LIDAR,
            atlasPrimitiveActions.setLidarParametersBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {

            DepthDataFilterParameters param = new DepthDataFilterParameters();
            param.nearScanRadius = 1.4f;
            atlasPrimitiveActions.setLidarParametersBehavior.setInput(param);
         }
      };

      //CLEAR LIDAR POINTS FOR CLEAN SCAN *******************************************
      BehaviorAction<SearchFarState> clearLidarTask = new BehaviorAction<SearchFarState>(SearchFarState.CLEAR_LIDAR, atlasPrimitiveActions.clearLidarBehavior);

      //SEARCH FOR BALL *******************************************

      BehaviorAction<SearchFarState> findBallTask = new BehaviorAction<SearchFarState>(SearchFarState.SEARCHING_FOR_SPHERE, initialSphereDetectionBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = new TextToSpeechPacket("LOOKING FOR BALL");
            sendPacket(p1);
            coactiveElement.searchingForBall.set(true);
            coactiveElement.foundBall.set(false);
            coactiveElement.ballX.set(0);
            coactiveElement.ballY.set(0);
            coactiveElement.ballZ.set(0);

         }
      };

      //

      // Confirm from the user that this is the correct ball *******************************************

      BehaviorAction<SearchFarState> validateBallTask = new BehaviorAction<SearchFarState>(SearchFarState.VALIDATING, waitForUserValidationBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            coactiveElement.searchingForBall.set(false);
            coactiveElement.waitingForValidation.set(true);
            coactiveElement.validAcknowledged.set(false);
            coactiveElement.foundBall.set(false);
            coactiveElement.currentState.set(PickUpBallBehaviorState.WAITING_FOR_USER_CONFIRMATION);
            coactiveElement.ballX.set(initialSphereDetectionBehavior.getBallLocation().getX());
            coactiveElement.ballY.set(initialSphereDetectionBehavior.getBallLocation().getY());
            coactiveElement.ballZ.set(initialSphereDetectionBehavior.getBallLocation().getZ());
            coactiveElement.ballRadius.set(initialSphereDetectionBehavior.getSpehereRadius());

         }
      };

      statemachine.addStateWithDoneTransition(enableLidarTask, SearchFarState.SETUP_LIDAR);
      statemachine.addStateWithDoneTransition(setLidarMediumRangeTask, SearchFarState.CLEAR_LIDAR);
      statemachine.addStateWithDoneTransition(clearLidarTask, SearchFarState.SEARCHING_FOR_SPHERE);

      if (requireUserValidation)
      {
         statemachine.addStateWithDoneTransition(findBallTask, SearchFarState.VALIDATING);
         statemachine.addState(validateBallTask);
      }

      else
         statemachine.addState(findBallTask);
      statemachine.setStartState(SearchFarState.ENABLE_LIDAR);

   }

   public boolean foundBall()
   {
      return initialSphereDetectionBehavior.foundBall();
   }

  
   public Point3D getBallLocation()
   {
      return initialSphereDetectionBehavior.getBallLocation();
   }

   @Override
   public void onBehaviorExited()
   {
   }

}
