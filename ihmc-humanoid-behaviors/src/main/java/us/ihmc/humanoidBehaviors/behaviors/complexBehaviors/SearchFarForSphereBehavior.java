package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.SearchFarForSphereBehavior.SearchFarState;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SphereDetectionBehavior;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataFilterParameters;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.yoVariables.variable.YoDouble;

public class SearchFarForSphereBehavior extends StateMachineBehavior<SearchFarState>
{
   public enum SearchFarState
   {
      ENABLE_LIDAR, SETUP_LIDAR, CLEAR_LIDAR, SEARCHING_FOR_SPHERE, VALIDATING
   }

   private final SphereDetectionBehavior initialSphereDetectionBehavior;
   private final AtlasPrimitiveActions atlasPrimitiveActions;

   public SearchFarForSphereBehavior(String robotName, YoDouble yoTime,  
                                     HumanoidReferenceFrames referenceFrames, Ros2Node ros2Node,
                                      AtlasPrimitiveActions atlasPrimitiveActions)
   {
      super(robotName, "SearchForSpehereFar", SearchFarState.class, yoTime, ros2Node);
      this.atlasPrimitiveActions = atlasPrimitiveActions;

      initialSphereDetectionBehavior = new SphereDetectionBehavior(robotName, ros2Node, referenceFrames);

      
      setupStateMachine();
   }

   @Override
   protected SearchFarState configureStateMachineAndReturnInitialKey(StateMachineFactory<SearchFarState, BehaviorAction> factory)
   {
      //ENABLE LIDAR
      BehaviorAction enableLidarTask = new BehaviorAction(atlasPrimitiveActions.enableLidarBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            // FIXME atlasPrimitiveActions.enableLidarBehavior.setLidarState(LidarState.ENABLE_BEHAVIOR_ONLY);
         }
      };

      //REDUCE LIDAR RANGE *******************************************

      BehaviorAction setLidarMediumRangeTask = new BehaviorAction(atlasPrimitiveActions.setLidarParametersBehavior)
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
      BehaviorAction clearLidarTask = new BehaviorAction(atlasPrimitiveActions.clearLidarBehavior);

      //SEARCH FOR BALL *******************************************

      BehaviorAction findBallTask = new BehaviorAction(initialSphereDetectionBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            publishTextToSpeech("LOOKING FOR BALL");
           /* coactiveElement.searchingForBall.set(true);
            coactiveElement.foundBall.set(false);
            coactiveElement.ballX.set(0);
            coactiveElement.ballY.set(0);
            coactiveElement.ballZ.set(0);*/
         }
      };

      //

      // Confirm from the user that this is the correct ball *******************************************

     

      factory.addStateAndDoneTransition(SearchFarState.ENABLE_LIDAR, enableLidarTask, SearchFarState.SETUP_LIDAR);
      factory.addStateAndDoneTransition(SearchFarState.SETUP_LIDAR, setLidarMediumRangeTask, SearchFarState.CLEAR_LIDAR);
      factory.addStateAndDoneTransition(SearchFarState.CLEAR_LIDAR, clearLidarTask, SearchFarState.SEARCHING_FOR_SPHERE);

      
         factory.addState(SearchFarState.SEARCHING_FOR_SPHERE, findBallTask);
      

      return SearchFarState.ENABLE_LIDAR;
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
