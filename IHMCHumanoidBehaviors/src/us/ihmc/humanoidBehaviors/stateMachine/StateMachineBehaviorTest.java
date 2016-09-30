package us.ihmc.humanoidBehaviors.stateMachine;

import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.ResetRobotBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ClearLidarBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.EnableLidarBehavior;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehaviorTest.SimpleState;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class StateMachineBehaviorTest extends StateMachineBehavior<SimpleState>
{

   public enum SimpleState
   {
      ENABLE_LIDAR, CLEAR_LIDAR, BEHAVIOR_COMPLETE
   }

   private EnableLidarBehavior enableLidarBehavior;
   private ClearLidarBehavior clearLidarBehavior;
   private ResetRobotBehavior resetRobotBehavior;
   private BehaviorCommunicationBridge outgoingCommunicationBridge;

   private DoubleYoVariable yoTime;

   //States
   private BehaviorStateWrapper<SimpleState> enableState;
   private BehaviorStateWrapper<SimpleState> clearState;
   private BehaviorStateWrapper<SimpleState> completeState;

   public StateMachineBehaviorTest(String name, String switchTimeName, DoubleYoVariable yoTime, YoVariableRegistry registry,
         BehaviorCommunicationBridge outgoingCommunicationBridge)
   {
      super(name, switchTimeName, SimpleState.class, yoTime, outgoingCommunicationBridge);
      this.outgoingCommunicationBridge = outgoingCommunicationBridge;
      this.yoTime = yoTime;
      init();
   }

   public void init()
   {
      enableLidarBehavior = new EnableLidarBehavior(outgoingCommunicationBridge);
      clearLidarBehavior = new ClearLidarBehavior(outgoingCommunicationBridge);
      resetRobotBehavior = new ResetRobotBehavior(outgoingCommunicationBridge, yoTime);
      
      enableState = createState(SimpleState.ENABLE_LIDAR,yoTime, enableLidarBehavior);
      clearState = createState(SimpleState.CLEAR_LIDAR,yoTime, clearLidarBehavior);
      completeState = createState(SimpleState.BEHAVIOR_COMPLETE,yoTime, resetRobotBehavior);
      
      enableState.createDefaultTransition(SimpleState.CLEAR_LIDAR);
      clearState.createDefaultTransition(SimpleState.BEHAVIOR_COMPLETE);

      getStateMachine().setCurrentState(SimpleState.ENABLE_LIDAR);
   }

  

   @Override
   public boolean isDone()
   {
      return completeState.isDone();
   }

   public void doTransitionIntoAction()
   {

   }

   public void doTransitionOutOfAction()
   {
      
   }
}
