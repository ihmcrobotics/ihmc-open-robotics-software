package us.ihmc.humanoidBehaviors.behaviors.rrtPlanner;

import java.util.ArrayList;

import us.ihmc.commons.PrintTools;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.rrtPlanner.ControlPointOptimizationStateMachineBehavior.ControlPointOptimizationStates;
import us.ihmc.humanoidBehaviors.behaviors.rrtPlanner.ValidNodesStateMachineBehavior.RRTExpandingStates;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.wholebodyValidityTester.SolarPanelPoseValidityTester;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.manipulation.planning.rrt.RRTNode;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;

public class ControlPointOptimizationStateMachineBehavior extends StateMachineBehavior<ControlPointOptimizationStates>
{
   private FullHumanoidRobotModel fullRobotModel;
   private WholeBodyControllerParameters wholeBodyControllerParameters;
         
   private ValidNodesStateMachineBehavior validNodesStateMachineBehavior;

   private CandidateBehavior candidateBehavior;
   
   private TestDoneBehavior doneBehavior;
   
   
   
   private RRTNode startNode;
   private RRTNode optimalControlPointNode;
   
   
   private int numberOfCandidates = 10;
   private int numberOfWayPoints = 10;
   
   private int currentIndexOfCandidate = 0;
   
   /*
    *  indexOfLinearPath=1; indexOfLinearPath<SolarPanelPath.getNumerOfWayPoints(); indexOfLinearPath++
    */
   private int indexOfLinearPath = 1;  
   
   
   
   public enum ControlPointOptimizationStates
   {
      CANDIDATE_CONTROLPOINTS, GET_SCORE, DONE
   }
   
   public ControlPointOptimizationStateMachineBehavior(RRTNode startNode, CommunicationBridge communicationBridge, DoubleYoVariable yoTime,  
                                         WholeBodyControllerParameters wholeBodyControllerParameters, FullHumanoidRobotModel fullRobotModel)
   {
      super("ControlPointOptimizationStateMachineBehavior", ControlPointOptimizationStates.class, yoTime, communicationBridge);
      
      PrintTools.info("ControlPointOptimizationStateMachineBehavior ");
      
      this.startNode = startNode;
      
      this.fullRobotModel = fullRobotModel;
      this.wholeBodyControllerParameters = wholeBodyControllerParameters;
            
      validNodesStateMachineBehavior = new ValidNodesStateMachineBehavior(new ArrayList<RRTNode>(), communicationBridge, yoTime, wholeBodyControllerParameters, fullRobotModel);
      candidateBehavior = new CandidateBehavior(communicationBridge);
      doneBehavior = new TestDoneBehavior(communicationBridge);
      
      
      setUpStateMachine();
   }
   
   public RRTNode getOptimalControlPointNode()
   {
      return optimalControlPointNode;
   }
   
   public ArrayList<RRTNode> getWayPointsNodes(RRTNode endNode)
   {
      ArrayList<RRTNode> retNodes = new ArrayList<RRTNode>();
            
      for(int i=0;i<numberOfWayPoints;i++)
      {
         RRTNode aNode = new TimeDomain1DNode();
         for(int j=0;j<aNode.getDimensionOfNodeData();j++)
         {
            double nodeData = startNode.getNodeData(j) + (endNode.getNodeData(j) - startNode.getNodeData(j))*(i)/(numberOfWayPoints-1);
            aNode.setNodeData(j, nodeData);
         }
         retNodes.add(aNode);
      }
      PrintTools.info(" "+numberOfWayPoints+" "+retNodes.size());
      return retNodes;
   }
   
   private void setUpStateMachine()
   {    
      BehaviorAction<ControlPointOptimizationStates> candidateAction = new BehaviorAction<ControlPointOptimizationStates>(ControlPointOptimizationStates.CANDIDATE_CONTROLPOINTS, candidateBehavior);
      
      BehaviorAction<ControlPointOptimizationStates> getScoreAction = new BehaviorAction<ControlPointOptimizationStates>(ControlPointOptimizationStates.GET_SCORE, validNodesStateMachineBehavior);
            
      BehaviorAction<ControlPointOptimizationStates> doneAction = new BehaviorAction<ControlPointOptimizationStates>(ControlPointOptimizationStates.DONE, doneBehavior);
      
      
      
      
      
      
      
      statemachine.addStateWithDoneTransition(candidateAction, ControlPointOptimizationStates.GET_SCORE);
      
      statemachine.addStateWithDoneTransition(getScoreAction, ControlPointOptimizationStates.DONE);
      
      statemachine.addState(doneAction);
      
      statemachine.setStartState(ControlPointOptimizationStates.CANDIDATE_CONTROLPOINTS);
   }

   @Override
   public void onBehaviorExited()
   {
      // TODO Auto-generated method stub
   }
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   private class CandidateBehavior extends AbstractBehavior
   {
      private boolean isDone = false;
      
      public CandidateBehavior(CommunicationBridgeInterface communicationBridge)
      {
         super(communicationBridge);
      }

      @Override
      public void doControl()
      {            
         isDone = true;
      }

      @Override
      public void onBehaviorEntered()
      {         
         // randomly selected endNode
         RRTNode endNode = new TimeDomain1DNode(4.0, Math.PI*0.1);
         validNodesStateMachineBehavior.setNodes(getWayPointsNodes(endNode));
         
//         ArrayList<RRTNode> nodes = new ArrayList<RRTNode>();
//         
//         nodes.add(new TimeDomain1DNode(0.0, 0.0));
//         nodes.add(new TimeDomain1DNode(0.3, Math.PI*0.1));
//         nodes.add(new TimeDomain1DNode(0.4, -Math.PI*0.1));
//         nodes.add(new TimeDomain1DNode(1.7, Math.PI*0.2));
//         nodes.add(new TimeDomain1DNode(0.3, Math.PI*0.2));
//         nodes.add(new TimeDomain1DNode(0.0, 0.0));
//         
//         validNodesStateMachineBehavior.setNodes(nodes);
         validNodesStateMachineBehavior.setSolarPanel(TimeDomain1DNode.cleaningPath.getSolarPanel());
      }

      @Override
      public void onBehaviorAborted()
      {
      }

      @Override
      public void onBehaviorPaused()
      {
      }

      @Override
      public void onBehaviorResumed()
      {
      }

      @Override
      public void onBehaviorExited()
      {         
      }

      @Override
      public boolean isDone()
      {
         return isDone;
      }
   }
   
   private class TestDoneBehavior extends AbstractBehavior
   {
      private boolean isDone = false;
      
      public TestDoneBehavior(CommunicationBridgeInterface communicationBridge)
      {
         super(communicationBridge);
      }

      @Override
      public void doControl()
      {            
         isDone = true;
      }

      @Override
      public void onBehaviorEntered()
      {   
      }

      @Override
      public void onBehaviorAborted()
      {
      }

      @Override
      public void onBehaviorPaused()
      {
      }

      @Override
      public void onBehaviorResumed()
      {
      }

      @Override
      public void onBehaviorExited()
      {         
      }

      @Override
      public boolean isDone()
      {
         return isDone;
      }
   }
}
