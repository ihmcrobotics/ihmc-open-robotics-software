package us.ihmc.humanoidBehaviors.behaviors.rrtPlanner;

import java.util.ArrayList;

import us.ihmc.commons.PrintTools;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.rrtPlanner.ControlPointOptimizationStateMachineBehavior.ControlPointOptimizationStates;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.wholebodyValidityTester.WholeBodyPoseValidityTester;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.manipulation.planning.rrt.RRTNode;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;

public class ControlPointOptimizationStateMachineBehavior extends StateMachineBehavior<ControlPointOptimizationStates>
{
   private boolean DEBUG = false;
            
   private ValidNodesStateMachineBehavior validNodesStateMachineBehavior;

   private CandidateBehavior candidateBehavior;
   
   private TestDoneBehavior doneBehavior;
   
   private RRTNode rootNode;
      
   private double optimalScoreOfControlPoint;   
   
   private ArrayList<RRTNode> currentControlPointNodePath = new ArrayList<RRTNode>();
   private ArrayList<RRTNode> optimalControlPointNodePath;
   
   private int numberOfCandidates = 30;
   private int numberOfLinearPath;
   private double minimumTimeGapOfWayPoints = 0.3;
   
   private int currentIndexOfCandidate;
   
   
   public enum ControlPointOptimizationStates
   {
      GET_SCORE, CANDIDATE_CONTROLPOINTS, DONE
   }
   
   public ControlPointOptimizationStateMachineBehavior(RRTNode startNode, CommunicationBridge communicationBridge, DoubleYoVariable yoTime,  
                                         WholeBodyControllerParameters wholeBodyControllerParameters, FullHumanoidRobotModel fullRobotModel)
   {
      super("ControlPointOptimizationStateMachineBehavior", ControlPointOptimizationStates.class, yoTime, communicationBridge);
      
      PrintTools.info("ControlPointOptimizationStateMachineBehavior ");
      
      this.rootNode = startNode;
                  
      validNodesStateMachineBehavior = new ValidNodesStateMachineBehavior(new ArrayList<RRTNode>(), communicationBridge, yoTime, wholeBodyControllerParameters, fullRobotModel);
      candidateBehavior = new CandidateBehavior(communicationBridge);
      doneBehavior = new TestDoneBehavior(communicationBridge);
            
      numberOfLinearPath = TimeDomain1DNode.cleaningPath.getNumerOfLinearPath();
      
      currentIndexOfCandidate = 0;
      
      optimalScoreOfControlPoint = Double.MAX_VALUE;
      
      setUpStateMachine();
   }
   
   public ArrayList<RRTNode> getOptimalControlPointNodePath()
   {
      return optimalControlPointNodePath;
   }
   
   private ArrayList<RRTNode> getWayPointsNodes(RRTNode startNode, RRTNode endNode)
   {
      ArrayList<RRTNode> retNodes = new ArrayList<RRTNode>();
            
      double timeGap = endNode.getNodeData(0)-startNode.getNodeData(0);
      
      int numberOfWayPoints = (int)(timeGap / minimumTimeGapOfWayPoints) + 1;
      
      for(int i=0;i<numberOfWayPoints;i++)
      {
         RRTNode aNode = new TimeDomain1DNode();
         for(int j=0;j<aNode.getDimensionOfNodeData();j++)
         {
            double nodeData = startNode.getNodeData(j) + (endNode.getNodeData(j) - startNode.getNodeData(j))*(i+1)/(numberOfWayPoints);
            aNode.setNodeData(j, nodeData);
         }
         retNodes.add(aNode);
      }
      
//      for(int i=0;i<numberOfWayPoints;i++)
//         PrintTools.info("data "+i+" "+ retNodes.get(i).getNodeData(0)+" "+ retNodes.get(i).getNodeData(1));

      return retNodes;
   }
   
   private RRTNode getRandomControlPoint(int indexOfControlPoint)
   {
      TimeDomain1DNode aNode = new TimeDomain1DNode();
      
      double timeOfEndNode = TimeDomain1DNode.cleaningPath.getArrivalTime().get(indexOfControlPoint+1);
            
      aNode.setNodeData(0, timeOfEndNode);
      
      aNode.setRandomNodeData();
      
      return aNode;
   }
   
   private void setUpStateMachine()
   {    
      BehaviorAction<ControlPointOptimizationStates> getScoreAction = new BehaviorAction<ControlPointOptimizationStates>(ControlPointOptimizationStates.GET_SCORE, validNodesStateMachineBehavior)
      {         
         @Override
         protected void setBehaviorInput()
         {
            if(DEBUG)
               PrintTools.info("getScoreAction "+currentIndexOfCandidate + " ");
            
            currentControlPointNodePath = new ArrayList<RRTNode>();
            currentControlPointNodePath.add(rootNode);
            
            ArrayList<RRTNode> randomSelectedNodes = new ArrayList<RRTNode>();
            randomSelectedNodes.add(rootNode);
            
            for(int i=0;i<numberOfLinearPath;i++)
            {
               RRTNode randomControlPoint = getRandomControlPoint(i);
               
               randomSelectedNodes.addAll(getWayPointsNodes(currentControlPointNodePath.get(i), randomControlPoint));
               currentControlPointNodePath.add(randomControlPoint);
            }
                                          
            validNodesStateMachineBehavior.setNodes(randomSelectedNodes);               
            validNodesStateMachineBehavior.setSolarPanel(TimeDomain1DNode.cleaningPath.getSolarPanel());
            
            currentIndexOfCandidate++;     
            
            
            
         }
      };
      
      BehaviorAction<ControlPointOptimizationStates> candidateAction = new BehaviorAction<ControlPointOptimizationStates>(ControlPointOptimizationStates.CANDIDATE_CONTROLPOINTS, candidateBehavior);
      
      StateTransitionCondition keepDoingCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {            
            boolean b = candidateBehavior.isDone() && currentIndexOfCandidate != numberOfCandidates;
            return b;
         }
      };
      
      StateTransitionCondition noSolutionCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {            
            boolean b = (candidateBehavior.isDone() && currentIndexOfCandidate == numberOfCandidates) && optimalControlPointNodePath == null;
            if(b == true)
            {
               currentIndexOfCandidate = 0;
               numberOfCandidates = 5;   
            }
            return b;
         }
      };

      StateTransitionCondition doneCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            boolean b = candidateBehavior.isDone() && currentIndexOfCandidate == numberOfCandidates && optimalControlPointNodePath != null;
            PrintTools.info("Yes solution ");
            return b;
         }
      };
      
      BehaviorAction<ControlPointOptimizationStates> doneAction = new BehaviorAction<ControlPointOptimizationStates>(ControlPointOptimizationStates.DONE, doneBehavior);
      
      
      
      
      
      statemachine.addStateWithDoneTransition(getScoreAction, ControlPointOptimizationStates.CANDIDATE_CONTROLPOINTS);
      
      statemachine.addState(candidateAction);
      
      candidateAction.addStateTransition(ControlPointOptimizationStates.GET_SCORE, keepDoingCondition);
      candidateAction.addStateTransition(ControlPointOptimizationStates.GET_SCORE, noSolutionCondition);
      candidateAction.addStateTransition(ControlPointOptimizationStates.DONE, doneCondition);
      
      statemachine.addState(doneAction);
      
      statemachine.setStartState(ControlPointOptimizationStates.GET_SCORE);
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
         if(DEBUG)
            PrintTools.info("CandidateBehavior");
         if(DEBUG)
            PrintTools.info(""+currentIndexOfCandidate+" nodesValidity is " + validNodesStateMachineBehavior.getNodesValdity() +" score is "+ validNodesStateMachineBehavior.getScore());
         
         
         if(validNodesStateMachineBehavior.getNodesValdity() == true)
         {
            if(optimalScoreOfControlPoint > validNodesStateMachineBehavior.getScore())
            {
               if(DEBUG)
                  PrintTools.info("");
               if(DEBUG)
                  PrintTools.info("New Control Point updated ! ");
               if(DEBUG)
                  PrintTools.info("");
               optimalControlPointNodePath = currentControlPointNodePath;
               optimalScoreOfControlPoint = validNodesStateMachineBehavior.getScore();
            }
         }
         
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
         PrintTools.info("TestDoneBehavior");
         for(int i =0;i<optimalControlPointNodePath.size();i++)
            PrintTools.info("optimalControlPointNodePath "+i+" "+ optimalControlPointNodePath.get(i).getNodeData(0)+" "+ optimalControlPointNodePath.get(i).getNodeData(1));
         PrintTools.info("numberOfTest "+ WholeBodyPoseValidityTester.numberOfTest);
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
