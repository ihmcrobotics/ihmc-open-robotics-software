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
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;

public class ControlPointOptimizationStateMachineBehavior extends StateMachineBehavior<ControlPointOptimizationStates>
{
   private boolean DEBUG = true;
   
   private FullHumanoidRobotModel fullRobotModel;
   private WholeBodyControllerParameters wholeBodyControllerParameters;
         
   private ValidNodesStateMachineBehavior validNodesStateMachineBehavior;

   private CandidateBehavior candidateBehavior;
   
   private TestDoneBehavior doneBehavior;
   
   
   
   private RRTNode rootNode;
   private RRTNode currentControlPoint;
   private RRTNode optimalControlPointCurrent;
   private RRTNode optimalControlPointNext;
   private double optimalScoreOfControlPoint;
   private ArrayList<RRTNode> optimalControlPointNodePath = new ArrayList<RRTNode>();
   
   private int numberOfCandidates = 6;
   private int numberOfLinearPath;
   private int numberOfWayPoints = 8;
   
   private int currentIndexOfCandidate;
   private int currentIndexOfLinearPath; 
   
   /*
    *  indexOfLinearPath=1; indexOfLinearPath<SolarPanelPath.getNumerOfWayPoints(); indexOfLinearPath++
    */
   
   
   
   
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
      
      this.fullRobotModel = fullRobotModel;
      this.wholeBodyControllerParameters = wholeBodyControllerParameters;
            
      validNodesStateMachineBehavior = new ValidNodesStateMachineBehavior(new ArrayList<RRTNode>(), communicationBridge, yoTime, wholeBodyControllerParameters, fullRobotModel);
      candidateBehavior = new CandidateBehavior(communicationBridge);
      doneBehavior = new TestDoneBehavior(communicationBridge);
      
      
      numberOfLinearPath = TimeDomain1DNode.cleaningPath.getNumerOfLinearPath();
      
      currentIndexOfCandidate = 0;
      currentIndexOfLinearPath = 0;
      
      optimalScoreOfControlPoint = Double.MAX_VALUE;
      
      
      
      optimalControlPointCurrent = rootNode;
      optimalControlPointNodePath.add(optimalControlPointCurrent);
       
      
      setUpStateMachine();
   }
   
   public ArrayList<RRTNode> getOptimalControlPointNodePath()
   {
      return optimalControlPointNodePath;
   }
   
   private ArrayList<RRTNode> getWayPointsNodes(RRTNode startNode, RRTNode endNode)
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
      
//      for(int i=0;i<numberOfWayPoints;i++)
//         PrintTools.info("data "+i+" "+ retNodes.get(i).getNodeData(0)+" "+ retNodes.get(i).getNodeData(1));

      return retNodes;
   }
   
   private RRTNode getRandomControlPoint()
   {
      TimeDomain1DNode aNode = new TimeDomain1DNode();
      
      double timeOfEndNode = TimeDomain1DNode.cleaningPath.getArrivalTime().get(currentIndexOfLinearPath+1);
            
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
            PrintTools.info("getScoreAction "+currentIndexOfCandidate + " "+currentIndexOfLinearPath);
            RRTNode randomControlPoint = getRandomControlPoint();
            currentControlPoint = randomControlPoint;
            
            // rootNode should be substituted with optimal control point among previous candidates.                     
            validNodesStateMachineBehavior.setNodes(getWayPointsNodes(optimalControlPointCurrent, randomControlPoint));   
            
            validNodesStateMachineBehavior.setSolarPanel(TimeDomain1DNode.cleaningPath.getSolarPanel());
            
            
            currentIndexOfCandidate++;            
            if(currentIndexOfCandidate == numberOfCandidates)
            {               
               currentIndexOfCandidate = 0;
               currentIndexOfLinearPath++;     
               optimalControlPointCurrent = optimalControlPointNext;
               optimalControlPointNodePath.add(optimalControlPointCurrent);
               
               optimalScoreOfControlPoint = Double.MAX_VALUE;
               
               PrintTools.info("optimalControlPointCurrent "+optimalControlPointCurrent.getNodeData(0)+" "+ optimalControlPointCurrent.getNodeData(1));
               PrintTools.info("");
            }
         }
      };
      
      BehaviorAction<ControlPointOptimizationStates> candidateAction = new BehaviorAction<ControlPointOptimizationStates>(ControlPointOptimizationStates.CANDIDATE_CONTROLPOINTS, candidateBehavior);
      
      StateTransitionCondition keepDoingCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {            
            boolean b = candidateBehavior.isDone() && currentIndexOfLinearPath != numberOfLinearPath;
            return b;
         }
      };

      StateTransitionCondition doneCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            boolean b = candidateBehavior.isDone() && currentIndexOfLinearPath == numberOfLinearPath;
            return b;
         }
      };
      
      BehaviorAction<ControlPointOptimizationStates> doneAction = new BehaviorAction<ControlPointOptimizationStates>(ControlPointOptimizationStates.DONE, doneBehavior);
      
      
      
      
      
      statemachine.addStateWithDoneTransition(getScoreAction, ControlPointOptimizationStates.CANDIDATE_CONTROLPOINTS);
      
      statemachine.addState(candidateAction);
      
      candidateAction.addStateTransition(ControlPointOptimizationStates.GET_SCORE, keepDoingCondition);
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
         PrintTools.info("CandidateBehavior");
         PrintTools.info(""+currentIndexOfCandidate+" nodesValidity is " + validNodesStateMachineBehavior.getNodesValdity() +" score is "+ validNodesStateMachineBehavior.getScore());
         if(validNodesStateMachineBehavior.getNodesValdity() == true)
         {
            if(optimalScoreOfControlPoint > validNodesStateMachineBehavior.getScore())
            {
               PrintTools.info("New Control Point updated ! ");
               optimalScoreOfControlPoint = validNodesStateMachineBehavior.getScore();
               optimalControlPointNext = currentControlPoint;
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
