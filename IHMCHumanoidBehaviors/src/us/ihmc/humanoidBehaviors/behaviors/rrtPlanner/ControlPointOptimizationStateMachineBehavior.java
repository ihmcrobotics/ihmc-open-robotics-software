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
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.manipulation.planning.rrt.generalrrt.RRTNode;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;

public class ControlPointOptimizationStateMachineBehavior extends StateMachineBehavior<ControlPointOptimizationStates>
{
   private boolean DEBUG = true;
   private int numberOfValidCandidates = 0;
            
   private ValidNodesStateMachineBehavior validNodesStateMachineBehavior;

   private CandidateBehavior candidateBehavior;
   
   private TestDoneBehavior doneBehavior;
   
   private RRTNode rootNode;
      
   private double optimalScoreOfControlPoint;   
   
   private ArrayList<RRTNode> currentControlPointNodePath = new ArrayList<RRTNode>();
   private ArrayList<RRTNode> optimalControlPointNodePath;
   
   private int numberOfCandidates = 2;
   private int numberOfLinearPath;
   private double minimumTimeGapOfWayPoints = 2.0;
   
   private int currentIndexOfCandidate;
   
   private int numberOfRetry = 1;
   private int numberOfCandidatesForRetry = 2;
   private int currentIndexOfRetry;
   
   private boolean isSolved = false;
   FullHumanoidRobotModel fullRobotModel;
   
   public enum ControlPointOptimizationStates
   {
      GET_SCORE, CANDIDATE_CONTROLPOINTS, DONE
   }
   
   public ControlPointOptimizationStateMachineBehavior(CommunicationBridge communicationBridge, DoubleYoVariable yoTime,  
                                         WholeBodyControllerParameters wholeBodyControllerParameters, FullHumanoidRobotModel fullRobotModel, HumanoidReferenceFrames referenceFrames)
   {
      super("ControlPointOptimizationStateMachineBehavior", ControlPointOptimizationStates.class, yoTime, communicationBridge);
      
      PrintTools.info("ControlPointOptimizationStateMachineBehavior ");
                        
      validNodesStateMachineBehavior = new ValidNodesStateMachineBehavior(communicationBridge, yoTime, wholeBodyControllerParameters, fullRobotModel, referenceFrames);
      candidateBehavior = new CandidateBehavior(communicationBridge);
      doneBehavior = new TestDoneBehavior(communicationBridge);
            
      currentIndexOfCandidate = 0;
      currentIndexOfRetry = 0;
      
      optimalScoreOfControlPoint = Double.MAX_VALUE;
      this.fullRobotModel = fullRobotModel;
      setUpStateMachine();
   }
   
   public void reInitialize()
   {
      currentIndexOfCandidate = 0;
      currentIndexOfRetry = 0;
      optimalScoreOfControlPoint = Double.MAX_VALUE;
      isSolved = false;
   }
   
   public void setRootNode(RRTNode rootNode)
   {
      this.rootNode = rootNode;
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
         RRTNode aNode = SolarPanelCleaningInfo.getNode();
         for(int j=0;j<aNode.getDimensionOfNodeData();j++)
         {
            double nodeData = startNode.getNodeData(j) + (endNode.getNodeData(j) - startNode.getNodeData(j))*(i+1)/(numberOfWayPoints);
            aNode.setNodeData(j, nodeData);
         }
         retNodes.add(aNode);
      }
      
      return retNodes;
   }
   
   private RRTNode getRandomControlPoint(int indexOfControlPoint)
   {
      RRTNode aNode = SolarPanelCleaningInfo.getNode();
      
      double timeOfEndNode = SolarPanelCleaningInfo.getCleaningPath().getArrivalTime().get(indexOfControlPoint+1);
            
      aNode.setNodeData(0, timeOfEndNode);
      
      aNode.setRandomNodeData();
      
      return aNode;
   }
   
   public boolean isSolved()
   {
      return isSolved;
   }
   
   public void setUpStateMachine()
   {    
      BehaviorAction<ControlPointOptimizationStates> getScoreAction = new BehaviorAction<ControlPointOptimizationStates>(ControlPointOptimizationStates.GET_SCORE, validNodesStateMachineBehavior)
      {         
         @Override
         protected void setBehaviorInput()
         {
            numberOfLinearPath = SolarPanelCleaningInfo.getCleaningPath().getNumerOfLinearPath();
            PrintTools.info("## number of linear path "+ numberOfLinearPath);
            
            if(DEBUG)
            {
               PrintTools.info("");
               PrintTools.info("getScoreAction "+currentIndexOfCandidate + " ");               
            }
            
            TimeDomain3DNode.defaultPelvisHeight = fullRobotModel.getPelvis().getParentJoint().getFrameAfterJoint().getTransformToWorldFrame().getM23();
            
            rootNode = SolarPanelCleaningInfo.getNode();
            
            currentControlPointNodePath = new ArrayList<RRTNode>();
            currentControlPointNodePath.add(getRandomControlPoint(-1));
            
            ArrayList<RRTNode> randomSelectedNodes = new ArrayList<RRTNode>();
            randomSelectedNodes.add(currentControlPointNodePath.get(currentControlPointNodePath.size()-1));
            
            for(int i=0;i<numberOfLinearPath;i++)
            {
               RRTNode randomControlPoint = getRandomControlPoint(i);
               
               randomSelectedNodes.addAll(getWayPointsNodes(currentControlPointNodePath.get(i), randomControlPoint));
               currentControlPointNodePath.add(randomControlPoint);
            }
            
            for(int i=0;i<currentControlPointNodePath.size();i++)
            {
//               PrintTools.info(""+i+" ");
//               for(int j=0;j<currentControlPointNodePath.get(i).getDimensionOfNodeData();j++)
//                  PrintTools.info(""+currentControlPointNodePath.get(i).getNodeData(j));
            }
                                          
            validNodesStateMachineBehavior.setNodes(randomSelectedNodes);               
            validNodesStateMachineBehavior.setSolarPanel(SolarPanelCleaningInfo.getCleaningPath().getSolarPanel());
            
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
            boolean b = ((candidateBehavior.isDone() && currentIndexOfCandidate == numberOfCandidates) && optimalControlPointNodePath == null) && (currentIndexOfRetry < numberOfRetry);
            if(b == true)
            {
               PrintTools.info("No solution .. Retry! " + currentIndexOfRetry);
               currentIndexOfRetry++;
               currentIndexOfCandidate = 0;
               numberOfCandidates = numberOfCandidatesForRetry;   
            }
            return b;
         }
      };

      StateTransitionCondition doneCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            boolean b = (candidateBehavior.isDone() && currentIndexOfCandidate == numberOfCandidates && optimalControlPointNodePath != null) || (currentIndexOfRetry == numberOfRetry);
            
            if(currentIndexOfRetry == numberOfRetry)
            {               
               isSolved = false;
               PrintTools.info("Finally... No solution "+isSolved);
            }
            else
            {               
               isSolved = true;
               PrintTools.info("Planner get solution!! "+numberOfValidCandidates);
               PrintTools.info("isSolved() "+isSolved());
               
            for(int i=0;i<optimalControlPointNodePath.size();i++)
            {
               PrintTools.info("optimal Path is "+i);
               for(int j=0;j<optimalControlPointNodePath.get(0).getDimensionOfNodeData();j++)
                  PrintTools.info(" "+ optimalControlPointNodePath.get(i).getNodeData(j));
               
               PrintTools.info("");
            }
               
            }
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
      PrintTools.info("setUpStateMachine done ");
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
            PrintTools.info(""+(currentIndexOfCandidate-1)+" nodesValidity is " + validNodesStateMachineBehavior.getNodesValdity() +" score is "+ validNodesStateMachineBehavior.getScore());
                  
         if(validNodesStateMachineBehavior.getNodesValdity() == true)
         {
            numberOfValidCandidates++;
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
