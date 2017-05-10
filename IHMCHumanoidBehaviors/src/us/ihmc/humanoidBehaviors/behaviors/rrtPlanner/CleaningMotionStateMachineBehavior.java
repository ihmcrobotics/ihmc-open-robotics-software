package us.ihmc.humanoidBehaviors.behaviors.rrtPlanner;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.rrtPlanner.CleaningMotionStateMachineBehavior.CleaningMotionState;
import us.ihmc.humanoidBehaviors.behaviors.wholebodyValidityTester.WholeBodyPoseValidityTester;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanel;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanelCleaningPose;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanelPath;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.transformables.Pose;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;

public class CleaningMotionStateMachineBehavior extends StateMachineBehavior<CleaningMotionState>
{
   private SolarPanel solarPanel;
   
   private SolarPanelCleaningPose readyPose;
   private SolarPanelPath cleaningPath;
   
   private GetSolarPanelBehavior getSolarPanelBehavior;
   private ControlPointOptimizationStateMachineBehavior controlPointOptimizationBehavior;
   
   private TestDoneBehavior doneBehavior;
   
   public enum CleaningMotionState
   {
      GET_SOLARPANEL, CONTROLPOINT_OPTIMIZATION, GOTO_READYPOSE, CLEANING_MOTION, DONE
   }
   
   public CleaningMotionStateMachineBehavior(CommunicationBridge communicationBridge, DoubleYoVariable yoTime,  
                                         WholeBodyControllerParameters wholeBodyControllerParameters, FullHumanoidRobotModel fullRobotModel)
   {
      super("CleaningMotionStateMachineBehavior", CleaningMotionState.class, yoTime, communicationBridge);

      
      PrintTools.info("CleaningMotionStateMachineBehavior ");
      
      
      setUpStateMachine();
   }
   
   private void setUpStateMachine()
   {    
      // condition check for the case that no solution is exist in the CONTROLPOINT_OPTIMIZATION state.
      // In that case, the state transition should be to the DONE state.
      
      // In CONTROLPOINT_OPTIMIZATION, manually selected cleaning motion is put for test.
   }

   @Override
   public void onBehaviorExited()
   {
      // TODO Auto-generated method stub
   }
   
   
   
   
   
   private void setUpSolarPanel()
   {
      Pose poseSolarPanel = new Pose();
      Quaternion quaternionSolarPanel = new Quaternion();
      poseSolarPanel.setPosition(0.7, -0.05, 1.0);
      quaternionSolarPanel.appendRollRotation(0.0);
      quaternionSolarPanel.appendPitchRotation(-Math.PI*0.25);
      poseSolarPanel.setOrientation(quaternionSolarPanel);
      
      double sizeWidth = 0.6;
      double sizeLength = 0.6;
      
      solarPanel = new SolarPanel(poseSolarPanel, sizeWidth, sizeLength);
   }
   
   private class GetSolarPanelBehavior extends AbstractBehavior
   {
      private boolean isDone = false;
      
      public GetSolarPanelBehavior(CommunicationBridgeInterface communicationBridge)
      {
         super(communicationBridge);
      }

      @Override
      public void doControl()
      {            
         setUpSolarPanel();
         isDone = true;
      }

      @Override
      public void onBehaviorEntered()
      {   
         PrintTools.info("GetSolarPanelBehavior");
         
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
         // set cleaning path
         
         readyPose = new SolarPanelCleaningPose(solarPanel, 0.5, 0.1, -0.15, -Math.PI*0.2);
         cleaningPath = new SolarPanelPath(readyPose);
         
         cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.1, 0.1, -0.15, -Math.PI*0.3), 4.0);         
         cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.1, 0.2, -0.15, -Math.PI*0.3), 1.0);
         cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.5, 0.2, -0.15, -Math.PI*0.2), 4.0);
         cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.5, 0.3, -0.15, -Math.PI*0.2), 1.0);
         //cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.1, 0.3, -0.15, -Math.PI*0.3), 4.0);
         
         TimeDomain1DNode.cleaningPath = cleaningPath;
         TimeDomain1DNode rootNode = new TimeDomain1DNode(cleaningPath.getArrivalTime().get(0), 0);
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
