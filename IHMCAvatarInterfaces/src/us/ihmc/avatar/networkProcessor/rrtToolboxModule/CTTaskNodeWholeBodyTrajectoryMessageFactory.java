package us.ihmc.avatar.networkProcessor.rrtToolboxModule;

import java.util.ArrayList;

import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.CTTaskNode;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class CTTaskNodeWholeBodyTrajectoryMessageFactory
{  
   private ArrayList<CTTaskNode> path;
   
   private double firstTrajectoryPointTime = 1.0;
   private double trajectoryTime;

   private WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage = new WholeBodyTrajectoryMessage();
   
   private SideDependentList<HandTrajectoryMessage> handTrajectoryMessages = new SideDependentList<>();
   private ChestTrajectoryMessage chestTrajectoryMessage;
   private PelvisTrajectoryMessage pelvisTrajectoryMessage;

   public CTTaskNodeWholeBodyTrajectoryMessageFactory()
   {
      
   }
   
   private void generateHandTrajectoryMessages()
   {
      for(RobotSide robotSide : RobotSide.values)
      {
         
      }
   }
   
   private void generateChestTrajectoryMessage()
   {
      
   }
   
   private void generatePelvisTrajectoryMessage()
   {
      
   }
   
   public void setCTTaskNodePath(ArrayList<CTTaskNode> path)
   {
      trajectoryTime = ConstrainedWholeBodyPlanningToolboxController.constrainedEndEffectorTrajectory.getTrajectoryTime();
      
      this.path = path;
   }

   public WholeBodyTrajectoryMessage getWholeBodyTrajectoryMessage()
   {
      generateHandTrajectoryMessages();
      generateChestTrajectoryMessage();
      generatePelvisTrajectoryMessage();
      wholeBodyTrajectoryMessage.clear();
      
      for(RobotSide robotSide : RobotSide.values)
         wholeBodyTrajectoryMessage.setHandTrajectoryMessage(handTrajectoryMessages.get(robotSide));
      wholeBodyTrajectoryMessage.setChestTrajectoryMessage(chestTrajectoryMessage);
      wholeBodyTrajectoryMessage.setPelvisTrajectoryMessage(pelvisTrajectoryMessage);
      
      return wholeBodyTrajectoryMessage;
   }
}
