package us.ihmc.avatar.networkProcessor.rrtToolboxModule;

import java.util.ArrayList;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.CTTaskNode;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
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
   
   private void updateHandTrajectoryMessages()
   {
      handTrajectoryMessages.put(RobotSide.LEFT, new HandTrajectoryMessage(RobotSide.LEFT, this.trajectoryTime, new Point3D(0.5, 0.35, 1.0), new Quaternion(), ReferenceFrame.getWorldFrame()));
      handTrajectoryMessages.put(RobotSide.RIGHT, new HandTrajectoryMessage(RobotSide.RIGHT, this.trajectoryTime, new Point3D(0.5, -0.35, 1.0), new Quaternion(), ReferenceFrame.getWorldFrame()));
   }
   
   private void updateChestTrajectoryMessage()   
   {
      
   }
   
   private void updatePelvisTrajectoryMessage()
   {
      
   }
   
   public void setCTTaskNodePath(ArrayList<CTTaskNode> path)
   {
      this.trajectoryTime = ConstrainedWholeBodyPlanningToolboxController.constrainedEndEffectorTrajectory.getTrajectoryTime();
      
      this.path = path;
   }

   public WholeBodyTrajectoryMessage getWholeBodyTrajectoryMessage()
   {
      updateHandTrajectoryMessages();
      updateChestTrajectoryMessage();
      updatePelvisTrajectoryMessage();
      wholeBodyTrajectoryMessage.clear();
      
      for(RobotSide robotSide : RobotSide.values)
         wholeBodyTrajectoryMessage.setHandTrajectoryMessage(handTrajectoryMessages.get(robotSide));
      wholeBodyTrajectoryMessage.setChestTrajectoryMessage(chestTrajectoryMessage);
      wholeBodyTrajectoryMessage.setPelvisTrajectoryMessage(pelvisTrajectoryMessage);
      
      return wholeBodyTrajectoryMessage;
   }
}
