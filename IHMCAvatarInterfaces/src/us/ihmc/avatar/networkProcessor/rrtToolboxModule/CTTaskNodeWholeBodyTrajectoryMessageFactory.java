package us.ihmc.avatar.networkProcessor.rrtToolboxModule;

import java.util.ArrayList;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConfigurationSpace;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConstrainedEndEffectorTrajectory;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.manipulation.planning.forwaypoint.SO3TrajectoryPointCalculatorTwo;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.CTTreeTools;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.CTTaskNode;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.waypoints.EuclideanTrajectoryPointCalculator;
import us.ihmc.robotics.math.trajectories.waypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class CTTaskNodeWholeBodyTrajectoryMessageFactory
{  
   private ArrayList<CTTaskNode> path;
   
   private ConstrainedEndEffectorTrajectory constrainedEndEffectorTrajectory;
   
   private double firstTrajectoryPointTime = 3.0;
   private double trajectoryTime;

   private WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage = new WholeBodyTrajectoryMessage();
   
   private SideDependentList<HandTrajectoryMessage> handTrajectoryMessages = new SideDependentList<>();
   private ChestTrajectoryMessage chestTrajectoryMessage;
   private PelvisTrajectoryMessage pelvisTrajectoryMessage;
   
   private ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public CTTaskNodeWholeBodyTrajectoryMessageFactory()
   {
//      for(RobotSide robotSide : RobotSide.values)
//         handTrajectoryMessages.put(robotSide, new HandTrajectoryMessage());
//      chestTrajectoryMessage = new ChestTrajectoryMessage();
//      pelvisTrajectoryMessage = new PelvisTrajectoryMessage();
   }
   
   private void updateHandTrajectoryMessages()
   {  
      for(RobotSide robotSide : RobotSide.values)
      {
         int numberOfTrajectoryPoints = path.size();
         
         HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage(robotSide, numberOfTrajectoryPoints);
         
         handTrajectoryMessage.getFrameInformation().setTrajectoryReferenceFrame(worldFrame);
         handTrajectoryMessage.getFrameInformation().setDataReferenceFrame(worldFrame);
         
         EuclideanTrajectoryPointCalculator euclideanTrajectoryPointCalculator = new EuclideanTrajectoryPointCalculator();
         
         for (int i = 0; i < numberOfTrajectoryPoints; i++)
         {
            CTTaskNode trajectoryNode = path.get(i);
            
            ConfigurationSpace configurationSpace = CTTreeTools.getConfigurationSpace(trajectoryNode, robotSide);
            
            Pose3D desiredPose = constrainedEndEffectorTrajectory.getEndEffectorPose(trajectoryNode.getNodeData(0), robotSide, configurationSpace);
            
            
            
            
            euclideanTrajectoryPointCalculator.appendTrajectoryPoint(new Point3D(desiredPose.getPosition()));
         }

         double[] trajectoryTimes = new double[numberOfTrajectoryPoints];
         
         for(int i=0;i<numberOfTrajectoryPoints;i++)
         {
            trajectoryTimes[i] = path.get(i).getTime();
         }
         
         euclideanTrajectoryPointCalculator.computeTrajectoryPointTimes(firstTrajectoryPointTime, trajectoryTimes);
         euclideanTrajectoryPointCalculator.computeTrajectoryPointVelocities(true);

         RecyclingArrayList<FrameEuclideanTrajectoryPoint> trajectoryPoints = euclideanTrajectoryPointCalculator.getTrajectoryPoints();
         
         
         for (int i = 0; i < numberOfTrajectoryPoints; i++)
         {         
            CTTaskNode trajectoryNode = path.get(i);
            
            ConfigurationSpace configurationSpace = CTTreeTools.getConfigurationSpace(trajectoryNode, robotSide);
            
            Pose3D desiredPose = constrainedEndEffectorTrajectory.getEndEffectorPose(trajectoryNode.getTime(), robotSide, configurationSpace);
            
            
            
            
            
            
            
            Point3D desiredPosition = new Point3D(desiredPose.getPosition());
            Vector3D desiredLinearVelocity = new Vector3D();
            Quaternion desiredOrientation = new Quaternion( desiredPose.getOrientation()  );
            
            if(robotSide == RobotSide.LEFT)
               desiredOrientation.appendRollRotation(Math.PI * 0.5);
            else
               desiredOrientation.appendRollRotation(-Math.PI * 0.5);
            
            
            
            
            Vector3D desiredAngularVelocity = new Vector3D();

            double time = trajectoryPoints.get(i).get(desiredPosition, desiredLinearVelocity);
            
            PrintTools.info(""+i+" "+time +" " + desiredLinearVelocity+" ");

            handTrajectoryMessage.setTrajectoryPoint(i, time, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity, worldFrame);
         }
         
         
         
         handTrajectoryMessages.put(robotSide, handTrajectoryMessage);
      
      }
   }
   
   private void updateChestTrajectoryMessage()   
   {
//      int numberOfTrajectoryPoints = path.size();
//      
//      chestTrajectoryMessage = new ChestTrajectoryMessage(numberOfTrajectoryPoints);
//      chestTrajectoryMessage.getFrameInformation().setTrajectoryReferenceFrame(worldFrame);
//      chestTrajectoryMessage.getFrameInformation().setDataReferenceFrame(worldFrame);
//      
//      SO3TrajectoryPointCalculatorTwo orientationCalculator = new SO3TrajectoryPointCalculatorTwo();
//      orientationCalculator.clear();
//      orientationCalculator.setFirstTrajectoryPointTime(firstTrajectoryPointTime);
//      
//      for(int i =0;i<numberOfTrajectoryPoints;i++)
//      {
//         CTTaskNode trajectoryNode = path.get(i);
//         
//         double time = firstTrajectoryPointTime + trajectoryNode.getTime();
//         
//         Quaternion desiredOrientation = new Quaternion();
//         
//         desiredOrientation.appendYawRotation(trajectoryNode.getNodeData(2));
//         desiredOrientation.appendPitchRotation(trajectoryNode.getNodeData(3));
//         desiredOrientation.appendRollRotation(trajectoryNode.getNodeData(4));
//         
//         orientationCalculator.appendTrajectoryPointOrientation(time, desiredOrientation);
//      }
//      
//      orientationCalculator.compute();
//      
//      for(int i =0;i<numberOfTrajectoryPoints;i++)
//      {
//         CTTaskNode trajectoryNode = path.get(i);
//         
//         double time = firstTrajectoryPointTime + trajectoryNode.getTime();
//         
//         Quaternion desiredOrientation = new Quaternion();
//         
//         desiredOrientation.appendYawRotation(trajectoryNode.getNodeData(2));
//         desiredOrientation.appendPitchRotation(trajectoryNode.getNodeData(3));
//         desiredOrientation.appendRollRotation(trajectoryNode.getNodeData(4));
//         
//         Vector3D desiredAngularVelocity = orientationCalculator.getTrajectoryPointsAngularVelocity().get(i);
//         
//         PrintTools.info(""+i+" "+ time +" "+desiredOrientation +" " + desiredAngularVelocity);
//         
//         chestTrajectoryMessage.setTrajectoryPoint(i, time, desiredOrientation, desiredAngularVelocity, worldFrame);
//      }
   }
   
   private void updatePelvisTrajectoryMessage()
   {
      
   }
   
   public void setCTTaskNodePath(ArrayList<CTTaskNode> path, ConstrainedEndEffectorTrajectory constrainedEndEffectorTrajectory)
   {
      this.constrainedEndEffectorTrajectory = constrainedEndEffectorTrajectory;
      this.trajectoryTime = constrainedEndEffectorTrajectory.getTrajectoryTime();
      
      this.path = path;
   }

   public WholeBodyTrajectoryMessage getWholeBodyTrajectoryMessage()
   {  
      wholeBodyTrajectoryMessage.clear();
      
      updateHandTrajectoryMessages();
      updateChestTrajectoryMessage();
      updatePelvisTrajectoryMessage();
      
      
      for(RobotSide robotSide : RobotSide.values)
         wholeBodyTrajectoryMessage.setHandTrajectoryMessage(handTrajectoryMessages.get(robotSide));
      
      
//      wholeBodyTrajectoryMessage.setChestTrajectoryMessage(chestTrajectoryMessage);
//      wholeBodyTrajectoryMessage.setPelvisTrajectoryMessage(pelvisTrajectoryMessage);
      

      
      return wholeBodyTrajectoryMessage;
   }
}
