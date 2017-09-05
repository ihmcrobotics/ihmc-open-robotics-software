package us.ihmc.avatar.networkProcessor.rrtToolboxModule;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepListVisualizer;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.humanoidRobotics.communication.packets.SE3TrajectoryPointMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConfigurationSpace;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConstrainedEndEffectorTrajectory;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.CTTreeTools;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.CTTaskNode;
import us.ihmc.robotics.geometry.FramePoint3D;
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

   public CTTaskNodeWholeBodyTrajectoryMessageFactory()
   {
      
   }
   
   private void updateHandTrajectoryMessages()
   {
//      handTrajectoryMessages.put(RobotSide.LEFT, new HandTrajectoryMessage(RobotSide.LEFT, this.trajectoryTime, new Point3D(0.5, 0.35, 1.0), new Quaternion(), ReferenceFrame.getWorldFrame()));
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      handTrajectoryMessages.put(RobotSide.RIGHT, new HandTrajectoryMessage(RobotSide.RIGHT, this.trajectoryTime, new Point3D(0.5, -0.35, 1.0), new Quaternion(), worldFrame));
      
      
      
            
      int numberOfTrajectoryPoints = path.size();
      
      HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage(RobotSide.LEFT, numberOfTrajectoryPoints);
      
      handTrajectoryMessage.getFrameInformation().setTrajectoryReferenceFrame(worldFrame);
      handTrajectoryMessage.getFrameInformation().setDataReferenceFrame(worldFrame);
      
      EuclideanTrajectoryPointCalculator euclideanTrajectoryPointCalculator = new EuclideanTrajectoryPointCalculator();
      
      for (int i = 0; i < numberOfTrajectoryPoints; i++)
      {
         CTTaskNode trajectoryNode = path.get(i);
         
         ConfigurationSpace configurationSpace = CTTreeTools.getConfigurationSpace(trajectoryNode, RobotSide.LEFT);
         
         Pose3D desiredPose = constrainedEndEffectorTrajectory.getEndEffectorPose(trajectoryNode.getNodeData(0), RobotSide.LEFT, configurationSpace);
         
         
         
         
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
         
         ConfigurationSpace configurationSpace = CTTreeTools.getConfigurationSpace(trajectoryNode, RobotSide.LEFT);
         
         Pose3D desiredPose = constrainedEndEffectorTrajectory.getEndEffectorPose(trajectoryNode.getNodeData(0), RobotSide.LEFT, configurationSpace);
         
         
         
         
         
         
         
         Point3D desiredPosition = new Point3D();
         Vector3D desiredLinearVelocity = new Vector3D();
         Quaternion desiredOrientation = new Quaternion( desiredPose.getOrientation()  );
         desiredOrientation.appendRollRotation(Math.PI * 0.5);
         Vector3D desiredAngularVelocity = new Vector3D();

         double time = trajectoryPoints.get(i).get(desiredPosition, desiredLinearVelocity);

         handTrajectoryMessage.setTrajectoryPoint(i, time, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity, worldFrame);
      }
      
      
      
      
      
      
      
      
      
      
      
      
      
      handTrajectoryMessages.put(RobotSide.LEFT, handTrajectoryMessage);
   }
   
   private void updateChestTrajectoryMessage()   
   {
      
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
