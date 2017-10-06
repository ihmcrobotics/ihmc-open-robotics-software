package us.ihmc.manipulation.planning.solarpanelmotion;

import java.util.ArrayList;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.robotics.robotSide.RobotSide;

public class SolarPanelPath
{
   private ArrayList<SolarPanelCleaningPose> wayPoses = new ArrayList<SolarPanelCleaningPose>();
   private ArrayList<Double> arrivalTime = new ArrayList<Double>();
   private ArrayList<SolarPanelLinearPath> linearPath = new ArrayList<SolarPanelLinearPath>();

   private WholeBodyTrajectoryMessage wholebodyMessageForPath;

   public SolarPanelPath(SolarPanelCleaningPose startPose)
   {
      wayPoses.add(startPose);
      arrivalTime.add(0.0);
   }
   
   public SolarPanelCleaningPose getStartPose()
   {
      return wayPoses.get(0);
   }
   
   public SolarPanel getSolarPanel()
   {
      return wayPoses.get(0).getSolarPanel();
   }
   
   public int getNumerOfLinearPath()
   {
      return linearPath.size();
   }
   public ArrayList<Double> getArrivalTime()
   {
      return arrivalTime;
   }

   public ArrayList<SolarPanelLinearPath> getLinearPath()
   {
      return linearPath;
   }

   public void addCleaningPose(SolarPanelCleaningPose cleaningPose, double timeToGo)
   {
      SolarPanelCleaningPose newStartPose = wayPoses.get(wayPoses.size()-1);
      double startTime = arrivalTime.get(arrivalTime.size()-1);
      double endTime = startTime + timeToGo;

      SolarPanelLinearPath linearPath = new SolarPanelLinearPath(newStartPose, cleaningPose, startTime, endTime);
      this.linearPath.add(linearPath);

      this.wayPoses.add(cleaningPose);
      this.arrivalTime.add(endTime);
   }

   public WholeBodyTrajectoryMessage getWholeBodyMessage()
   {
      return wholebodyMessageForPath;
   }

   public WholeBodyTrajectoryMessage getWholeBodyMessageForValidityTest(double zRotation, double pelvisYaw, double pelvisHeight, double time)
   {
      WholeBodyTrajectoryMessage message = new WholeBodyTrajectoryMessage();
      SolarPanelCleaningPose cleaningPose = getCleaningPose(time);
      cleaningPose.setZRotation(zRotation);

      double defaultMotionTime = 3.0;

      HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage(RobotSide.RIGHT, defaultMotionTime,
                                                                              cleaningPose.getDesiredHandPosition(), cleaningPose.getDesiredHandOrientation(),
                                                                              ReferenceFrame.getWorldFrame());
      //PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = new PelvisHeightTrajectoryMessage(defaultMotionTime, pelvisHeight);

      Quaternion desiredChestOrientation = new Quaternion();
      desiredChestOrientation.appendYawRotation(pelvisYaw);
      ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage(defaultMotionTime, desiredChestOrientation, ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());

      message.setHandTrajectoryMessage(handTrajectoryMessage);
      message.setChestTrajectoryMessage(chestTrajectoryMessage);
      //message.setPelvisTrajectoryMessage(pelvisHeightTrajectoryMessage);

      return message;
   }

   public SolarPanelCleaningPose getCleaningPose(double time)
   {
      int indexOfLinearPath = 0;

      if(arrivalTime.size() > 2)
      {
         for(int i =0;i<arrivalTime.size()-1;i++)
         {
            if(arrivalTime.get(i) < time && time <= arrivalTime.get(i+1))
            {
               indexOfLinearPath = i;
            }
         }
      }
      else
      {
         indexOfLinearPath = 0;
      }

      SolarPanelCleaningPose cleaningPose = linearPath.get(indexOfLinearPath).getInterpolatedCleaningPose(time);

      return cleaningPose;
   }


}
