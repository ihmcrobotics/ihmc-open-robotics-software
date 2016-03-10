package us.ihmc.humanoidBehaviors.behaviors.scripts.engine;


import java.io.IOException;
import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.humanoidRobotics.communication.packets.walking.ComHeightPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.EndOfScriptCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootPosePacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepData;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataList;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage.FootstepOrigin;
import us.ihmc.humanoidRobotics.communication.packets.walking.PauseCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.PauseWalkingMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisHeightTrajectoryMessage;

public class RefactoringToNewPacketsScriptTransformer extends ScriptTransformer
{

   public RefactoringToNewPacketsScriptTransformer(String scriptDirectoryPath) throws IOException, InterruptedException
   {
      super(scriptDirectoryPath);
   }

   @Override
   public Object transformScriptObject(Object object)
   {
      if (object instanceof ComHeightPacket)
      {
         ComHeightPacket comHeightPacket = (ComHeightPacket) object;

         PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = new PelvisHeightTrajectoryMessage(1);
         double position = comHeightPacket.heightOffset + 0.65;
         double velocity = 0.0;
         pelvisHeightTrajectoryMessage.setTrajectoryPoint(0, comHeightPacket.trajectoryTime, position, velocity);
         
         return pelvisHeightTrajectoryMessage;
      }
      else if (object instanceof FootPosePacket)
      {
         FootPosePacket footPosePacket = (FootPosePacket) object;

         FootTrajectoryMessage footTrajectoryMessage = new FootTrajectoryMessage(footPosePacket.getRobotSide(), 1);
         
         Point3d position = footPosePacket.position;
         Quat4d orientation = footPosePacket.orientation;
         Vector3d linearVelocity = new Vector3d();
         Vector3d angularVelocity = new Vector3d();
         footTrajectoryMessage.setTrajectoryPoint(0, 1.0, position, orientation, linearVelocity, angularVelocity);
         
         return footTrajectoryMessage;
      }
      else if (object instanceof FootstepDataList)
      {
         FootstepDataList footstepDataList = (FootstepDataList) object;

         FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage(footstepDataList.swingTime, footstepDataList.transferTime);
         
         ArrayList<FootstepData> oldFootstepDataList = footstepDataList.footstepDataList;
         
         for (FootstepData footstepData : oldFootstepDataList)
         {
            FootstepDataMessage footstepDataMessage = new FootstepDataMessage(footstepData.robotSide, footstepData.location, footstepData.orientation);
            footstepDataMessage.setOrigin(FootstepOrigin.AT_ANKLE_FRAME);
            footstepDataMessage.setSwingHeight(footstepData.swingHeight);
            footstepDataListMessage.add(footstepDataMessage);
         }

         return footstepDataListMessage;
      }
      else if (object instanceof PauseCommand)
      {
         PauseWalkingMessage pauseWalkingMessage = new PauseWalkingMessage(true);
         return pauseWalkingMessage;
      }
      else if (object instanceof EndOfScriptCommand)
      {
         return object;
      }
      else
      {
         System.err.println("No Transformation for objects of type " + object.getClass());
      }
      
      return object;
   }
   
   public static void main(String[] args) throws IOException, InterruptedException
   {
      ArrayList<String> paths = new ArrayList<>();
//      paths.add("..\\Atlas\\scripts");
//      paths.add("..\\DarpaRoboticsChallenge\\scripts");
//      paths.add("..\\DarpaRoboticsChallenge\\scriptsSaved");
      paths.add("..\\DarpaRoboticsChallenge\\resources\\scripts\\ExerciseAndJUnitScripts");
//      paths.add("..\\IHMCHumanoidBehaviors\\scripts");
//      paths.add("..\\IHMCHumanoidBehaviors\\\resources\\scripts");
//      paths.add("..\\IHMCHumanoidOperatorInterface\\scripts");
//      paths.add("..\\IHMCHumanoidOperatorInterface\\resources\\finalScripts");

//      ArrayList<String> newPaths = moveScriptDirectories(paths, ScriptTransformer.ORIGINAL);

      int index = 0;
      for (String path : paths)
      {
         System.out.println("Transforming scripts in: " + paths.get(index++));
         new RefactoringToNewPacketsScriptTransformer(path);
      }
   }
}
