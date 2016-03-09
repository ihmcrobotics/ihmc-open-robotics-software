package us.ihmc.humanoidBehaviors.behaviors.scripts.engine;


import java.io.IOException;
import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.humanoidRobotics.communication.packets.walking.ComHeightPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootPosePacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
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
         double position = comHeightPacket.trajectoryTime;
         double velocity = 0.0;
         pelvisHeightTrajectoryMessage.setTrajectoryPoint(0, 1.0, position, velocity);
         
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

      ArrayList<String> newPaths = moveScriptDirectories(paths, ScriptTransformer.ORIGINAL);

      int index = 0;
      for (String path : newPaths)
      {
         System.out.println("Transforming scripts in: " + paths.get(index++));
         new RefactoringToNewPacketsScriptTransformer(path);
      }
   }
}
