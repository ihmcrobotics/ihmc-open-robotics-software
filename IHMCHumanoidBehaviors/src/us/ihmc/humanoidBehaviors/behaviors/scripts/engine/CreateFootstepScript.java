package us.ihmc.humanoidBehaviors.behaviors.scripts.engine;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.EnumMap;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.walking.EndOfScriptCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PauseWalkingMessage;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.io.printing.PrintTools;

public class CreateFootstepScript
{
   // swing and transfer time are overwritten by UI commands
   private static final double desiredSwingTime = 0.6;
   private static final double desiredTransferTime = 0.0;

   private static final double stepLength = 0.3;
   private static final double stepWidth = 0.3;

   private enum ContactType {
      FULL,
      LINE_FRONT_BACK,
      LINE_SIDE_SIDE,
      HALF_FRONT,
      HALF_BACK
   }

   private ContactType[] contactSequence = {
         ContactType.LINE_SIDE_SIDE,
         ContactType.LINE_FRONT_BACK};

   private static final double footLength = 0.22;
   private static final double footWidth = 0.11;
   private static final double lineWidth = 0.02;
   private static final double ankleHeight = 0.084;

   private static final Point2D[] contactPointsFull = {
         new Point2D(footLength/2.0, -footWidth/2.0),
         new Point2D(footLength/2.0, footWidth/2.0),
         new Point2D(-footLength/2.0, footWidth/2.0),
         new Point2D(-footLength/2.0, -footWidth/2.0)};
   private static final Point2D[] contactPointsLineFrontToBack = {
         new Point2D(footLength/2.0, -lineWidth/2.0),
         new Point2D(footLength/2.0, lineWidth/2.0),
         new Point2D(-footLength/2.0, lineWidth/2.0),
         new Point2D(-footLength/2.0, -lineWidth/2.0)};
   private static final Point2D[] contactPointsLineSideToSide = {
         new Point2D(lineWidth/2.0, -footWidth/2.0),
         new Point2D(lineWidth/2.0, footWidth/2.0),
         new Point2D(-lineWidth/2.0, footWidth/2.0),
         new Point2D(-lineWidth/2.0, -footWidth/2.0)};
   private static final Point2D[] contactPointsHalfFront = {
         new Point2D(footLength/2.0, -footWidth/2.0),
         new Point2D(footLength/2.0, footWidth/2.0),
         new Point2D(0.0, footWidth/2.0),
         new Point2D(0.0, -footWidth/2.0)};
   private static final Point2D[] contactPointsHalfBack = {
         new Point2D(0.0, -footWidth/2.0),
         new Point2D(0.0, footWidth/2.0),
         new Point2D(-footLength/2.0, footWidth/2.0),
         new Point2D(-footLength/2.0, -footWidth/2.0)};

   private static final EnumMap<ContactType, Point2D[]> contactPointMap = new EnumMap<>(ContactType.class);
   static
   {
      contactPointMap.put(ContactType.FULL, contactPointsFull);
      contactPointMap.put(ContactType.LINE_FRONT_BACK, contactPointsLineFrontToBack);
      contactPointMap.put(ContactType.LINE_SIDE_SIDE, contactPointsLineSideToSide);
      contactPointMap.put(ContactType.HALF_FRONT, contactPointsHalfFront);
      contactPointMap.put(ContactType.HALF_BACK, contactPointsHalfBack);
   }

   public CreateFootstepScript(String filePath) throws IOException, InterruptedException
   {
      File file = new File(filePath);
      if (file.isDirectory())
      {
         System.err.println("Expected file - got a directory");
         return;
      }

      File newScript = new File(file.getParentFile(), file.getName());
      ScriptFileSaver scriptFileSaver = null;
      try
      {
         scriptFileSaver = new ScriptFileSaver(newScript, true);
      }
      catch (IOException e)
      {
         PrintTools.error("During writing: " + e.getClass().getSimpleName() + " for the file:" + newScript);
         if (e.getCause() != null)
            PrintTools.error(e.getCause().getMessage());
         else
            PrintTools.error(e.getMessage());
         return;
      }

      ArrayList<Object> scriptObjects = new ArrayList<>();
      assembleScript(scriptObjects);

      for (Object newObject : scriptObjects)
      {
         ScriptObject newScriptObject = new ScriptObject(System.currentTimeMillis(), newObject);
         scriptFileSaver.recordObject(newScriptObject.getTimeStamp(), newScriptObject.getScriptObject());
      }
      scriptFileSaver.close();
      System.out.println("Created script: " + newScript);
   }

   private void assembleScript(ArrayList<Object> scriptObjects)
   {
      scriptObjects.add(new PauseWalkingMessage(true));
      FootstepDataListMessage footsteps = new FootstepDataListMessage(desiredSwingTime, desiredTransferTime);
      addFootsteps(footsteps);
      scriptObjects.add(footsteps);
      scriptObjects.add(new PauseWalkingMessage(true));
      scriptObjects.add(new EndOfScriptCommand());
   }

   private void addFootsteps(FootstepDataListMessage footsteps)
   {
      footsteps.footstepDataList.clear();
      int idx = 0;
      for (ContactType contactType : contactSequence)
      {
         footsteps.footstepDataList.add(createFootstep(contactType, idx++));
      }
      footsteps.footstepDataList.add(createFootstep(ContactType.FULL, idx++));
      footsteps.footstepDataList.add(createFootstep(ContactType.FULL, idx++));
   }

   private FootstepDataMessage createFootstep(ContactType contactType, int idx)
   {
      FootstepDataMessage footstep = new FootstepDataMessage();
      RobotSide robotSide = idx%2 == 0 ? RobotSide.LEFT : RobotSide.RIGHT;
      double x = Math.floor((double)(idx+2) / 2.0) * stepLength;
      double y = robotSide == RobotSide.RIGHT ? -stepWidth : 0.0;
      Point2D[] contactPoints = contactPointMap.get(contactType);

      // set robot side
      footstep.robotSide = robotSide;
      // set pose
      footstep.location = new Point3D(x, y, ankleHeight);
      footstep.setOrientation(new Quaternion(0.0, 0.0, 0.0, 1.0));
      // set contact points
      footstep.predictedContactPoints = new ArrayList<>();
      for (int i = 0; i < contactPoints.length; i++)
      {
         Point2D contactPoint = new Point2D(contactPoints[i]);
         footstep.predictedContactPoints.add(contactPoint);
      }

      return footstep;
   }

   public static void main(String[] args) throws IOException, InterruptedException
   {
      ArrayList<Path> paths = new ArrayList<>();
      paths.add(Paths.get("..", "Atlas", "scripts", "partialFootholds_sequence4.xml"));

      for (Path path : paths)
      {
         String pathString = path.toString();
         System.out.println("Creating script: " + pathString);
         new CreateFootstepScript(pathString);
      }
   }
}
