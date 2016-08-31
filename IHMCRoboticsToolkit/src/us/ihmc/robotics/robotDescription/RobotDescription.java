package us.ihmc.robotics.robotDescription;

import java.util.ArrayList;

public class RobotDescription
{
   private String name;
   private final ArrayList<JointDescription> rootJoints = new ArrayList<>();

   public RobotDescription(String name)
   {
      this.name = name;
   }



}
