package us.ihmc.robotics.robotDescription;

import java.util.ArrayList;

public class RobotDescription
{
   private String name;
   private final ArrayList<JointDescription> rootJoints = new ArrayList<>();

   public RobotDescription(String name)
   {
      this.setName(name);
   }

   public void addRootJoint(JointDescription rootJoint)
   {
      this.rootJoints.add(rootJoint);
   }

   public String getName()
   {
      return name;
   }

   public void setName(String name)
   {
      this.name = name;
   }
}
