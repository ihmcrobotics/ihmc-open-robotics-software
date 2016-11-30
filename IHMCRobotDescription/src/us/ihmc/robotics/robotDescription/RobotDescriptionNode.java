package us.ihmc.robotics.robotDescription;

import java.util.ArrayList;

public interface RobotDescriptionNode
{
   public abstract String getName();
   public abstract ArrayList<JointDescription> getChildrenJoints();

   
   public void scale(double factor, double massScalePower);

}
