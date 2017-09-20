package us.ihmc.robotics.robotDescription;

import java.util.ArrayList;
import java.util.List;

public interface RobotDescriptionNode
{
   public abstract String getName();
   public abstract ArrayList<JointDescription> getChildrenJoints();

   
   public void scale(double factor, double massScalePower, List<String> ignoreInertiaScaleJointList);

}
