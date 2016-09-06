package us.ihmc.SdfLoader;

import us.ihmc.SdfLoader.models.FullRobotModelFactory;
import us.ihmc.robotics.robotDescription.RobotDescription;

public interface SDFFullRobotModelFactory extends FullRobotModelFactory
{
   public abstract RobotDescription getRobotDescription();
}
