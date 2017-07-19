package us.ihmc.manipulation.planning.trajectory;

import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;

public class WipingTrajectory extends ConstrainedEndEffectorTrajectory
{

   public WipingTrajectory(double trajectoryTime)
   {
      super(trajectoryTime);
   }

   @Override
   public SelectionMatrix6D defineControllableSelectionMatrix()
   {
      return null;
   }

   @Override
   public ConfigurationBuildOrder defineConfigurationBuildOrder()
   {
      return null;
   }

   @Override
   protected RobotSide defineRobotSide()
   {
      return null;
   }

   @Override
   protected ConfigurationSpace getConfigurationSpace(double time)
   {
      return null;
   }

   
   
}
