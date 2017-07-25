package us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;

public abstract class ConstrainedEndEffectorTrajectory implements ConstrainedConfigurationSpace
{
   protected ConfigurationBuildOrder configurationBuildOrder;
   protected SelectionMatrix6D controllableSelectionMatrix;
   protected double trajectoryTime;
   protected RobotSide robotSide;

   public ConstrainedEndEffectorTrajectory(double trajectoryTime)
   {
      this.trajectoryTime = trajectoryTime;

      this.robotSide = defineRobotSide();
      this.controllableSelectionMatrix = defineControllableSelectionMatrix();
      this.configurationBuildOrder = defineConfigurationBuildOrder();
   }

   public void setTrajectoryTime(double trajectoryTime)
   {
      this.trajectoryTime = trajectoryTime;
   }

   public double getTrajectoryTime()
   {
      return this.trajectoryTime;
   }

   public RobotSide getRobotSide()
   {
      if (robotSide == null)
         PrintTools.warn("RobotSide of the end effector should be defined.");
      return this.robotSide;
   }

   public RobotSide getAnotherRobotSide()
   {
      if (getRobotSide() == RobotSide.RIGHT)
         return RobotSide.LEFT;
      else
         return RobotSide.RIGHT;
   }

   public Pose3D getEndEffectorPose(double time, ConfigurationSpace controllableConfigurationSpace)
   {
      ConfigurationSpace constrainedConfigurationSpace = getConfigurationSpace(time);

      ConfigurationSpace finalConfigurationSpace = constrainedConfigurationSpace.overrideConfigurationSpaceCopy(controllableSelectionMatrix,
                                                                                                                controllableConfigurationSpace);

      Point3D translation;
      Quaternion orientation;
      Pose3D pose3D;
      translation = new Point3D(finalConfigurationSpace.createRigidBodyTransform(configurationBuildOrder).getTranslationVector());
      orientation = new Quaternion(finalConfigurationSpace.createRigidBodyTransform(configurationBuildOrder).getRotationMatrix());
      pose3D = new Pose3D(translation, orientation);

      return pose3D;
   }

   protected abstract RobotSide defineRobotSide();

   protected abstract ConfigurationSpace getConfigurationSpace(double time);

}
