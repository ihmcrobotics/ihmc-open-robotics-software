package us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;

public abstract class ConstrainedEndEffectorTrajectory
{
   protected TaskRegion taskNodeRegion;
   protected SideDependentList<ConfigurationBuildOrder> configurationBuildOrders;
   protected SideDependentList<SelectionMatrix6D> controllableSelectionMatrices;
   protected double trajectoryTime;

   public ConstrainedEndEffectorTrajectory(double trajectoryTime)
   {
      this.trajectoryTime = trajectoryTime;

      this.controllableSelectionMatrices = defineControllableSelectionMatrices();
      this.configurationBuildOrders = defineConfigurationBuildOrders();
      this.taskNodeRegion = defineTaskRegion();
   }

   public void setTrajectoryTime(double trajectoryTime)
   {
      this.trajectoryTime = trajectoryTime;
   }

   public double getTrajectoryTime()
   {
      return this.trajectoryTime;
   }

   public Pose3D getEndEffectorPose(double time, RobotSide robotSide, ConfigurationSpace controllableConfigurationSpace)
   {
      ConfigurationSpace constrainedConfigurationSpace = getConfigurationSpace(time).get(robotSide);

      ConfigurationSpace finalConfigurationSpace = constrainedConfigurationSpace.overrideConfigurationSpaceCopy(controllableSelectionMatrices.get(robotSide),
                                                                                                                controllableConfigurationSpace);

      Point3D translation;
      Quaternion orientation;
      Pose3D pose3D;
      translation = new Point3D(finalConfigurationSpace.createRigidBodyTransform(configurationBuildOrders.get(robotSide)).getTranslationVector());
      orientation = new Quaternion(finalConfigurationSpace.createRigidBodyTransform(configurationBuildOrders.get(robotSide)).getRotationMatrix());
      pose3D = new Pose3D(translation, orientation);

      return pose3D;
   }

   public TaskRegion getTaskRegion()
   {
      return this.taskNodeRegion;
   }

   public abstract SideDependentList<SelectionMatrix6D> defineControllableSelectionMatrices();

   public abstract SideDependentList<ConfigurationBuildOrder> defineConfigurationBuildOrders();

   public abstract TaskRegion defineTaskRegion();

   public abstract SideDependentList<ConfigurationSpace> getConfigurationSpace(double time);
}