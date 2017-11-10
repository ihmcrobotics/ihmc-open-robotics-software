package us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationBuildOrder;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpace;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConstrainedEndEffectorTrajectory;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.GenericTaskNode;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.TaskRegion;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationBuildOrder.ConfigurationSpaceName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;

public class TrackingLineTrajectory extends ConstrainedEndEffectorTrajectory
{
   private Point3D initialPosition;
   private Point3D finalPosition;

   public TrackingLineTrajectory(Point3D initialPosition, Point3D finalPosition, double trajectoryTime)
   {
      super(trajectoryTime);
      this.initialPosition = initialPosition;
      this.finalPosition = finalPosition;
   }
   
   @Override
   public SideDependentList<SelectionMatrix6D> defineControllableSelectionMatrices()
   {
      SideDependentList<SelectionMatrix6D> selectionMatrices = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
         selectionMatrices.put(robotSide, new SelectionMatrix6D());

      selectionMatrices.get(RobotSide.LEFT).selectLinearX(false);
      selectionMatrices.get(RobotSide.LEFT).selectLinearY(false);
      selectionMatrices.get(RobotSide.LEFT).selectLinearZ(false);

      selectionMatrices.get(RobotSide.LEFT).selectAngularX(false);
      selectionMatrices.get(RobotSide.LEFT).selectAngularY(false);
      selectionMatrices.get(RobotSide.LEFT).selectAngularZ(false);

      selectionMatrices.get(RobotSide.RIGHT).selectLinearX(false);
      selectionMatrices.get(RobotSide.RIGHT).selectLinearY(false);
      selectionMatrices.get(RobotSide.RIGHT).selectLinearZ(false);

      selectionMatrices.get(RobotSide.RIGHT).selectAngularX(false);
      selectionMatrices.get(RobotSide.RIGHT).selectAngularY(false);
      selectionMatrices.get(RobotSide.RIGHT).selectAngularZ(false);

      return selectionMatrices;
   }

   @Override
   public SideDependentList<ConfigurationBuildOrder> defineConfigurationBuildOrders()
   {
      SideDependentList<ConfigurationBuildOrder> configurationBuildOrders = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
         configurationBuildOrders.put(robotSide,
                                      new ConfigurationBuildOrder(ConfigurationSpaceName.X, ConfigurationSpaceName.Y, ConfigurationSpaceName.Z,
                                                                  ConfigurationSpaceName.ROLL, ConfigurationSpaceName.PITCH, ConfigurationSpaceName.YAW));

      return configurationBuildOrders;
   }

   @Override
   public TaskRegion defineTaskRegion()
   {
      TaskRegion taskNodeRegion = new TaskRegion(GenericTaskNode.nodeDimension);

      taskNodeRegion.setRandomRegion(0, 0.0, trajectoryTime);

      taskNodeRegion.setRandomRegion(1, 0.85, 0.90);
      taskNodeRegion.setRandomRegion(2, -20.0 / 180 * Math.PI, 20.0 / 180 * Math.PI);
      taskNodeRegion.setRandomRegion(3, -20.0 / 180 * Math.PI, 20.0 / 180 * Math.PI);
      taskNodeRegion.setRandomRegion(4, -3.0 / 180 * Math.PI, 3.0 / 180 * Math.PI);

      taskNodeRegion.setRandomRegion(5, 0.0, 0.0);
      taskNodeRegion.setRandomRegion(6, 0.0, 0.0);
      taskNodeRegion.setRandomRegion(7, 0.0, 0.0);
      taskNodeRegion.setRandomRegion(8, 0.0, 0.0);
      taskNodeRegion.setRandomRegion(9, 0.0, 0.0);
      taskNodeRegion.setRandomRegion(10, 0.0, 0.0);

      taskNodeRegion.setRandomRegion(11, 0.0, 0.0);
      taskNodeRegion.setRandomRegion(12, 0.0, 0.0);
      taskNodeRegion.setRandomRegion(13, 0.0, 0.0);
      taskNodeRegion.setRandomRegion(14, 0.0, 0.0);
      taskNodeRegion.setRandomRegion(15, 0.0, 0.0);
      taskNodeRegion.setRandomRegion(16, 0.0, 0.0);

      return taskNodeRegion;
   }

   @Override
   public SideDependentList<ConfigurationSpace> getConfigurationSpace(double time)
   {
      
      
      
      
      return null;
   }

}
