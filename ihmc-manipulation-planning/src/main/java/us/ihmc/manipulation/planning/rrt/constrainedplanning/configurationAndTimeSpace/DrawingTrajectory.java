package us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConfigurationBuildOrder;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConfigurationBuildOrder.ConfigurationSpaceName;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConfigurationSpace;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConstrainedEndEffectorTrajectory;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.GenericTaskNode;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.TaskRegion;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;

public class DrawingTrajectory extends ConstrainedEndEffectorTrajectory
{
   public DrawingTrajectory(double trajectoryTime)
   {
      super(trajectoryTime);
      
      Point3D arcCenterPoint = new Point3D(0.56, 0.0, 1.1);
      Quaternion arcCenterOrientation = new Quaternion();
      arcCenterOrientation.appendPitchRotation(-Math.PI * 0.48);
      
      RigidBodyTransform wallRigidBodyTransform = new RigidBodyTransform(arcCenterOrientation, arcCenterPoint);
      System.out.println(wallRigidBodyTransform);
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
      selectionMatrices.get(RobotSide.LEFT).selectAngularZ(true);

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
   public SideDependentList<ConfigurationSpace> getConfigurationSpace(double time)
   {
      double convertable = 1;
      double arcRadius = 0.35;

      /*
       * Draw Circle in clockwise.
       */
      double arcAngle = time / getTrajectoryTime() * Math.PI * 2;

      Point3D arcCenterPoint = new Point3D(0.53, 0.0, 1.2);
      Quaternion arcCenterOrientation = new Quaternion();
      arcCenterOrientation.appendPitchRotation(-Math.PI * 0.48);
      
      RigidBodyTransform arcCenterRigidBodyController = new RigidBodyTransform(arcCenterOrientation, arcCenterPoint);

      arcCenterRigidBodyController.appendYawRotation(-arcAngle);
      arcCenterRigidBodyController.appendTranslation(0, arcRadius * convertable, 0);
      arcCenterRigidBodyController.appendYawRotation(arcAngle);

      SideDependentList<ConfigurationSpace> configurationSpaces = new SideDependentList<>();

      ConfigurationSpace holdingConfiguration = new ConfigurationSpace();

      holdingConfiguration.setTranslation(-0.2, -0.5, 0.60);
      holdingConfiguration.setRotation(0.5 * Math.PI, 0.0, -0.4 * Math.PI);

      configurationSpaces.put(RobotSide.RIGHT, holdingConfiguration);

      ConfigurationSpace drawingConfigurationSpace = new ConfigurationSpace();
      drawingConfigurationSpace.setTranslation(arcCenterRigidBodyController.getTranslationVector());
      drawingConfigurationSpace.setRotation(0, -0.4 * Math.PI, -0.0*Math.PI);

      configurationSpaces.put(RobotSide.LEFT, drawingConfigurationSpace);

      return configurationSpaces;
   }

   @Override
   public TaskRegion defineTaskRegion()
   {
      TaskRegion taskNodeRegion = new TaskRegion(GenericTaskNode.nodeDimension);

      taskNodeRegion.setRandomRegion(0, 0.0, trajectoryTime);

      taskNodeRegion.setRandomRegion(1, 0.75, 0.90);
      taskNodeRegion.setRandomRegion(2, -20.0 / 180 * Math.PI, 20.0 / 180 * Math.PI);
      taskNodeRegion.setRandomRegion(3, -20.0 / 180 * Math.PI, 20.0 / 180 * Math.PI);
      taskNodeRegion.setRandomRegion(4, -3.0 / 180 * Math.PI, 3.0 / 180 * Math.PI);

      taskNodeRegion.setRandomRegion(5, 0.0, 0.0);
      taskNodeRegion.setRandomRegion(6, 0.0, 0.0);
      taskNodeRegion.setRandomRegion(7, 0.0, 0.0);
      taskNodeRegion.setRandomRegion(8, 0.0, 0.0);
      taskNodeRegion.setRandomRegion(9, 0.0, 0.0);
      taskNodeRegion.setRandomRegion(10, -160.0 / 180 * Math.PI, -20.0 / 180 * Math.PI);

      taskNodeRegion.setRandomRegion(11, 0.0, 0.0);
      taskNodeRegion.setRandomRegion(12, 0.0, 0.0);
      taskNodeRegion.setRandomRegion(13, 0.0, 0.0);
      taskNodeRegion.setRandomRegion(14, 0.0, 0.0);
      taskNodeRegion.setRandomRegion(15, 0.0, 0.0);
      taskNodeRegion.setRandomRegion(16, 0.0, 0.0);
      
//      taskNodeRegion.setRandomRegion(1, 0.75, 0.90);
//      taskNodeRegion.setRandomRegion(2, -20.0 / 180 * Math.PI, 5.0 / 180 * Math.PI);
//      taskNodeRegion.setRandomRegion(3, -10.0 / 180 * Math.PI, 20.0 / 180 * Math.PI);
//      taskNodeRegion.setRandomRegion(4, -3.0 / 180 * Math.PI, 3.0 / 180 * Math.PI);
//
//      taskNodeRegion.setRandomRegion(5, 0.0, 0.0);
//      taskNodeRegion.setRandomRegion(6, 0.0, 0.0);
//      taskNodeRegion.setRandomRegion(7, 0.0, 0.0);
//      taskNodeRegion.setRandomRegion(8, 0.0, 0.0);
//      taskNodeRegion.setRandomRegion(9, 0.0, 0.0);
//      taskNodeRegion.setRandomRegion(10, -150.0 / 180 * Math.PI, -40.0 / 180 * Math.PI);
//
//      taskNodeRegion.setRandomRegion(11, 0.0, 0.0);
//      taskNodeRegion.setRandomRegion(12, 0.0, 0.0);
//      taskNodeRegion.setRandomRegion(13, 0.0, 0.0);
//      taskNodeRegion.setRandomRegion(14, 0.0, 0.0);
//      taskNodeRegion.setRandomRegion(15, 0.0, 0.0);
//      taskNodeRegion.setRandomRegion(16, 0.0, 0.0);

      return taskNodeRegion;
   }

}