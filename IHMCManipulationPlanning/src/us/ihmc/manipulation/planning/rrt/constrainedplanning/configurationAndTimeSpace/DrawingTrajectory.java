package us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.TaskRegion;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConfigurationBuildOrder;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConfigurationSpace;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConstrainedEndEffectorTrajectory;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConfigurationBuildOrder.ConfigurationSpaceName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;

public class DrawingTrajectory extends ConstrainedEndEffectorTrajectory
{   
   public DrawingTrajectory(double trajectoryTime, RobotSide robotSide)
   {
      super(trajectoryTime);
      this.robotSide = robotSide;
   }

   @Override
   public SelectionMatrix6D defineControllableSelectionMatrix()
   {
      SelectionMatrix6D selectionMatrix6D = new SelectionMatrix6D();
      selectionMatrix6D.selectLinearX(false);
      selectionMatrix6D.selectLinearY(false);
      selectionMatrix6D.selectLinearZ(false);

      selectionMatrix6D.selectAngularX(false);
      selectionMatrix6D.selectAngularY(false);
      selectionMatrix6D.selectAngularZ(true);

      return selectionMatrix6D;
   }

   @Override
   public ConfigurationBuildOrder defineConfigurationBuildOrder()
   {
      ConfigurationBuildOrder configurationBuildOrder;
      configurationBuildOrder = new ConfigurationBuildOrder(ConfigurationSpaceName.X, ConfigurationSpaceName.Y, ConfigurationSpaceName.Z,
                                                            ConfigurationSpaceName.ROLL, ConfigurationSpaceName.PITCH, ConfigurationSpaceName.YAW);

      return configurationBuildOrder;
   }

   @Override
   protected RobotSide defineRobotSide()
   {
      return robotSide;
   }

   @Override
   protected ConfigurationSpace getConfigurationSpace(double time)
   {
      double convertable;
      if(robotSide == RobotSide.RIGHT)
         convertable = -1;
      else
         convertable = 1;
      
      double arcRadius = 0.35;
      /*
       * Draw Circle in clockwise.
       */
      double arcAngle = time / getTrajectoryTime() * Math.PI * 2;

      Point3D arcCenterPoint = new Point3D(0.55, 0.0, 1.2);
      Quaternion arcCenterOrientation = new Quaternion();
      arcCenterOrientation.appendPitchRotation(-Math.PI * 0.5);
      RigidBodyTransform arcCenterRigidBodyController = new RigidBodyTransform(arcCenterOrientation, arcCenterPoint);

      arcCenterRigidBodyController.appendYawRotation(-arcAngle);
      arcCenterRigidBodyController.appendTranslation(0, arcRadius*convertable, 0);
      arcCenterRigidBodyController.appendYawRotation(arcAngle);

      ConfigurationSpace configurationSpace = new ConfigurationSpace();
      configurationSpace.setTranslation(arcCenterRigidBodyController.getTranslationVector());
      configurationSpace.setRotation(0, -0.5 * Math.PI, 0);

      return configurationSpace;
   }

   @Override
   public TaskRegion defineTaskRegion()
   {
      TaskRegion taskNodeRegion = new TaskRegion(GenericTaskNode.nodeDimension);
      
      taskNodeRegion.setRandomRegion(0, 0.0, trajectoryTime);
      taskNodeRegion.setRandomRegion(1, 0.75, 0.90);
      taskNodeRegion.setRandomRegion(2, -25.0 / 180 * Math.PI, 25.0 / 180 * Math.PI);
      taskNodeRegion.setRandomRegion(3, -20.0 / 180 * Math.PI, 20.0 / 180 * Math.PI);
      taskNodeRegion.setRandomRegion(4, -3.0 / 180 * Math.PI, 3.0 / 180 * Math.PI);
      taskNodeRegion.setRandomRegion(5, 0.0, 0.0);
      taskNodeRegion.setRandomRegion(6, 0.0, 0.0);
      taskNodeRegion.setRandomRegion(7, 0.0, 0.0);
      taskNodeRegion.setRandomRegion(8, 0.0, 0.0);
      taskNodeRegion.setRandomRegion(9, 0.0, 0.0);
      taskNodeRegion.setRandomRegion(10, -180.0/180*Math.PI, 0.0/180*Math.PI);
      
      return taskNodeRegion;
   }

}
