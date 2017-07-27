package us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.ConfigurationBuildOrder.ConfigurationSpaceName;
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

      Point3D arcCenterPoint = new Point3D(0.6, 0.0, 1.1);
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

}
