package us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.rotationConversion.YawPitchRollConversion;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.DualWayPointsPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConfigurationBuildOrder;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConfigurationBuildOrder.ConfigurationSpaceName;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConfigurationSpace;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConstrainedEndEffectorTrajectory;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.GenericTaskNode;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.TaskRegion;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;

public class DualWayPointsTrajectory extends ConstrainedEndEffectorTrajectory
{
   private SideDependentList<Pose3D[]> poseOfWayPoints;

   public DualWayPointsTrajectory(double trajectoryTime)
   {
      super(trajectoryTime);
   }

   public DualWayPointsTrajectory(DualWayPointsPacket dualWayPointsPacket)
   {
      super(dualWayPointsPacket.trajectoryTime);

      poseOfWayPoints = new SideDependentList<Pose3D[]>();
      
      poseOfWayPoints.put(RobotSide.LEFT, new Pose3D[dualWayPointsPacket.numberOfWayPointsLeft]);
      poseOfWayPoints.put(RobotSide.RIGHT, new Pose3D[dualWayPointsPacket.numberOfWayPointsRight]);

      for (int i = 0; i < poseOfWayPoints.get(RobotSide.LEFT).length; i++)
      {
         poseOfWayPoints.get(RobotSide.LEFT)[i] = new Pose3D();
         
         poseOfWayPoints.get(RobotSide.LEFT)[i].setPosition(dualWayPointsPacket.positionOfWayPointsLeft[i]);
         poseOfWayPoints.get(RobotSide.LEFT)[i].setOrientation(dualWayPointsPacket.orientationOfWayPointsLeft[i]);
      }

      for (int i = 0; i < poseOfWayPoints.get(RobotSide.RIGHT).length; i++)
      {
         poseOfWayPoints.get(RobotSide.RIGHT)[i] = new Pose3D();

         poseOfWayPoints.get(RobotSide.RIGHT)[i].setPosition(dualWayPointsPacket.positionOfWayPointsRight[i]);
         poseOfWayPoints.get(RobotSide.RIGHT)[i].setOrientation(dualWayPointsPacket.orientationOfWayPointsRight[i]);
      }      
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
                                                                  ConfigurationSpaceName.YAW, ConfigurationSpaceName.PITCH, ConfigurationSpaceName.ROLL));

      return configurationBuildOrders;
   }

   @Override
   public TaskRegion defineTaskRegion()
   {
      TaskRegion taskNodeRegion = new TaskRegion(GenericTaskNode.nodeDimension);

      taskNodeRegion.setRandomRegion(0, 0.0, trajectoryTime);

      taskNodeRegion.setRandomRegion(1, 0.75, 0.90);
      taskNodeRegion.setRandomRegion(2, -20.0 / 180 * Math.PI, 20.0 / 180 * Math.PI);
      taskNodeRegion.setRandomRegion(3, -20.0 / 180 * Math.PI, 20.0 / 180 * Math.PI);
      taskNodeRegion.setRandomRegion(4, -8.0 / 180 * Math.PI, 8.0 / 180 * Math.PI);

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
      SideDependentList<ConfigurationSpace> configurationSpaces = new SideDependentList<>();
      
      if(time > this.trajectoryTime)
         time = this.trajectoryTime;
      
      for(RobotSide robotSide:RobotSide.values)
      {
         int numberOfSegments = poseOfWayPoints.get(robotSide).length-1;
         
         ConfigurationSpace controlConfiguration = new ConfigurationSpace();
         
         if(numberOfSegments > 0)
         {
            double[] segmentTimes = new double[numberOfSegments];
            double[] segmentDistance = new double[numberOfSegments];
            
            double totalDistance = 0.0;
            for(int i=0;i<segmentDistance.length;i++)
            {
               segmentDistance[i] = getSegmentDistance(poseOfWayPoints.get(robotSide)[i], poseOfWayPoints.get(robotSide)[i+1]);
               totalDistance = totalDistance + segmentDistance[i];         
            }
            
            for(int i=0;i<segmentTimes.length;i++)
            {
               segmentTimes[i] = this.trajectoryTime * segmentDistance[i] / totalDistance;
            }
          
            int indexOfSegment = 0;
            double leftTime = time;
            double alpha = 0.0;
            
            for(int i=0;i<segmentTimes.length;i++)
            {  
               leftTime = leftTime - segmentTimes[i];
               
               if(leftTime <= 0)
               {
                  indexOfSegment = i;
                  leftTime = leftTime + segmentTimes[i];
                  
                  alpha = leftTime / segmentTimes[i];
                  
                  break;
               }
               
               if(i==segmentTimes.length - 1 && leftTime < 1.0e-3)
               {
                  indexOfSegment = i;
                  alpha = 1.0;
                  break;
               }
                
               if(i==segmentTimes.length - 1)
                  PrintTools.warn("time calculating is wrong");
            }
            
            Point3D interpolatedPoint = new Point3D();
            interpolatedPoint.interpolate(poseOfWayPoints.get(robotSide)[indexOfSegment].getPosition(), poseOfWayPoints.get(robotSide)[indexOfSegment+1].getPosition(), alpha);
            
            Quaternion interpolatedOrientation = new Quaternion();
            interpolatedOrientation.interpolate(poseOfWayPoints.get(robotSide)[indexOfSegment].getOrientation(), poseOfWayPoints.get(robotSide)[indexOfSegment+1].getOrientation(), alpha);
            
            Vector3D interpolatedOrientationYPR = new Vector3D();
            YawPitchRollConversion.convertQuaternionToYawPitchRoll(interpolatedOrientation, interpolatedOrientationYPR);
            
            
            
            controlConfiguration.setTranslation(interpolatedPoint);
            controlConfiguration.setRotation(interpolatedOrientationYPR.getX(), interpolatedOrientationYPR.getY(), interpolatedOrientationYPR.getZ());

            
         }
         else
         {
            Point3D interpolatedPoint = new Point3D();
            interpolatedPoint.set(poseOfWayPoints.get(robotSide)[0].getPosition());
            
            Quaternion interpolatedOrientation = new Quaternion();
            interpolatedOrientation.set(poseOfWayPoints.get(robotSide)[0].getOrientation());
            
            Vector3D interpolatedOrientationYPR = new Vector3D();
            YawPitchRollConversion.convertQuaternionToYawPitchRoll(interpolatedOrientation, interpolatedOrientationYPR);
            
            controlConfiguration.setTranslation(interpolatedPoint);
            controlConfiguration.setRotation(interpolatedOrientationYPR.getX(), interpolatedOrientationYPR.getY(), interpolatedOrientationYPR.getZ());
         }
         
         configurationSpaces.put(robotSide, controlConfiguration);  
      }
            
      return configurationSpaces;
   }
   
   private double getSegmentDistance(Pose3D pose1, Pose3D pose2)
   {
      double ratioOrientationToPosition = 1.0;
      
      double orientationDistance = 0.0;
      ManipulationTools.computeDisplacementQuaternion(new Quaternion(pose1.getOrientation()), new Quaternion(pose2.getOrientation()), orientationDistance);
      
      orientationDistance = Math.abs(orientationDistance);
      
      return pose2.getPosition().distance(pose1.getPosition()) + orientationDistance * ratioOrientationToPosition;
   }
}
