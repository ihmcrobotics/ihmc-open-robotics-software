package us.ihmc.humanoidRobotics.communication.packets;

import controller_msgs.msg.dds.ChestTrajectoryMessage;
import controller_msgs.msg.dds.HandTrajectoryMessage;
import controller_msgs.msg.dds.KinematicsPlanningToolboxOutputStatus;
import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import controller_msgs.msg.dds.PelvisTrajectoryMessage;
import controller_msgs.msg.dds.SO3TrajectoryMessage;
import controller_msgs.msg.dds.SO3TrajectoryPointMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.math.trajectories.waypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.generators.EuclideanTrajectoryPointCalculator;
import us.ihmc.robotics.math.trajectories.waypoints.generators.SO3TrajectoryPointCalculator;
import us.ihmc.robotics.robotSide.RobotSide;

public class KinematicsPlanningToolboxOutputConverter
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage;
   private final KinematicsToolboxOutputConverter converter;

   public KinematicsPlanningToolboxOutputConverter(FullHumanoidRobotModelFactory fullRobotModelFactory)
   {
      converter = new KinematicsToolboxOutputConverter(fullRobotModelFactory);
   }

   private void computeHandTrajectoryMessages(KinematicsPlanningToolboxOutputStatus solution)
   {
      for (RobotSide robotSide : RobotSide.values)
         computeHandTrajectoryMessage(solution, robotSide);
   }

   private void computeHandTrajectoryMessage(KinematicsPlanningToolboxOutputStatus solution, RobotSide robotSide)
   {
      int numberOfTrajectoryPoints = solution.getRobotConfigurations().size();
      HandTrajectoryMessage trajectoryMessage = new HandTrajectoryMessage();
      trajectoryMessage.setRobotSide(robotSide.toByte());

      trajectoryMessage.getSe3Trajectory().getFrameInformation().setTrajectoryReferenceFrameId(MessageTools.toFrameId(worldFrame));
      trajectoryMessage.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(worldFrame));

      Point3D[] desiredPositions = new Point3D[numberOfTrajectoryPoints];
      Quaternion[] desiredOrientations = new Quaternion[numberOfTrajectoryPoints];

      EuclideanTrajectoryPointCalculator euclideanTrajectoryPointCalculator = new EuclideanTrajectoryPointCalculator();
      SO3TrajectoryPointCalculator orientationCalculator = new SO3TrajectoryPointCalculator();
      orientationCalculator.clear();

      for (int i = 0; i < numberOfTrajectoryPoints; i++)
      {
         converter.updateFullRobotModel(solution.getRobotConfigurations().get(i));

         Point3D desiredPosition = new Point3D();
         Quaternion desiredOrientation = new Quaternion();
         ReferenceFrame controlFrame = converter.getFullRobotModel().getHandControlFrame(robotSide);
         FramePose3D desiredHandPose = new FramePose3D(controlFrame);
         desiredHandPose.changeFrame(worldFrame);
         desiredHandPose.get(desiredPosition, desiredOrientation);

         desiredPositions[i] = new Point3D(desiredPosition);
         desiredOrientations[i] = new Quaternion(desiredOrientation);

         double time = solution.getKeyFrameTimes().get(i);
         euclideanTrajectoryPointCalculator.appendTrajectoryPoint(time, new Point3D(desiredPosition));
         orientationCalculator.appendTrajectoryPointOrientation(time, desiredOrientation);
      }

      orientationCalculator.compute();
      euclideanTrajectoryPointCalculator.computeTrajectoryPointVelocities(false);
      RecyclingArrayList<FrameEuclideanTrajectoryPoint> trajectoryPoints = euclideanTrajectoryPointCalculator.getTrajectoryPoints();

      for (int i = 0; i < numberOfTrajectoryPoints; i++)
      {
         Vector3D desiredLinearVelocity = new Vector3D();
         Vector3D desiredAngularVelocity = new Vector3D();

         trajectoryPoints.get(i).get(desiredPositions[i], desiredLinearVelocity);
         double time = trajectoryPoints.get(i).getTime();

         orientationCalculator.getTrajectoryPoints().get(i).getAngularVelocity(desiredAngularVelocity);

         trajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add()
                          .set(HumanoidMessageTools.createSE3TrajectoryPointMessage(time, desiredPositions[i], desiredOrientations[i], desiredLinearVelocity,
                                                                                    desiredAngularVelocity));
      }

      if (robotSide == RobotSide.LEFT)
         wholeBodyTrajectoryMessage.getLeftHandTrajectoryMessage().set(trajectoryMessage);
      else
         wholeBodyTrajectoryMessage.getRightHandTrajectoryMessage().set(trajectoryMessage);
   }

   private void computeChestTrajectoryMessage(KinematicsPlanningToolboxOutputStatus solution)
   {
      int numberOfTrajectoryPoints = solution.getRobotConfigurations().size();
      ChestTrajectoryMessage trajectoryMessage = new ChestTrajectoryMessage();

      SO3TrajectoryMessage so3Trajectory = trajectoryMessage.getSo3Trajectory();
      so3Trajectory.getFrameInformation().setTrajectoryReferenceFrameId(MessageTools.toFrameId(worldFrame));
      so3Trajectory.getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(worldFrame));

      Quaternion[] desiredOrientations = new Quaternion[numberOfTrajectoryPoints];

      SO3TrajectoryPointCalculator orientationCalculator = new SO3TrajectoryPointCalculator();
      orientationCalculator.clear();

      for (int i = 0; i < numberOfTrajectoryPoints; i++)
      {
         converter.updateFullRobotModel(solution.getRobotConfigurations().get(i));

         ReferenceFrame controlFrame = converter.getFullRobotModel().getChest().getBodyFixedFrame();
         FramePose3D desiredHandPose = new FramePose3D(controlFrame);
         desiredHandPose.changeFrame(worldFrame);
         Quaternion desiredOrientation = new Quaternion(desiredHandPose.getOrientation());

         desiredOrientations[i] = new Quaternion(desiredOrientation);

         double time = solution.getKeyFrameTimes().get(i);
         orientationCalculator.appendTrajectoryPointOrientation(time, desiredOrientation);
      }

      orientationCalculator.compute();

      for (int i = 0; i < numberOfTrajectoryPoints; i++)
      {
         Vector3D desiredAngularVelocity = new Vector3D();

         double time = solution.getKeyFrameTimes().get(i);

         orientationCalculator.getTrajectoryPoints().get(i).getAngularVelocity(desiredAngularVelocity);

         SO3TrajectoryPointMessage trajectoryPoint = so3Trajectory.getTaskspaceTrajectoryPoints().add();
         trajectoryPoint.setTime(time);
         trajectoryPoint.getOrientation().set(desiredOrientations[i]);
         trajectoryPoint.getAngularVelocity().set(desiredAngularVelocity);
      }

      wholeBodyTrajectoryMessage.getChestTrajectoryMessage().set(trajectoryMessage);
   }

   private void computePelvisTrajectoryMessage(KinematicsPlanningToolboxOutputStatus solution)
   {
      int numberOfTrajectoryPoints = solution.getRobotConfigurations().size();
      PelvisTrajectoryMessage trajectoryMessage = new PelvisTrajectoryMessage();

      trajectoryMessage.getSe3Trajectory().getFrameInformation().setTrajectoryReferenceFrameId(MessageTools.toFrameId(worldFrame));
      trajectoryMessage.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(worldFrame));

      Point3D[] desiredPositions = new Point3D[numberOfTrajectoryPoints];
      Quaternion[] desiredOrientations = new Quaternion[numberOfTrajectoryPoints];

      EuclideanTrajectoryPointCalculator euclideanTrajectoryPointCalculator = new EuclideanTrajectoryPointCalculator();
      SO3TrajectoryPointCalculator orientationCalculator = new SO3TrajectoryPointCalculator();
      orientationCalculator.clear();

      for (int i = 0; i < numberOfTrajectoryPoints; i++)
      {
         converter.updateFullRobotModel(solution.getRobotConfigurations().get(i));

         Point3D desiredPosition = new Point3D();
         Quaternion desiredOrientation = new Quaternion();
         ReferenceFrame controlFrame = converter.getFullRobotModel().getPelvis().getBodyFixedFrame();
         FramePose3D desiredHandPose = new FramePose3D(controlFrame);
         desiredHandPose.changeFrame(worldFrame);
         desiredHandPose.get(desiredPosition, desiredOrientation);

         desiredPositions[i] = new Point3D(desiredPosition);
         desiredOrientations[i] = new Quaternion(desiredOrientation);

         double time = solution.getKeyFrameTimes().get(i);
         euclideanTrajectoryPointCalculator.appendTrajectoryPoint(time, new Point3D(desiredPosition));
         orientationCalculator.appendTrajectoryPointOrientation(time, desiredOrientation);
      }

      orientationCalculator.compute();
      euclideanTrajectoryPointCalculator.computeTrajectoryPointVelocities(false);
      RecyclingArrayList<FrameEuclideanTrajectoryPoint> trajectoryPoints = euclideanTrajectoryPointCalculator.getTrajectoryPoints();

      for (int i = 0; i < numberOfTrajectoryPoints; i++)
      {
         Vector3D desiredLinearVelocity = new Vector3D();
         Vector3D desiredAngularVelocity = new Vector3D();

         trajectoryPoints.get(i).get(desiredPositions[i], desiredLinearVelocity);
         double time = trajectoryPoints.get(i).getTime();

         orientationCalculator.getTrajectoryPoints().get(i).getAngularVelocity(desiredAngularVelocity);

         trajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add()
                          .set(HumanoidMessageTools.createSE3TrajectoryPointMessage(time, desiredPositions[i], desiredOrientations[i], desiredLinearVelocity,
                                                                                    desiredAngularVelocity));
      }

      wholeBodyTrajectoryMessage.getPelvisTrajectoryMessage().set(trajectoryMessage);
   }

   private void computeNeckTrajectoryMessage(KinematicsPlanningToolboxOutputStatus solution)
   {

   }

   public void computeWholeBodyTrajectoryMessage(KinematicsPlanningToolboxOutputStatus solution)
   {
      computeHandTrajectoryMessages(solution);
      computeChestTrajectoryMessage(solution);
      computePelvisTrajectoryMessage(solution);
      computeNeckTrajectoryMessage(solution);
   }

   public void setMessageToCreate(WholeBodyTrajectoryMessage wholebodyTrajectoryMessage)
   {
      this.wholeBodyTrajectoryMessage = wholebodyTrajectoryMessage;
   }

   public KinematicsToolboxOutputStatus getRobotConfiguration(KinematicsPlanningToolboxOutputStatus solution, double time)
   {
      double minimumGap = Double.MAX_VALUE;
      int nodeIndex = 0;
      for (int i = 0; i < solution.getKeyFrameTimes().size(); i++)
      {
         double gap = Math.abs(time - solution.getKeyFrameTimes().get(i));
         if (gap < minimumGap)
         {
            minimumGap = gap;
            nodeIndex = i;
         }
      }

      return solution.getRobotConfigurations().get(nodeIndex);
   }
}
