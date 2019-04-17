package us.ihmc.humanoidRobotics.communication.packets;

import controller_msgs.msg.dds.ChestTrajectoryMessage;
import controller_msgs.msg.dds.HandTrajectoryMessage;
import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import controller_msgs.msg.dds.PelvisTrajectoryMessage;
import controller_msgs.msg.dds.SO3TrajectoryMessage;
import controller_msgs.msg.dds.SO3TrajectoryPointMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryToolboxOutputStatus;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.math.trajectories.generators.EuclideanTrajectoryPointCalculator;
import us.ihmc.robotics.math.trajectories.generators.SO3TrajectoryPointCalculator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.lists.FrameEuclideanTrajectoryPointList;
import us.ihmc.robotics.robotSide.RobotSide;

public class WholeBodyTrajectoryToolboxOutputConverter
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage;

   private KinematicsToolboxOutputConverter converter;

   private double firstTrajectoryPointTime = 0.0;

   public WholeBodyTrajectoryToolboxOutputConverter(FullHumanoidRobotModelFactory fullRobotModelFactory)
   {
      converter = new KinematicsToolboxOutputConverter(fullRobotModelFactory);
   }

   public void setMessageToCreate(WholeBodyTrajectoryMessage wholebodyTrajectoryMessage)
   {
      this.wholeBodyTrajectoryMessage = wholebodyTrajectoryMessage;
   }

   public double getFirstTrajectoryPointTime()
   {
      return firstTrajectoryPointTime;
   }

   public void setFirstTrajectoryPointTime(double firstTrajectoryPointTime)
   {
      this.firstTrajectoryPointTime = firstTrajectoryPointTime;
   }

   public void updateFullRobotModel(WholeBodyTrajectoryToolboxOutputStatus solution)
   {
      for (RobotSide robotSide : RobotSide.values)
         computHandTrajectoryMessage(solution, robotSide);
      computeChestTrajectoryMessage(solution);
      computePelvisTrajectoryMessage(solution);
   }

   private void computHandTrajectoryMessage(WholeBodyTrajectoryToolboxOutputStatus solution, RobotSide robotSide)
   {
      int numberOfTrajectoryPoints = solution.getRobotConfigurations().size();
      HandTrajectoryMessage trajectoryMessage = new HandTrajectoryMessage();
      trajectoryMessage.setRobotSide(robotSide.toByte());

      trajectoryMessage.getSe3Trajectory().getFrameInformation().setTrajectoryReferenceFrameId(MessageTools.toFrameId(worldFrame));
      trajectoryMessage.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(worldFrame));

      // message params.
      ReferenceFrame trajectoryFrame = worldFrame;
      Point3D[] desiredPositions = new Point3D[numberOfTrajectoryPoints];
      Quaternion[] desiredOrientations = new Quaternion[numberOfTrajectoryPoints];

      EuclideanTrajectoryPointCalculator euclideanTrajectoryPointCalculator = new EuclideanTrajectoryPointCalculator();
      SO3TrajectoryPointCalculator orientationCalculator = new SO3TrajectoryPointCalculator();
      orientationCalculator.clear();

      // get points.
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

         double time = firstTrajectoryPointTime + solution.getTrajectoryTimes().get(i);
         euclideanTrajectoryPointCalculator.appendTrajectoryPoint(time, new Point3D(desiredPosition));
         orientationCalculator.appendTrajectoryPoint(time, desiredOrientation);
      }

      // get velocities.
      orientationCalculator.compute();
      euclideanTrajectoryPointCalculator.compute(solution.getTrajectoryTimes().get(numberOfTrajectoryPoints - 1));
      FrameEuclideanTrajectoryPointList trajectoryPoints = euclideanTrajectoryPointCalculator.getTrajectoryPoints();

      // set trajectory points.
      for (int i = 0; i < numberOfTrajectoryPoints; i++)
      {
         Vector3D desiredLinearVelocity = new Vector3D();
         Vector3D desiredAngularVelocity = new Vector3D();

         trajectoryPoints.getTrajectoryPoint(i).get(desiredPositions[i], desiredLinearVelocity);
         double time = trajectoryPoints.getTrajectoryPoint(i).getTime();

         orientationCalculator.getTrajectoryPoint(i).getAngularVelocity(desiredAngularVelocity);

         trajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add()
                          .set(HumanoidMessageTools.createSE3TrajectoryPointMessage(time, desiredPositions[i], desiredOrientations[i], desiredLinearVelocity,
                                                                                    desiredAngularVelocity));
      }

      if (robotSide == RobotSide.LEFT)
         wholeBodyTrajectoryMessage.getLeftHandTrajectoryMessage().set(trajectoryMessage);
      else
         wholeBodyTrajectoryMessage.getRightHandTrajectoryMessage().set(trajectoryMessage);
   }

   private void computeChestTrajectoryMessage(WholeBodyTrajectoryToolboxOutputStatus solution)
   {
      int numberOfTrajectoryPoints = solution.getRobotConfigurations().size();
      ChestTrajectoryMessage trajectoryMessage = new ChestTrajectoryMessage();

      SO3TrajectoryMessage so3Trajectory = trajectoryMessage.getSo3Trajectory();
      so3Trajectory.getFrameInformation().setTrajectoryReferenceFrameId(MessageTools.toFrameId(worldFrame));
      so3Trajectory.getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(worldFrame));

      // message params.
      ReferenceFrame trajectoryFrame = worldFrame;
      Quaternion[] desiredOrientations = new Quaternion[numberOfTrajectoryPoints];

      SO3TrajectoryPointCalculator orientationCalculator = new SO3TrajectoryPointCalculator();
      orientationCalculator.clear();

      // get points.
      for (int i = 0; i < numberOfTrajectoryPoints; i++)
      {
         converter.updateFullRobotModel(solution.getRobotConfigurations().get(i));

         ReferenceFrame controlFrame = converter.getFullRobotModel().getChest().getBodyFixedFrame();
         FramePose3D desiredHandPose = new FramePose3D(controlFrame);
         desiredHandPose.changeFrame(worldFrame);
         Quaternion desiredOrientation = new Quaternion(desiredHandPose.getOrientation());

         desiredOrientations[i] = new Quaternion(desiredOrientation);

         double time = firstTrajectoryPointTime + solution.getTrajectoryTimes().get(i);
         orientationCalculator.appendTrajectoryPoint(time, desiredOrientation);
      }

      // get velocities.
      orientationCalculator.compute();

      // set trajectory points.
      for (int i = 0; i < numberOfTrajectoryPoints; i++)
      {
         Vector3D desiredAngularVelocity = new Vector3D();

         double time = firstTrajectoryPointTime + solution.getTrajectoryTimes().get(i);

         orientationCalculator.getTrajectoryPoint(i).getAngularVelocity(desiredAngularVelocity);

         SO3TrajectoryPointMessage trajectoryPoint = so3Trajectory.getTaskspaceTrajectoryPoints().add();
         trajectoryPoint.setTime(time);
         trajectoryPoint.getOrientation().set(desiredOrientations[i]);
         trajectoryPoint.getAngularVelocity().set(desiredAngularVelocity);
      }

      wholeBodyTrajectoryMessage.getChestTrajectoryMessage().set(trajectoryMessage);
   }

   private void computePelvisTrajectoryMessage(WholeBodyTrajectoryToolboxOutputStatus solution)
   {
      int numberOfTrajectoryPoints = solution.getRobotConfigurations().size();
      PelvisTrajectoryMessage trajectoryMessage = new PelvisTrajectoryMessage();

      trajectoryMessage.getSe3Trajectory().getFrameInformation().setTrajectoryReferenceFrameId(MessageTools.toFrameId(worldFrame));
      trajectoryMessage.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(worldFrame));

      // message params.
      Point3D[] desiredPositions = new Point3D[numberOfTrajectoryPoints];
      Quaternion[] desiredOrientations = new Quaternion[numberOfTrajectoryPoints];

      EuclideanTrajectoryPointCalculator euclideanTrajectoryPointCalculator = new EuclideanTrajectoryPointCalculator();
      SO3TrajectoryPointCalculator orientationCalculator = new SO3TrajectoryPointCalculator();
      orientationCalculator.clear();

      // get points.
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

         double time = solution.getTrajectoryTimes().get(i);
         euclideanTrajectoryPointCalculator.appendTrajectoryPoint(time, new Point3D(desiredPosition));
         orientationCalculator.appendTrajectoryPoint(time, desiredOrientation);
      }

      // get velocities.
      orientationCalculator.compute();
      euclideanTrajectoryPointCalculator.compute(solution.getTrajectoryTimes().get(numberOfTrajectoryPoints - 1));
      FrameEuclideanTrajectoryPointList trajectoryPoints = euclideanTrajectoryPointCalculator.getTrajectoryPoints();
      trajectoryPoints.addTimeOffset(firstTrajectoryPointTime);

      // set trajectory points.
      for (int i = 0; i < numberOfTrajectoryPoints; i++)
      {
         Vector3D desiredLinearVelocity = new Vector3D();
         Vector3D desiredAngularVelocity = new Vector3D();

         trajectoryPoints.getTrajectoryPoint(i).get(desiredPositions[i], desiredLinearVelocity);
         double time = trajectoryPoints.getTrajectoryPoint(i).getTime();

         orientationCalculator.getTrajectoryPoint(i).getAngularVelocity(desiredAngularVelocity);

         trajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add()
                          .set(HumanoidMessageTools.createSE3TrajectoryPointMessage(time, desiredPositions[i], desiredOrientations[i], desiredLinearVelocity,
                                                                                    desiredAngularVelocity));
      }

      wholeBodyTrajectoryMessage.getPelvisTrajectoryMessage().set(trajectoryMessage);
   }

   private void computeHeadTrajectoryMessage(WholeBodyTrajectoryToolboxOutputStatus solution)
   {

   }

   public KinematicsToolboxOutputStatus getRobotConfiguration(WholeBodyTrajectoryToolboxOutputStatus solution, double time)
   {
      double minimumGap = Double.MAX_VALUE;
      int nodeIndex = 0;
      for (int i = 0; i < solution.getTrajectoryTimes().size(); i++)
      {
         double gap = Math.abs(time - solution.getTrajectoryTimes().get(i));
         if (gap < minimumGap)
         {
            minimumGap = gap;
            nodeIndex = i;
         }
      }

      return solution.getRobotConfigurations().get(nodeIndex);
   }
}
