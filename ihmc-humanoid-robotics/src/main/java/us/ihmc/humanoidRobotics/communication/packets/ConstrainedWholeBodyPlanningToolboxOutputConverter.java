package us.ihmc.humanoidRobotics.communication.packets;

import java.util.ArrayList;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.utils.NameBasedHashCodeTools;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.CTTreeTools;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConfigurationSpace;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConstrainedEndEffectorTrajectory;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConstrainedWholeBodyPlanningToolboxOutputStatus;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.NodeDataPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.waypoints.EuclideanTrajectoryPointCalculator;
import us.ihmc.robotics.math.trajectories.waypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.SO3TrajectoryPointCalculator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;

public class ConstrainedWholeBodyPlanningToolboxOutputConverter
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final FullHumanoidRobotModel fullRobotModelToUseForConversion;
   private final HumanoidReferenceFrames referenceFrames;
   private final FloatingInverseDynamicsJoint rootJoint;
   private final OneDoFJoint[] oneDoFJoints;
   private final int jointsHashCode;

   private WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage;

   private double trajectoryTime = 0.0;

   private double firstTrajectoryPointTime = 6.0;

   private ConstrainedEndEffectorTrajectory constrainedEndEffectorTrajectory;

   public ConstrainedWholeBodyPlanningToolboxOutputConverter(FullHumanoidRobotModelFactory fullRobotModelFactory)
   {
      this.fullRobotModelToUseForConversion = fullRobotModelFactory.createFullRobotModel();
      rootJoint = fullRobotModelToUseForConversion.getRootJoint();
      oneDoFJoints = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModelToUseForConversion);
      jointsHashCode = (int) NameBasedHashCodeTools.computeArrayHashCode(oneDoFJoints);
      referenceFrames = new HumanoidReferenceFrames(fullRobotModelToUseForConversion);
   }

   public WholeBodyTrajectoryMessage getWholebodyTrajectoryMessage()
   {
      return wholeBodyTrajectoryMessage;
   }

   public void setMessageToCreate(WholeBodyTrajectoryMessage wholebodyTrajectoryMessage)
   {
      this.wholeBodyTrajectoryMessage = wholebodyTrajectoryMessage;
   }

   public void setConstrainedEndEffectorTrajectory(ConstrainedEndEffectorTrajectory constrainedEndEffectorTrajectory)
   {
      this.constrainedEndEffectorTrajectory = constrainedEndEffectorTrajectory;
   }

   public double getTrajectoryTime()
   {
      return trajectoryTime;
   }

   public void setTrajectoryTime(double trajectoryTime)
   {
      this.trajectoryTime = trajectoryTime;
   }

   public double getFirstTrajectoryPointTime()
   {
      return firstTrajectoryPointTime;
   }

   public void setFirstTrajectoryPointTime(double firstTrajectoryPointTime)
   {
      this.firstTrajectoryPointTime = firstTrajectoryPointTime;
   }

   private SideDependentList<HandTrajectoryMessage> handTrajectoryMessages = new SideDependentList<>();
   private ChestTrajectoryMessage chestTrajectoryMessage;
   private PelvisTrajectoryMessage pelvisTrajectoryMessage;

   private void updateHandTrajectoryMessages()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         int numberOfTrajectoryPoints = outputPath.size();

         PrintTools.info("" + numberOfTrajectoryPoints);

         HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage(robotSide, numberOfTrajectoryPoints);

         handTrajectoryMessage.getFrameInformation().setTrajectoryReferenceFrame(worldFrame);
         handTrajectoryMessage.getFrameInformation().setDataReferenceFrame(worldFrame);

         EuclideanTrajectoryPointCalculator euclideanTrajectoryPointCalculator = new EuclideanTrajectoryPointCalculator();

         for (int i = 0; i < numberOfTrajectoryPoints; i++)
         {
            // CTTaskNode trajectoryNode = path.get(i);
            NodeDataPacket trajectoryNode = outputPath.get(i);

            ConfigurationSpace configurationSpace = CTTreeTools.getConfigurationSpace(trajectoryNode, robotSide);

            Pose3D desiredPose = constrainedEndEffectorTrajectory.getEndEffectorPose(trajectoryNode.getQ(0), robotSide, configurationSpace);

            euclideanTrajectoryPointCalculator.appendTrajectoryPoint(new Point3D(desiredPose.getPosition()));
         }

         double[] trajectoryTimes = new double[numberOfTrajectoryPoints];

         for (int i = 0; i < numberOfTrajectoryPoints; i++)
            trajectoryTimes[i] = outputPath.get(i).getQ(0);

         euclideanTrajectoryPointCalculator.computeTrajectoryPointTimes(firstTrajectoryPointTime, trajectoryTimes);

         euclideanTrajectoryPointCalculator.computeTrajectoryPointVelocities(false);

         RecyclingArrayList<FrameEuclideanTrajectoryPoint> trajectoryPoints = euclideanTrajectoryPointCalculator.getTrajectoryPoints();

         for (int i = 0; i < numberOfTrajectoryPoints; i++)
         {
            // CTTaskNode trajectoryNode = path.get(i);
            NodeDataPacket trajectoryNode = outputPath.get(i);

            ConfigurationSpace configurationSpace = CTTreeTools.getConfigurationSpace(trajectoryNode, robotSide);

            Pose3D desiredPose = constrainedEndEffectorTrajectory.getEndEffectorPose(trajectoryNode.getQ(0), robotSide, configurationSpace);

            Point3D desiredPosition = new Point3D(desiredPose.getPosition());
            Vector3D desiredLinearVelocity = new Vector3D();
            Quaternion desiredOrientation = new Quaternion(desiredPose.getOrientation());

            if (robotSide == RobotSide.LEFT)
               desiredOrientation.appendRollRotation(Math.PI * 0.5);
            else
               desiredOrientation.appendRollRotation(-Math.PI * 0.5);

            Vector3D desiredAngularVelocity = new Vector3D();

            double time = trajectoryPoints.get(i).get(desiredPosition, desiredLinearVelocity);

            //            PrintTools.info(""+i+" "+time +" " + desiredLinearVelocity+" ");

            handTrajectoryMessage.setTrajectoryPoint(i, time, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity, worldFrame);
         }

         handTrajectoryMessages.put(robotSide, handTrajectoryMessage);

      }
   }

   private void updateChestTrajectoryMessage()
   {
      int numberOfTrajectoryPoints = outputPath.size();

      PrintTools.info("" + numberOfTrajectoryPoints);

      chestTrajectoryMessage = new ChestTrajectoryMessage(numberOfTrajectoryPoints);
      chestTrajectoryMessage.getFrameInformation().setTrajectoryReferenceFrame(worldFrame);
      chestTrajectoryMessage.getFrameInformation().setDataReferenceFrame(worldFrame);

      PrintTools.info("" + chestTrajectoryMessage.getNumberOfTrajectoryPoints());

      SO3TrajectoryPointCalculator orientationCalculator = new SO3TrajectoryPointCalculator();
      orientationCalculator.clear();
      orientationCalculator.setFirstTrajectoryPointTime(firstTrajectoryPointTime);

      for (int i = 0; i < numberOfTrajectoryPoints; i++)
      {
      // CTTaskNode trajectoryNode = path.get(i);
         NodeDataPacket trajectoryNode = outputPath.get(i);

         double time = firstTrajectoryPointTime + trajectoryNode.getQ(0);

         Quaternion desiredOrientation = new Quaternion();

         desiredOrientation.appendYawRotation(trajectoryNode.getQ(2));
         desiredOrientation.appendPitchRotation(trajectoryNode.getQ(3));
         desiredOrientation.appendRollRotation(trajectoryNode.getQ(4));

         orientationCalculator.appendTrajectoryPointOrientation(time, desiredOrientation);
      }

      orientationCalculator.compute();

      for (int i = 0; i < numberOfTrajectoryPoints; i++)
      {
      // CTTaskNode trajectoryNode = path.get(i);
         NodeDataPacket trajectoryNode = outputPath.get(i);

         double time = firstTrajectoryPointTime + trajectoryNode.getQ(0);

         Quaternion desiredOrientation = new Quaternion();

         desiredOrientation.appendYawRotation(trajectoryNode.getQ(2));
         desiredOrientation.appendPitchRotation(trajectoryNode.getQ(3));
         desiredOrientation.appendRollRotation(trajectoryNode.getQ(4));

         Vector3D desiredAngularVelocity = orientationCalculator.getTrajectoryPointsAngularVelocity().get(i);

         PrintTools.info("" + i + " " + time);

         chestTrajectoryMessage.setTrajectoryPoint(i, time, desiredOrientation, desiredAngularVelocity, worldFrame);
      }
   }

   private void updatePelvisTrajectoryMessage()
   {
      int numberOfTrajectoryPoints = outputPath.size();

      PrintTools.info("" + numberOfTrajectoryPoints);

      pelvisTrajectoryMessage = new PelvisTrajectoryMessage(numberOfTrajectoryPoints);
      pelvisTrajectoryMessage.getFrameInformation().setTrajectoryReferenceFrame(worldFrame);
      pelvisTrajectoryMessage.getFrameInformation().setDataReferenceFrame(worldFrame);

      SelectionMatrix6D selectionMatrix6D = new SelectionMatrix6D();
      selectionMatrix6D.clearAngularSelection();
      selectionMatrix6D.clearLinearSelection();
      selectionMatrix6D.selectLinearZ(true);
      pelvisTrajectoryMessage.setSelectionMatrix(selectionMatrix6D);

      EuclideanTrajectoryPointCalculator euclideanTrajectoryPointCalculator = new EuclideanTrajectoryPointCalculator();

      for (int i = 0; i < numberOfTrajectoryPoints; i++)
      {
         // CTTaskNode trajectoryNode = path.get(i);
         NodeDataPacket trajectoryNode = outputPath.get(i);

         Point3D pelvisPosition = new Point3D(0, 0, trajectoryNode.getQ(1));

         euclideanTrajectoryPointCalculator.appendTrajectoryPoint(pelvisPosition);
      }

      double[] trajectoryTimes = new double[numberOfTrajectoryPoints];

      for (int i = 0; i < numberOfTrajectoryPoints; i++)
         trajectoryTimes[i] = outputPath.get(i).getQ(0);

      euclideanTrajectoryPointCalculator.computeTrajectoryPointTimes(firstTrajectoryPointTime, trajectoryTimes);
      euclideanTrajectoryPointCalculator.computeTrajectoryPointVelocities(false);

      RecyclingArrayList<FrameEuclideanTrajectoryPoint> trajectoryPoints = euclideanTrajectoryPointCalculator.getTrajectoryPoints();

      for (int i = 0; i < numberOfTrajectoryPoints; i++)
      {
         // CTTaskNode trajectoryNode = path.get(i);
         NodeDataPacket trajectoryNode = outputPath.get(i);

         Point3D pelvisPosition = new Point3D(0, 0, trajectoryNode.getQ(1));

         Quaternion orientation = new Quaternion();
         Vector3D angularVelocity = new Vector3D();

         Vector3D pelvisLinearVelocity = new Vector3D();

         double time = trajectoryPoints.get(i).get(pelvisPosition, pelvisLinearVelocity);

         pelvisTrajectoryMessage.setTrajectoryPoint(i, time, pelvisPosition, orientation, pelvisLinearVelocity, angularVelocity, worldFrame);

         //         PrintTools.info(""+ i+" "+pelvisPosition +" "+pelvisLinearVelocity);
      }
   }

   public ArrayList<NodeDataPacket> outputPath;

   public void updateFullRobotModel(ConstrainedWholeBodyPlanningToolboxOutputStatus solution)
   {
      //      if (jointsHashCode != solution.jointNameHash)
      //         throw new RuntimeException("Hashes are different.");
      //
      //      for (int i = 0; i < oneDoFJoints.length; i++)
      //      {
      //         float q = solution.getJointAngles()[i];
      //         OneDoFJoint joint = oneDoFJoints[i];
      //         joint.setQ(q);
      //      }
      //      
      //      Vector3D32 translation = solution.getPelvisTranslation();
      //      rootJoint.setPosition(translation.getX(), translation.getY(), translation.getZ());
      //      Quaternion32 orientation = solution.getPelvisOrientation();
      //      rootJoint.setRotation(orientation.getX(), orientation.getY(), orientation.getZ(), orientation.getS());
      //      fullRobotModelToUseForConversion.updateFrames();

      outputPath = solution.outputPath;

      wholeBodyTrajectoryMessage.clear();

      updateHandTrajectoryMessages();
      updateChestTrajectoryMessage();
      updatePelvisTrajectoryMessage();

      for (RobotSide robotSide : RobotSide.values)
         wholeBodyTrajectoryMessage.setHandTrajectoryMessage(handTrajectoryMessages.get(robotSide));

      wholeBodyTrajectoryMessage.setChestTrajectoryMessage(chestTrajectoryMessage);
      wholeBodyTrajectoryMessage.setPelvisTrajectoryMessage(pelvisTrajectoryMessage);
   }
}
