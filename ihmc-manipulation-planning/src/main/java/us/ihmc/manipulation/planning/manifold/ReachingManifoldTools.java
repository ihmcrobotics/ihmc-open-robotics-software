package us.ihmc.manipulation.planning.manifold;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.MismatchedSizeException;

import controller_msgs.msg.dds.ReachingManifoldMessage;
import controller_msgs.msg.dds.RigidBodyExplorationConfigurationMessage;
import controller_msgs.msg.dds.WaypointBasedTrajectoryMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryToolboxMessage;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.rotationConversion.AxisAngleConversion;
import us.ihmc.euclid.rotationConversion.RotationMatrixConversion;
import us.ihmc.euclid.shape.Cylinder3D;
import us.ihmc.euclid.shape.Sphere3D;
import us.ihmc.euclid.shape.Torus3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.SegmentedLine3DMeshDataGenerator;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessageTools.FunctionTrajectory;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.ReachingManifoldCommand;
import us.ihmc.manipulation.planning.exploringSpatial.ExploringRigidBodyTools;
import us.ihmc.manipulation.planning.exploringSpatial.TrajectoryLibraryForDRC;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.numericalMethods.GradientDescentModule;
import us.ihmc.robotics.numericalMethods.SingleQueryFunction;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;

/**
 * Essential tools for manifolds.
 * Create list of ReachingManifoldMessages.
 * Calculate closest point on list of manifolds.
 * Calculate distance between list of manifolds and given RigidBodyTransform.
 * Get static graphics for manifolds.
 */
public class ReachingManifoldTools
{
   private static double extrapolateRatio = 1.5;

   public static Graphics3DObject createManifoldMessageStaticGraphic(ReachingManifoldMessage reachingManifoldMessage, double radius,
                                                                     int resolutionForSingleSpace)
   {
      Pose3D originPose = new Pose3D(reachingManifoldMessage.getManifoldOriginPosition(), reachingManifoldMessage.getManifoldOriginOrientation());

      int numberOfPoints = (int) Math.pow(resolutionForSingleSpace, reachingManifoldMessage.getManifoldConfigurationSpaceNames().size());

      SegmentedLine3DMeshDataGenerator segmentedLine3DMeshGenerator = new SegmentedLine3DMeshDataGenerator(numberOfPoints, resolutionForSingleSpace, radius);

      Point3D[] points = new Point3D[numberOfPoints];
      for (int i = 0; i < numberOfPoints; i++)
      {
         Pose3D pose = new Pose3D(originPose);
         double[] configurationValues = new double[reachingManifoldMessage.getManifoldConfigurationSpaceNames().size()];
         int[] configurationIndex = new int[reachingManifoldMessage.getManifoldConfigurationSpaceNames().size()];

         int tempIndex = i;
         for (int j = reachingManifoldMessage.getManifoldConfigurationSpaceNames().size(); j > 0; j--)
         {
            configurationIndex[j - 1] = (int) (tempIndex / Math.pow(resolutionForSingleSpace, j - 1));
            tempIndex = (int) (tempIndex % Math.pow(resolutionForSingleSpace, j - 1));
         }

         for (int j = 0; j < reachingManifoldMessage.getManifoldConfigurationSpaceNames().size(); j++)
         {
            configurationValues[j] = (reachingManifoldMessage.getManifoldUpperLimits().get(j) - reachingManifoldMessage.getManifoldLowerLimits().get(j))
                  / (resolutionForSingleSpace - 1) * configurationIndex[j] + reachingManifoldMessage.getManifoldLowerLimits().get(j);

            ConfigurationSpaceName configurationSpaceName = ConfigurationSpaceName.fromByte(reachingManifoldMessage.getManifoldConfigurationSpaceNames()
                                                                                                                   .get(j));

            switch (configurationSpaceName)
            {
            case X:
               pose.appendTranslation(configurationValues[j], 0.0, 0.0);
               break;
            case Y:
               pose.appendTranslation(0.0, configurationValues[j], 0.0);
               break;
            case Z:
               pose.appendTranslation(0.0, 0.0, configurationValues[j]);
               break;
            case ROLL:
               pose.appendRollRotation(configurationValues[j]);
               break;
            case PITCH:
               pose.appendPitchRotation(configurationValues[j]);
               break;
            case YAW:
               pose.appendYawRotation(configurationValues[j]);
               break;
            default:
               break;
            }
         }

         points[i] = new Point3D(pose.getPosition());
      }

      if (points.length > 1)
         segmentedLine3DMeshGenerator.compute(points);

      Graphics3DObject graphics = new Graphics3DObject();
      for (MeshDataHolder mesh : segmentedLine3DMeshGenerator.getMeshDataHolders())
      {
         graphics.addMeshData(mesh, YoAppearance.AliceBlue());
      }

      return graphics;
   }

   public static WholeBodyTrajectoryToolboxMessage createReachingWholeBodyTrajectoryToolboxMessage(FullHumanoidRobotModel fullRobotModel, RobotSide robotSide,
                                                                                                   List<ReachingManifoldMessage> reachingManifoldMessages,
                                                                                                   double desiredTrajectoryTime)
   {
      RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
      double extrapolateRatio = ReachingManifoldTools.extrapolateRatio;
      double trajectoryTimeBeforeExtrapolated = desiredTrajectoryTime;
      double trajectoryTime = trajectoryTimeBeforeExtrapolated * extrapolateRatio;

      // configuration message
      WholeBodyTrajectoryToolboxConfigurationMessage configuration = new WholeBodyTrajectoryToolboxConfigurationMessage();
      configuration.getInitialConfiguration().set(HumanoidMessageTools.createKinematicsToolboxOutputStatus(fullRobotModel));
      configuration.setMaximumExpansionSize(500);

      // messages
      List<WaypointBasedTrajectoryMessage> handTrajectories = new ArrayList<>();
      List<RigidBodyExplorationConfigurationMessage> rigidBodyConfigurations = new ArrayList<>();
      List<ReachingManifoldMessage> reachingManifolds = new ArrayList<>();

      // TODO : increase number of resolution.
      double timeResolution = trajectoryTime / 20.0;

      MovingReferenceFrame handControlFrame = fullRobotModel.getHandControlFrame(robotSide);
      RigidBodyTransform handTransform = handControlFrame.getTransformToWorldFrame();

      RigidBodyTransform closestPointOnManifold = new RigidBodyTransform();
      RigidBodyTransform endTransformOnTrajectory = new RigidBodyTransform();

      List<ReachingManifoldCommand> manifolds = new ArrayList<>();
      for (int i = 0; i < reachingManifoldMessages.size(); i++)
      {
         ReachingManifoldCommand manifold = new ReachingManifoldCommand();
         manifold.setFromMessage(reachingManifoldMessages.get(i));
         manifolds.add(manifold);
      }
      ReachingManifoldTools.packClosestRigidBodyTransformOnManifold(manifolds, handTransform, closestPointOnManifold, 1.0, 0.1);
      ReachingManifoldTools.packExtrapolatedTransform(handTransform, closestPointOnManifold, extrapolateRatio, endTransformOnTrajectory);
      reachingManifolds.addAll(reachingManifoldMessages);

      FunctionTrajectory handFunction = time -> TrajectoryLibraryForDRC.computeLinearTrajectory(time, trajectoryTime, handTransform, endTransformOnTrajectory);

      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
      selectionMatrix.resetSelection();
      WaypointBasedTrajectoryMessage trajectory = WholeBodyTrajectoryToolboxMessageTools.createTrajectoryMessage(hand, 0.0, trajectoryTime, timeResolution,
                                                                                                                 handFunction, selectionMatrix);

      RigidBodyTransform handControlFrameTransformToBodyFixedFrame = new RigidBodyTransform();
      handControlFrame.getTransformToDesiredFrame(handControlFrameTransformToBodyFixedFrame, hand.getBodyFixedFrame());
      trajectory.getControlFramePositionInEndEffector().set(handControlFrameTransformToBodyFixedFrame.getTranslationVector());
      trajectory.getControlFrameOrientationInEndEffector().set(handControlFrameTransformToBodyFixedFrame.getRotationMatrix());

      handTrajectories.add(trajectory);

      ConfigurationSpaceName[] spaces = {ConfigurationSpaceName.X, ConfigurationSpaceName.Y, ConfigurationSpaceName.Z, ConfigurationSpaceName.SO3};
      rigidBodyConfigurations.add(HumanoidMessageTools.createRigidBodyExplorationConfigurationMessage(hand, spaces));

      WholeBodyTrajectoryToolboxMessage message = HumanoidMessageTools.createWholeBodyTrajectoryToolboxMessage(configuration, handTrajectories,
                                                                                                               reachingManifolds, rigidBodyConfigurations);

      return message;
   }

   public static double packClosestRigidBodyTransformOnManifold(List<ReachingManifoldCommand> manifolds, Pose3D pose,
                                                                RigidBodyTransform rigidBodyTransformToPack, double positionWeight, double orientationWeight)
   {
      return packClosestRigidBodyTransformOnManifold(manifolds, new RigidBodyTransform(pose.getOrientation(), pose.getPosition()), rigidBodyTransformToPack,
                                                     positionWeight, orientationWeight);
   }

   public static double packClosestRigidBodyTransformOnManifold(List<ReachingManifoldCommand> manifolds, RigidBodyTransform rigidBodyTransform,
                                                                RigidBodyTransform rigidBodyTransformToPack, double positionWeight, double orientationWeight)
   {
      double distance = Double.MAX_VALUE;
      int indexToPack = 0;
      for (int i = 0; i < manifolds.size(); i++)
      {
         double d = packClosestRigidBodyTransformOnManifold(manifolds.get(i), rigidBodyTransform, rigidBodyTransformToPack, positionWeight, orientationWeight);
         if (d < distance)
         {
            distance = d;
            indexToPack = i;
         }
      }

      return packClosestRigidBodyTransformOnManifold(manifolds.get(indexToPack), rigidBodyTransform, rigidBodyTransformToPack, positionWeight,
                                                     orientationWeight);
   }

   public static List<ReachingManifoldMessage> createSphereManifoldMessagesForValkyrie(RobotSide robotSide, RigidBodyBasics hand, Sphere3D sphere3D)
   {
      return createSphereManifoldMessagesForValkyrie(robotSide, hand, sphere3D.getPosition(), sphere3D.getRadius());
   }

   public static List<ReachingManifoldMessage> createCylinderManifoldMessagesForValkyrie(RobotSide robotSide, RigidBodyBasics hand, Cylinder3D cylinder3D)
   {
      return createCylinderManifoldMessagesForValkyrie(robotSide, hand, cylinder3D.getPosition(), cylinder3D.getOrientation(), cylinder3D.getRadius(),
                                                       cylinder3D.getHeight());
   }

   public static List<ReachingManifoldMessage> createTorusManifoldMessagesForValkyrie(RobotSide robotSide, RigidBodyBasics hand, Torus3D torus3D)
   {
      return createTorusManifoldMessagesForValkyrie(robotSide, hand, torus3D.getPosition(), torus3D.getOrientation(), torus3D.getRadius(),
                                                    torus3D.getTubeRadius());
   }

   public static List<ReachingManifoldMessage> createSphereManifoldMessagesForValkyrie(RobotSide robotSide, RigidBodyBasics hand,
                                                                                       Tuple3DReadOnly manifoldOriginPosition, double radius)
   {
      List<ReachingManifoldMessage> messages = new ArrayList<>();
      ReachingManifoldMessage message = HumanoidMessageTools.createReachingManifoldMessage(hand);

      ConfigurationSpaceName[] spaces = {ConfigurationSpaceName.YAW, ConfigurationSpaceName.ROLL, ConfigurationSpaceName.Y, ConfigurationSpaceName.PITCH};
      double[] lowerLimits = new double[] {-Math.PI, -0.5 * Math.PI, robotSide.negateIfRightSide(radius), -Math.PI};
      double[] upperLimits = new double[] {Math.PI, 0.5 * Math.PI, robotSide.negateIfRightSide(radius), Math.PI};

      message.getManifoldOriginPosition().set(manifoldOriginPosition);

      HumanoidMessageTools.packManifold(ConfigurationSpaceName.toBytes(spaces), lowerLimits, upperLimits, message);
      messages.add(message);

      return messages;
   }

   public static List<ReachingManifoldMessage> createCylinderManifoldMessagesForValkyrie(RobotSide robotSide, RigidBodyBasics hand,
                                                                                         Tuple3DReadOnly manifoldOriginPosition,
                                                                                         RotationMatrixReadOnly manifoldOriginOrientation, double radius,
                                                                                         double height)
   {
      double topAreaReductionRatio = 1.0;
      double thicknessForViz = 0.01;

      List<ReachingManifoldMessage> messages = new ArrayList<>();
      ReachingManifoldMessage sideMessage = HumanoidMessageTools.createReachingManifoldMessage(hand);
      ReachingManifoldMessage topMessage = HumanoidMessageTools.createReachingManifoldMessage(hand);
      ReachingManifoldMessage bottomMessage = HumanoidMessageTools.createReachingManifoldMessage(hand);

      ConfigurationSpaceName[] sideSpaces = {ConfigurationSpaceName.YAW, ConfigurationSpaceName.Z, ConfigurationSpaceName.Y};
      double[] sideLowerLimits = new double[] {-Math.PI, 0.0, robotSide.negateIfRightSide(radius)};
      double[] sideUpperLimits = new double[] {Math.PI, height, robotSide.negateIfRightSide(radius)};

      sideMessage.getManifoldOriginPosition().set(manifoldOriginPosition);
      sideMessage.getManifoldOriginOrientation().set(manifoldOriginOrientation);

      HumanoidMessageTools.packManifold(ConfigurationSpaceName.toBytes(sideSpaces), sideLowerLimits, sideUpperLimits, sideMessage);

      ConfigurationSpaceName[] topSpaces = {ConfigurationSpaceName.Y, ConfigurationSpaceName.PITCH, ConfigurationSpaceName.X};
      double[] topLowerLimits = new double[] {height - thicknessForViz, -Math.PI, -radius * topAreaReductionRatio};
      double[] topUpperLimits = new double[] {height, Math.PI, radius * topAreaReductionRatio};

      topMessage.getManifoldOriginPosition().set(manifoldOriginPosition);
      RotationMatrix topOrientation = new RotationMatrix(manifoldOriginOrientation);
      topOrientation.appendRollRotation(robotSide.negateIfRightSide(0.5 * Math.PI));
      topMessage.getManifoldOriginOrientation().set(topOrientation);

      HumanoidMessageTools.packManifold(ConfigurationSpaceName.toBytes(topSpaces), topLowerLimits, topUpperLimits, topMessage);

      ConfigurationSpaceName[] bottomSpaces = {ConfigurationSpaceName.Y, ConfigurationSpaceName.PITCH, ConfigurationSpaceName.X};
      double[] bottomLowerLimits = new double[] {0.0 - thicknessForViz, -Math.PI, -radius * topAreaReductionRatio};
      double[] bottomUpperLimits = new double[] {0.0, Math.PI, radius * topAreaReductionRatio};

      bottomMessage.getManifoldOriginPosition().set(manifoldOriginPosition);
      RotationMatrix bottomOrientation = new RotationMatrix(manifoldOriginOrientation);
      bottomOrientation.appendRollRotation(robotSide.negateIfRightSide(-0.5 * Math.PI));
      bottomMessage.getManifoldOriginOrientation().set(bottomOrientation);

      HumanoidMessageTools.packManifold(ConfigurationSpaceName.toBytes(bottomSpaces), bottomLowerLimits, bottomUpperLimits, bottomMessage);

      messages.add(sideMessage);
      messages.add(topMessage);
      messages.add(bottomMessage);

      return messages;
   }

   /**
    * match z and y direction of Hand control frame with x and y direction of torus.
    */
   public static List<ReachingManifoldMessage> createTorusManifoldMessagesForValkyrie(RobotSide robotSide, RigidBodyBasics hand,
                                                                                      Tuple3DReadOnly manifoldOriginPosition,
                                                                                      RotationMatrixReadOnly manifoldOriginOrientation, double radius,
                                                                                      double thickness)
   {
      double thicknessForViz = 0.01;

      List<ReachingManifoldMessage> messages = new ArrayList<>();
      ReachingManifoldMessage message = HumanoidMessageTools.createReachingManifoldMessage(hand);

      ConfigurationSpaceName[] spaces = {ConfigurationSpaceName.ROLL, ConfigurationSpaceName.Y, ConfigurationSpaceName.YAW, ConfigurationSpaceName.Y};
      double[] lowerLimits = new double[] {-Math.PI, radius, -Math.PI, thickness - thicknessForViz};
      double[] upperLimits = new double[] {Math.PI, radius, Math.PI, thickness};

      message.getManifoldOriginPosition().set(manifoldOriginPosition);
      RotationMatrix orientation = new RotationMatrix(manifoldOriginOrientation);
      orientation.appendPitchRotation(0.5 * Math.PI);
      message.getManifoldOriginOrientation().set(orientation);

      HumanoidMessageTools.packManifold(ConfigurationSpaceName.toBytes(spaces), lowerLimits, upperLimits, message);
      messages.add(message);

      return messages;
   }

   public static ReachingManifoldMessage createGoalManifoldMessage(RigidBodyBasics hand, FunctionTrajectory handFunction, double trajectoryTime,
                                                                   ConfigurationSpaceName[] manifoldSpaces)
   {
      ReachingManifoldMessage reachingManifoldMessage = HumanoidMessageTools.createReachingManifoldMessage(hand);

      reachingManifoldMessage.getManifoldOriginPosition().set(handFunction.compute(trajectoryTime).getPosition());
      reachingManifoldMessage.getManifoldOriginOrientation().set(handFunction.compute(trajectoryTime).getOrientation());

      double[] lowerLimits = new double[manifoldSpaces.length];
      double[] upperLimits = new double[manifoldSpaces.length];
      for (int i = 0; i < lowerLimits.length; i++)
      {
         lowerLimits[i] = manifoldSpaces[i].getDefaultExplorationLowerLimit();
         upperLimits[i] = manifoldSpaces[i].getDefaultExplorationUpperLimit();
      }

      HumanoidMessageTools.packManifold(ConfigurationSpaceName.toBytes(manifoldSpaces), lowerLimits, upperLimits, reachingManifoldMessage);

      return reachingManifoldMessage;
   }

   public static ReachingManifoldMessage createGoalManifoldMessage(RigidBodyBasics hand, FunctionTrajectory handFunction, double trajectoryTime,
                                                                   ConfigurationSpaceName[] manifoldSpaces, double[] upperLimits, double[] lowerLimits)
   {
      ReachingManifoldMessage reachingManifoldMessage = HumanoidMessageTools.createReachingManifoldMessage(hand);

      reachingManifoldMessage.getManifoldOriginPosition().set(handFunction.compute(trajectoryTime).getPosition());
      reachingManifoldMessage.getManifoldOriginOrientation().set(handFunction.compute(trajectoryTime).getOrientation());

      HumanoidMessageTools.packManifold(ConfigurationSpaceName.toBytes(manifoldSpaces), lowerLimits, upperLimits, reachingManifoldMessage);

      return reachingManifoldMessage;
   }

   public static void packExtrapolatedTransform(RigidBodyTransform from, RigidBodyTransform to, double ratio, RigidBodyTransform toPack)
   {
      Point3D pointToPack = new Point3D();
      RotationMatrix orientationToPack = new RotationMatrix();

      packExtrapolatedPoint(from.getTranslationVector(), to.getTranslationVector(), ratio, pointToPack);
      packExtrapolatedOrienation(from.getRotationMatrix(), to.getRotationMatrix(), ratio, orientationToPack);

      toPack.setIdentity();
      toPack.setTranslation(pointToPack);
      toPack.setRotation(orientationToPack);
   }

   public static double getDistance(RigidBodyTransform origin, RigidBodyTransform end, RigidBodyTransform to, double positionWeight, double orientationWeight)
   {
      int wayPointSize = 100;

      double minimumDistance = Double.MAX_VALUE;
      for (int i = 0; i < wayPointSize; i++)
      {
         double interpolatedRatio = i / (double) 100;
         RigidBodyTransform interpolated = new RigidBodyTransform();

         packExtrapolatedTransform(origin, end, interpolatedRatio, interpolated);
         minimumDistance = Math.min(minimumDistance, getDistance(interpolated, to, positionWeight, orientationWeight));
      }

      return minimumDistance;
   }

   public static double getDistance(RigidBodyTransform from, RigidBodyTransform to, double positionWeight, double orientationWeight)
   {
      Point3D pointFrom = new Point3D(from.getTranslationVector());
      Quaternion orientationFrom = new Quaternion(from.getRotationMatrix());

      Point3D pointTo = new Point3D(to.getTranslationVector());
      Quaternion orientationTo = new Quaternion(to.getRotationMatrix());

      double positionDistance = positionWeight * pointFrom.distance(pointTo);
      double orientationDistance = orientationWeight * orientationFrom.distance(orientationTo);

      double distance = positionDistance + orientationDistance;

      return distance;
   }

   private static void packExtrapolatedPoint(Vector3DReadOnly from, Vector3DReadOnly to, double ratio, Point3D toPack)
   {
      toPack.setX(ratio * (to.getX() - from.getX()) + from.getX());
      toPack.setY(ratio * (to.getY() - from.getY()) + from.getY());
      toPack.setZ(ratio * (to.getZ() - from.getZ()) + from.getZ());
   }

   private static void packExtrapolatedOrienation(RotationMatrixReadOnly from, RotationMatrixReadOnly to, double ratio, RotationMatrix toPack)
   {
      Quaternion invFrom = new Quaternion(from);
      invFrom.inverse();

      Quaternion delFromTo = new Quaternion();
      delFromTo.multiply(invFrom, new Quaternion(to));

      AxisAngle delFromToAxisAngle = new AxisAngle();
      AxisAngleConversion.convertQuaternionToAxisAngle(delFromTo, delFromToAxisAngle);

      AxisAngle delFromExtraAxisAngle = new AxisAngle(delFromToAxisAngle);
      double extrapolatedAngle = ratio * delFromToAxisAngle.getAngle();
      delFromExtraAxisAngle.setAngle(extrapolatedAngle);

      AxisAngle toPackAxisAngle = new AxisAngle(from);
      toPackAxisAngle.multiply(delFromExtraAxisAngle);

      AxisAngle temp = new AxisAngle(from);
      temp.multiply(delFromToAxisAngle);

      RotationMatrixConversion.convertAxisAngleToMatrix(toPackAxisAngle, toPack);
   }

   private static double packClosestRigidBodyTransformOnManifold(ReachingManifoldCommand reachingManifoldCommand, RigidBodyTransform rigidBodyTransform,
                                                                 RigidBodyTransform rigidBodyTransformToPack, double positionWeight, double orientationWeight)
   {
      double[] manifoldUpperLimits = reachingManifoldCommand.getManifoldUpperLimits().toArray();
      double[] manifoldLowerLimits = reachingManifoldCommand.getManifoldLowerLimits().toArray();

      TDoubleArrayList initialInput = new TDoubleArrayList();
      TDoubleArrayList upperLimits = new TDoubleArrayList();
      TDoubleArrayList lowerLimits = new TDoubleArrayList();
      for (int i = 0; i < manifoldLowerLimits.length; i++)
      {
         initialInput.add((manifoldUpperLimits[i] + manifoldLowerLimits[i]) / 2);
         upperLimits.add(manifoldUpperLimits[i]);
         lowerLimits.add(manifoldLowerLimits[i]);
      }

      SingleQueryFunction function = new SingleQueryFunction()
      {
         @Override
         public double getQuery(TDoubleArrayList values)
         {
            RigidBodyTransform closestTransform = new RigidBodyTransform();
            packRigidBodyTransformOnManifold(reachingManifoldCommand, values, closestTransform);

            return getDistance(rigidBodyTransform, closestTransform, positionWeight, orientationWeight);
         }
      };
      GradientDescentModule solver = new GradientDescentModule(function, initialInput);

      solver.setMaximumIterations(200);
      solver.setInputLowerLimit(lowerLimits);
      solver.setInputUpperLimit(upperLimits);

      solver.run();
      TDoubleArrayList optimalSolution = solver.getOptimalInput();

      rigidBodyTransformToPack.setIdentity();
      rigidBodyTransformToPack.appendTranslation(reachingManifoldCommand.getManifoldOriginPosition());
      rigidBodyTransformToPack.setRotation(reachingManifoldCommand.getManifoldOriginOrientation());

      for (int i = 0; i < reachingManifoldCommand.getDimensionOfManifold(); i++)
      {
         RigidBodyTransform appendingTransform = ExploringRigidBodyTools.getLocalRigidBodyTransform(reachingManifoldCommand.getDegreeOfManifold(i), optimalSolution.get(i));
         rigidBodyTransformToPack.multiply(appendingTransform);
      }

      return solver.getOptimalQuery();
   }

   private static void packRigidBodyTransformOnManifold(ReachingManifoldCommand reachingManifoldCommand, TDoubleArrayList configurations,
                                                        RigidBodyTransform rigidBodyTransformToPack)
   {
      int dimensionOfManifold = reachingManifoldCommand.getDimensionOfManifold();
      if (dimensionOfManifold != configurations.size())
         throw new MismatchedSizeException("configuration space size and name size are not matched.");

      rigidBodyTransformToPack.setIdentity();
      rigidBodyTransformToPack.setTranslation(reachingManifoldCommand.getManifoldOriginPosition());
      rigidBodyTransformToPack.setRotation(reachingManifoldCommand.getManifoldOriginOrientation());

      for (int i = 0; i < configurations.size(); i++)
      {
         RigidBodyTransform appendingTransform = ExploringRigidBodyTools.getLocalRigidBodyTransform(reachingManifoldCommand.getDegreeOfManifold(i), configurations.get(i));
         rigidBodyTransformToPack.multiply(appendingTransform);
      }

   }
}