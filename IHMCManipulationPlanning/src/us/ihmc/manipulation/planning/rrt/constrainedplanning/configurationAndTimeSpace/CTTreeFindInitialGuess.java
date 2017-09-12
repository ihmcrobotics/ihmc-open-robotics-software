package us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ForkJoinPool;
import java.util.function.IntConsumer;
import java.util.stream.IntStream;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConfigurationSpace;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConstrainedEndEffectorTrajectory;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.RobotKinematicsConfiguration;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.TaskRegion;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.CTTreeTools;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.tools.WheneverWholeBodyKinematicsSolver;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoInteger;

public class CTTreeFindInitialGuess
{
   private final int numberOfSolvers;
   
   private final List<WheneverWholeBodyKinematicsSolver> kinematicsSolvers = new ArrayList<>();

   private final List<YoInteger> cntKinematicsSolvers = new ArrayList<>();

   private final List<SideDependentList<YoFramePose>> endeffectorPoses = new ArrayList<>();

   private final List<CTTaskNode> initialGuessNodes = new ArrayList<>();

   private final List<YoBoolean> validities = new ArrayList<>();

   private final ForkJoinPool forkJoinPool;

   private double bestScore = 0.0;

   private CTTaskNode bestNode;

   public CTTreeFindInitialGuess(FullHumanoidRobotModelFactory fullRobotModelFactory, int numberOfThread, YoVariableRegistry registry)
   {
      this(fullRobotModelFactory, numberOfThread, numberOfThread, registry);
   }

   public CTTreeFindInitialGuess(FullHumanoidRobotModelFactory fullRobotModelFactory, int numberOfThread, int numberOfSolvers, YoVariableRegistry registry)
   {

      this.numberOfSolvers = numberOfSolvers;

      forkJoinPool = new ForkJoinPool(numberOfThread);

      for (int i = 0; i < numberOfSolvers; i++)
      {
         kinematicsSolvers.add(new WheneverWholeBodyKinematicsSolver(fullRobotModelFactory));
         cntKinematicsSolvers.add(new YoInteger("cntKinematicsSolver" + i, registry));
         SideDependentList<YoFramePose> endeffectorPose = new SideDependentList<>();
         for (RobotSide robotSide : RobotSide.values)
            endeffectorPose.put(robotSide, new YoFramePose("" + robotSide + "endeffectorPoseFrame" + i, ReferenceFrame.getWorldFrame(), registry));
         endeffectorPoses.add(endeffectorPose);
         validities.add(new YoBoolean("currentIsValid" + i, registry));
      }
   }

   public void findInitialGuess(CTTaskNode node, TaskRegion taskRegion, RobotKinematicsConfiguration initialConfiguration,
                                ConstrainedEndEffectorTrajectory constrainedEndEffectorTrajectory, double handCoordinateOffsetX)
         throws InterruptedException, ExecutionException
   {
      initialGuessNodes.clear();

      for (int i = 0; i < numberOfSolvers; i++)
      {
         CTTaskNode initialGuessNode = new CTTaskNode(node);

         CTTreeTools.setRandomNormalizedNodeData(initialGuessNode, true, 0.0);
         initialGuessNode.setNormalizedNodeData(0, 0);
         initialGuessNode.convertNormalizedDataToData(taskRegion);

         initialGuessNodes.add(initialGuessNode);
      }

      forkJoinPool.submit(() -> IntStream.range(0, numberOfSolvers).parallel().forEach(new IntConsumer()
      {
         @Override
         public void accept(int index)
         {
            CTTaskNode initialGuessNode = initialGuessNodes.get(index);
            WheneverWholeBodyKinematicsSolver kinematicsSolver = kinematicsSolvers.get(index);
            SideDependentList<YoFramePose> endeffectorPose = endeffectorPoses.get(index);
            YoInteger cntKinematicSolver = cntKinematicsSolvers.get(index);

            updateValidity(initialGuessNode, kinematicsSolver, initialConfiguration, constrainedEndEffectorTrajectory, handCoordinateOffsetX, endeffectorPose,
                           cntKinematicSolver);
            validities.get(index).set(initialGuessNode.getValidity());
         }
      })).get();

      // TODO wait until done
      // Setup data for visualization

      bestScore = kinematicsSolvers.get(0).getArmJointLimitScore();
      if (!validities.get(0).getBooleanValue())
         bestScore = 0.0;
      bestNode = initialGuessNodes.get(0);

      for (int i = 1; i < numberOfSolvers; i++)
      {
         if (!validities.get(i).getBooleanValue())
            continue;

         if (bestScore < kinematicsSolvers.get(i).getArmJointLimitScore())
         {
            bestScore = kinematicsSolvers.get(i).getArmJointLimitScore();
            bestNode = initialGuessNodes.get(i);
         }
      }

   }

   private static boolean updateValidity(CTTaskNode node, WheneverWholeBodyKinematicsSolver kinematicsSolver, RobotKinematicsConfiguration initialConfiguration,
                                         ConstrainedEndEffectorTrajectory constrainedEndEffectorTrajectory, double handCoordinateOffsetX,
                                         SideDependentList<YoFramePose> endeffectorPose, YoInteger cntKinematicSolver)
   {
      if (node.getParentNode() != null)
      {
         kinematicsSolver.updateRobotConfigurationData(node.getParentNode().getConfiguration());
      }
      else
      {
         kinematicsSolver.updateRobotConfigurationData(initialConfiguration);
      }

      kinematicsSolver.initialize();

      kinematicsSolver.holdCurrentTrajectoryMessages();

      /*
       * set whole body tasks. pose from 'constrainedEndEffectorTrajectory' is
       * considered as being in MidZUpframe. for kinematics solver, append
       * offset
       */
      SideDependentList<ConfigurationSpace> configurationSpaces = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         configurationSpaces.put(robotSide, CTTreeTools.getConfigurationSpace(node, robotSide));
         
         Pose3D desiredPose = constrainedEndEffectorTrajectory.getEndEffectorPose(node.getNodeData(0), robotSide, configurationSpaces.get(robotSide));

         endeffectorPose.get(robotSide).setPosition(desiredPose.getPosition());
         endeffectorPose.get(robotSide).setOrientation(desiredPose.getOrientation());

         desiredPose.appendTranslation(handCoordinateOffsetX, 0.0, 0.0);

         kinematicsSolver.setDesiredHandPose(robotSide, desiredPose);
      }

      Quaternion desiredChestOrientation = new Quaternion();
      desiredChestOrientation.appendYawRotation(node.getNodeData(2));

      desiredChestOrientation.appendPitchRotation(node.getNodeData(3));
      desiredChestOrientation.appendRollRotation(node.getNodeData(4));
      kinematicsSolver.setDesiredChestOrientation(desiredChestOrientation);

      kinematicsSolver.setDesiredPelvisHeight(node.getNodeData(1));

      kinematicsSolver.putTrajectoryMessages();

      /*
       * result
       */
      kinematicsSolver.solve();
      boolean result = kinematicsSolver.getResult();

      //      node.setConfigurationJoints(kinematicsSolver.getFullRobotModelCopy());
      node.setConfigurationJoints(kinematicsSolver.getDesiredFullRobotModel());

      node.setValidity(result);

      cntKinematicSolver.set(kinematicsSolver.getCntForUpdateInternal());

      return result;
   }

   public double getBestScore()
   {
      return bestScore;
   }

   public CTTaskNode getBestNode()
   {
      return bestNode;
   }

   public List<YoInteger> getCntKinematicsSolvers()
   {
      return cntKinematicsSolvers;
   }

   public List<SideDependentList<YoFramePose>> getEndeffectorPoses()
   {
      return endeffectorPoses;
   }

   public List<CTTaskNode> getInitialGuessNodes()
   {
      return initialGuessNodes;
   }

   public List<YoBoolean> getValidities()
   {
      return validities;
   }

}
