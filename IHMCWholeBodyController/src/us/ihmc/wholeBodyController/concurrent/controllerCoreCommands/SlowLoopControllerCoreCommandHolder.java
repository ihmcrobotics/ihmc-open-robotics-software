package us.ihmc.wholeBodyController.concurrent.controllerCoreCommands;

import java.util.HashMap;
import java.util.Map;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.ExternalWrenchCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.JointspaceAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.PlaneContactStateCommandPool;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.PointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.SpatialAccelerationCommand;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;

public class SlowLoopControllerCoreCommandHolder
{
   private static final int INITIAL_CAPACITY = 15;
   private final Map<String, RigidBody> slowLoopRigidBodyMap = new HashMap<>();
   private final Map<String, RigidBody> fastLoopRigidBodyMap = new HashMap<>();

   private final InverseDynamicsCommandList inverseDynamicsCommandList = new InverseDynamicsCommandList();
   private final PlaneContactStateCommandPool planeContactStateCommandPool = new PlaneContactStateCommandPool();

   private final RecyclingArrayList<ExternalWrenchCommand> externalWrenchCommands = new RecyclingArrayList<>(INITIAL_CAPACITY, ExternalWrenchCommand.class);
   private final RecyclingArrayList<JointspaceAccelerationCommand> jointspaceAccelerationCommands = new RecyclingArrayList<>(INITIAL_CAPACITY, JointspaceAccelerationCommand.class);
   private final RecyclingArrayList<MomentumRateCommand> momentumRateCommands = new RecyclingArrayList<>(INITIAL_CAPACITY, MomentumRateCommand.class);
   private final RecyclingArrayList<PlaneContactStateCommand> planeContactStateCommands = new RecyclingArrayList<>(INITIAL_CAPACITY, PlaneContactStateCommand.class);
   private final RecyclingArrayList<PointAccelerationCommand> pointAccelerationCommands = new RecyclingArrayList<>(INITIAL_CAPACITY, PointAccelerationCommand.class);
   private final RecyclingArrayList<SpatialAccelerationCommand> spatialAccelerationCommands = new RecyclingArrayList<>(INITIAL_CAPACITY, SpatialAccelerationCommand.class);

   public SlowLoopControllerCoreCommandHolder(FullHumanoidRobotModel slowLoopFullRobotModel, FullHumanoidRobotModel fastLoopFullRobotModel)
   {
      setupRigidBodyMap(slowLoopFullRobotModel, slowLoopRigidBodyMap);
      setupRigidBodyMap(fastLoopFullRobotModel, fastLoopRigidBodyMap);
   }

   public void writeSlowLoopData(ControllerCoreCommand commandToWrite)
   {
      
   }

   // No need to put all of the rigid bodies (there is a lot especially when having hands).
   private static void setupRigidBodyMap(FullHumanoidRobotModel fullRobotModel, Map<String, RigidBody> rigidBodyMapToPack)
   {
      addRigidBodyToMap(fullRobotModel.getElevator(), rigidBodyMapToPack);
      addRigidBodyToMap(fullRobotModel.getHead(), rigidBodyMapToPack);
      addRigidBodyToMap(fullRobotModel.getChest(), rigidBodyMapToPack);
      addRigidBodyToMap(fullRobotModel.getPelvis(), rigidBodyMapToPack);

      for (RobotSide robotSide : RobotSide.values)
      {
         addRigidBodyToMap(fullRobotModel.getHand(robotSide), rigidBodyMapToPack);
         addRigidBodyToMap(fullRobotModel.getFoot(robotSide), rigidBodyMapToPack);
      }
   }

   public static void addRigidBodyToMap(RigidBody foot, Map<String, RigidBody> rigidBodyMapToPack)
   {
      rigidBodyMapToPack.put(foot.getName(), foot);
   }

   public static class Builder implements us.ihmc.concurrent.Builder<SlowLoopControllerCoreCommandHolder>
   {
      public Builder()
      {
      }

      @Override
      public SlowLoopControllerCoreCommandHolder newInstance()
      {
         return null;
      }
   }
}
