package us.ihmc.avatar.scs2;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationDataReadOnly;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemStateIntegrator;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.RobotStateDefinition;
import us.ihmc.scs2.definition.state.interfaces.JointStateBasics;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngine;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.scs2.simulation.screwTools.SingleRobotFirstOrderIntegrator;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;

public class KinematicsOnlyPhysicsEngine implements PhysicsEngine
{
   private final ReferenceFrame inertialFrame;
//   private final MultiBodySystemStateIntegrator integrator = new MultiBodySystemStateIntegrator();
   private final SingleRobotFirstOrderIntegrator integrator = new SingleRobotFirstOrderIntegrator();
   private final YoRegistry rootRegistry;
   private final YoRegistry physicsEngineRegistry = new YoRegistry(getClass().getSimpleName());
   private final List<Robot> robotList = new ArrayList<>();
   private final List<TerrainObjectDefinition> terrainObjectDefinitions = new ArrayList<>();

   private HighLevelHumanoidControllerFactory highLevelHumanoidControllerFactory;
   private boolean hasBeenInitialized = false;

   public KinematicsOnlyPhysicsEngine(ReferenceFrame inertialFrame,
                                      YoRegistry rootRegistry)
   {
      this.rootRegistry = rootRegistry;
      this.inertialFrame = inertialFrame;
   }

   public void setHighLevelHumanoidControllerFactory(HighLevelHumanoidControllerFactory highLevelHumanoidControllerFactory)
   {
      this.highLevelHumanoidControllerFactory = highLevelHumanoidControllerFactory;
   }

   @Override
   public void addTerrainObject(TerrainObjectDefinition terrainObjectDefinition)
   {
      terrainObjectDefinitions.add(terrainObjectDefinition);
   }

   @Override
   public void addRobot(Robot robot)
   {
      inertialFrame.checkReferenceFrameMatch(robot.getInertialFrame());
      rootRegistry.addChild(robot.getRegistry());
      physicsEngineRegistry.addChild(robot.getSecondaryRegistry());
      robotList.add(robot);
   }

   @Override
   public void initialize(Vector3DReadOnly gravity)
   {
      for (Robot robot : robotList)
      {
         robot.initializeState();
         robot.getControllerManager().initializeControllers();
      }
      hasBeenInitialized = true;
   }

   @Override
   public void simulate(double currentTime, double dt, Vector3DReadOnly gravity)
   {
      if (!hasBeenInitialized)
      {
         initialize(gravity);
         return;
      }

      HighLevelHumanoidControllerToolbox controllerToolbox = highLevelHumanoidControllerFactory.getHighLevelHumanoidControllerToolbox();
      WholeBodyControllerCore controllerCore = highLevelHumanoidControllerFactory.getWholeBodyControllerCoreFactory().getWholeBodyControllerCore();
      RootJointDesiredConfigurationDataReadOnly outputForRootJoint = controllerCore.getOutputForRootJoint();

      for (Robot robot : robotList)
      {
         robot.updateFrames();
         robot.getControllerManager().updateControllers(currentTime);
         robot.getControllerManager().writeControllerOutputForAllJoints(JointStateType.values());

//         robot.get

         integrator.integrate(dt, robot);
      }
   }

   @Override
   public void pause()
   {
      for (Robot robot : robotList)
      {
         robot.getControllerManager().pauseControllers();
      }
   }

   @Override
   public ReferenceFrame getInertialFrame()
   {
      return inertialFrame;
   }

   @Override
   public List<Robot> getRobots()
   {
      return robotList;
   }

   @Override
   public List<RobotDefinition> getRobotDefinitions()
   {
      return robotList.stream().map(Robot::getRobotDefinition).collect(Collectors.toList());
   }

   @Override
   public List<TerrainObjectDefinition> getTerrainObjectDefinitions()
   {
      return terrainObjectDefinitions;
   }

   @Override
   public List<RobotStateDefinition> getBeforePhysicsRobotStateDefinitions()
   {
      return null;
   }

   @Override
   public YoRegistry getPhysicsEngineRegistry()
   {
      return physicsEngineRegistry;
   }
}
