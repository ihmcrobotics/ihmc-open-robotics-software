package us.ihmc.zulu;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.parameterEstimation.InertiaVisualizationTools;
import us.ihmc.commonWalkingControlModules.parameterEstimation.YoInertiaEllipsoid;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.definition.SCS2DefinitionTools;
import us.ihmc.scs2.definition.YawPitchRollTransformDefinition;
import us.ihmc.scs2.definition.robot.IMUSensorDefinition;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.OneDoFJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SensorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.collision.CollidableHelper;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.simulationToolkit.RobotDefinitionTools;

import java.util.ArrayList;

public class ZuluModelViewer
{
   private static final boolean SHOW_COORDINATES_AT_JOINT_ORIGIN = false;
   private static final boolean SHOW_IMU_FRAMES = false;
   private static final boolean SHOW_HAND_CONTROL_FRAME = false;
   private static final boolean SHOW_INERTIA_FRAME = false;
   private static final boolean SHOW_SIM_COLLISIONS = false;
   private static final boolean SHOW_KINEMATICS_COLLISIONS = false;
   private static final boolean SHOW_INERTIA_ELLIPSOIDS = false;

   public ZuluModelViewer()
   {
      DRCRobotModel robotModel = new ZuluRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS);
      RobotDefinition robotDefinition = robotModel.getRobotDefinition();

      if (SHOW_COORDINATES_AT_JOINT_ORIGIN)
      { // RGB -> XYZ
         addJointAxes(robotDefinition);
      }
      if (SHOW_IMU_FRAMES)
      {
         addIMUFrames(robotDefinition);
      }
      if (SHOW_HAND_CONTROL_FRAME)
      {
         addHandControlFrames(robotModel.getJointMap(), robotDefinition);
      }
      if (SHOW_INERTIA_FRAME)
      {
         addInertiaFrame(robotDefinition);
      }

      MaterialDefinition collisionMaterialDefinition = new MaterialDefinition(ColorDefinitions.Coral().derive(0, 1.0, 1.0, 0.3));
      if (SHOW_KINEMATICS_COLLISIONS)
      {
         RobotCollisionModel collisionModel = robotModel.getHumanoidRobotKinematicsCollisionModel();
         if (collisionModel != null)
         {
            RobotDefinitionTools.addCollisionsToRobotDefinition(collisionModel.getRobotCollidables(robotModel.createFullRobotModel().getElevator()),
                                                                robotDefinition);
            SCS2DefinitionTools.addCollisionVisualsToRobot(robotDefinition, collisionMaterialDefinition);
         }
      }

      if (SHOW_SIM_COLLISIONS)
      {
         RobotCollisionModel collisionModel = robotModel.getSimulationRobotCollisionModel(new CollidableHelper(), "robot", "ground");
         if (collisionModel != null)
         {
            RobotDefinitionTools.addCollisionsToRobotDefinition(collisionModel.getRobotCollidables(robotModel.createFullRobotModel().getElevator()),
                                                                robotDefinition);
            SCS2DefinitionTools.addCollisionVisualsToRobot(robotDefinition, collisionMaterialDefinition);
         }
      }

      SimulationSession session = new SimulationSession();
      Robot simulationRobot = session.addRobot(robotModel.getRobotDefinition());

      YoGraphicGroupDefinition extraViz = new YoGraphicGroupDefinition("ExtraVisualization", new ArrayList<>());

      if (SHOW_INERTIA_ELLIPSOIDS)
      {
         ArrayList<YoInertiaEllipsoid> inertialEllipsoids = InertiaVisualizationTools.createYoInertiaEllipsoids(simulationRobot.getRootBody(),
                                                                                                                session.getRootRegistry());
         YoGraphicDefinition ellipsoidGroup = InertiaVisualizationTools.getInertiaEllipsoidGroup(inertialEllipsoids);
         extraViz.addChild(ellipsoidGroup);
      }

      session.addYoGraphicDefinitions(extraViz);
      SessionVisualizer.startSessionVisualizer(session);
   }
   private static void addJointAxes(RobotDefinition robotDefinition)
   {
      double jointAxisGraphicSize = 0.15;

      for (OneDoFJointDefinition jointDefinition : robotDefinition.getAllOneDoFJoints())
      {
         VisualDefinitionFactory visualDefinitionFactory = new VisualDefinitionFactory();
         visualDefinitionFactory.addCoordinateSystem(jointAxisGraphicSize);
         jointDefinition.getSuccessor().getVisualDefinitions().addAll(visualDefinitionFactory.getVisualDefinitions());
      }
   }

   private static void addIMUFrames(RobotDefinition robotDefinition)
   {
      double imuFrameGraphicSize = 0.06;

      for (JointDefinition jointDefinition : robotDefinition.getAllJoints())
      {
         for (SensorDefinition sensorDefinition : jointDefinition.getSensorDefinitions())
         {
            if (sensorDefinition instanceof IMUSensorDefinition imuDefinition)
            {
               YawPitchRollTransformDefinition imuTransform = imuDefinition.getTransformToJoint();
               VisualDefinitionFactory visualDefinitionFactory = new VisualDefinitionFactory();
               visualDefinitionFactory.appendTransform(imuTransform);
               visualDefinitionFactory.addCoordinateSystem(imuFrameGraphicSize);
               jointDefinition.getSuccessor().getVisualDefinitions().addAll(visualDefinitionFactory.getVisualDefinitions());
            }
         }
      }
   }

   private static void addHandControlFrames(HumanoidJointNameMap jointMap, RobotDefinition robotDefinition)
   {
      double handControlFrameGraphicSize = 0.3;

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyTransform handControlFrameToWristTransform = jointMap.getHandControlFrameToWristTransform(robotSide);
         RigidBodyDefinition handDefinition = robotDefinition.getRigidBodyDefinition(jointMap.getHandName(robotSide));

         VisualDefinitionFactory visualDefinitionFactory = new VisualDefinitionFactory();
         visualDefinitionFactory.appendTransform(handControlFrameToWristTransform);
         visualDefinitionFactory.addCoordinateSystem(handControlFrameGraphicSize);
         handDefinition.getVisualDefinitions().addAll(visualDefinitionFactory.getVisualDefinitions());
      }
   }

   private static void addInertiaFrame(RobotDefinition robotDefinition)
   {
      double inertiaFrameGraphicSize = 0.3;

      for (JointDefinition jointDefinition : robotDefinition.getAllJoints())
      {
         RigidBodyDefinition rigidBodyDefinition = jointDefinition.getSuccessor();
         YawPitchRollTransformDefinition inertialToJointFrameTransform = rigidBodyDefinition.getInertiaPose();
         VisualDefinitionFactory visualDefinitionFactory = new VisualDefinitionFactory();
         visualDefinitionFactory.appendTransform(inertialToJointFrameTransform);
         visualDefinitionFactory.addCoordinateSystem(inertiaFrameGraphicSize);
         rigidBodyDefinition.getVisualDefinitions().addAll(visualDefinitionFactory.getVisualDefinitions());
      }
   }

   public static void main(String[] args)
   {
      new ZuluModelViewer();
   }
}
