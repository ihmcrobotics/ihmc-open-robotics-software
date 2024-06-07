package us.ihmc.behaviors.simulation;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.behaviors.simulation.door.DoorDefinition;
import us.ihmc.perception.sceneGraph.rigidBody.RigidBodySceneObjectDefinitions;
import us.ihmc.perception.sceneGraph.rigidBody.TableModelParameters;
import us.ihmc.scs2.simulation.robot.Robot;

import java.util.function.Function;

/**
 * These are preplaced objects used for behaviors, manipulation,
 * navigation simulations.
 *
 * We return Function<ReferenceFrame, Robot> because we don't know the
 * inertial frame of the Session required to make the Robot.
 *
 * TODO: This is a very early version of this. If you see an improvement,
 *   like better parameterizing initial state, that's good, let's do it.
 *   Eventually we should make an SCS 2 scene editor.
 */
public class SimulationSceneObjectRobotBuilders
{
   public static final double SPACE_TO_ALLOW_IT_TO_FALL_ONTO_SURFACE = 0.01;
   public static final double TABLE_X = -1.0;
   public static final double TABLE_Y = 2.0;
   public static final double TABLE_Z = TableModelParameters.TABLE_LEG_LENGTH + SPACE_TO_ALLOW_IT_TO_FALL_ONTO_SURFACE;
   public static final double TABLE_SURFACE_Z = TableModelParameters.TABLE_LEG_LENGTH + TableModelParameters.TABLE_THICKNESS;

   public static Function<ReferenceFrame, Robot> getPushDoorBuilder(int id, Point3D position)
   {
      return inertialFrame ->
      {
         DoorDefinition doorDefinition = getDoorWithArUcoMarkersDefinition(id);
         // Rotate the door so the push side is facing
         doorDefinition.getInitialSixDoFState().setConfiguration(new YawPitchRoll(Math.PI, 0.0, 0.0), position);
         Robot robot = new Robot(doorDefinition, inertialFrame);
         return robot;
      };
   }

   public static Function<ReferenceFrame, Robot> getPullDoorBuilder(int id, Point3D position)
   {
      return inertialFrame ->
      {
         DoorDefinition doorDefinition = getDoorWithArUcoMarkersDefinition(id);
         doorDefinition.getInitialSixDoFState().setConfiguration(new YawPitchRoll(0.0, 0.0, 0.0), position);
         Robot robot = new Robot(doorDefinition, inertialFrame);
         return robot;
      };
   }

   private static DoorDefinition getDoorWithArUcoMarkersDefinition(int id)
   {
      DoorDefinition doorDefinition = new DoorDefinition(id);
      doorDefinition.getDoorPanelDefinition().setAddArUcoMarkers(true);
      doorDefinition.build();
      // doorDefinition.getInitialHingeState().setEffort(15.0);
      return doorDefinition;
   }

   public static Function<ReferenceFrame, Robot> getTableBuilder()
   {
      return inertialFrame ->
      {
         TableDefinition tableDefinition = new TableDefinition();
         tableDefinition.setAddArUcoMarkers(true);
         tableDefinition.build();
         tableDefinition.getInitialSixDoFState().setConfiguration(new YawPitchRoll(0.0, 0.0, 0.0), new Point3D(TABLE_X, TABLE_Y, TABLE_Z));
         Robot robot = new Robot(tableDefinition, inertialFrame);
         return robot;
      };
   }

   public static Function<ReferenceFrame, Robot> getCanOfSoupOnTableBuilder()
   {
      return inertialFrame ->
      {
         CanOfSoupDefinition canOfSoupDefinition = new CanOfSoupDefinition();
         canOfSoupDefinition.build();
         canOfSoupDefinition.getInitialSixDoFState().setConfiguration(new YawPitchRoll(0.0, 0.0, 0.0),
            new Point3D(TABLE_X + TableModelParameters.TABLE_ARUCO_MARKER_FROM_LEFT_EDGE + RigidBodySceneObjectDefinitions.MARKER_TO_CAN_OF_SOUP_X_WORLD,
                        TABLE_Y + RigidBodySceneObjectDefinitions.CAN_OF_SOUP_FROM_TABLE_EDGE,
                        TABLE_SURFACE_Z + SPACE_TO_ALLOW_IT_TO_FALL_ONTO_SURFACE));
         Robot robot = new Robot(canOfSoupDefinition, inertialFrame);
         return robot;
      };
   }
}
