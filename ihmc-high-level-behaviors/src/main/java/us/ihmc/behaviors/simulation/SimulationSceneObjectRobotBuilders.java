package us.ihmc.behaviors.simulation;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.perception.sceneGraph.rigidBody.RigidBodySceneObjectDefinitions;
import us.ihmc.perception.sceneGraph.rigidBody.TableModelParameters;

/**
 * These are preplaced objects used for behaviors, manipulation,
 * navigation simulations.
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

   public static TableDefinition getTableBuilder(String name)
   {
      TableDefinition tableDefinition = new TableDefinition(name);
      tableDefinition.setAddArUcoMarkers(true);
      tableDefinition.build();
      tableDefinition.getInitialSixDoFState().setConfiguration(new YawPitchRoll(0.0, 0.0, 0.0), new Point3D(TABLE_X, TABLE_Y, TABLE_Z));
      return tableDefinition;
   }

   public static CanOfSoupDefinition getCanOfSoupOnTableBuilder(String name)
   {
      CanOfSoupDefinition canOfSoupDefinition = new CanOfSoupDefinition(name);
      canOfSoupDefinition.build();
      canOfSoupDefinition.getInitialSixDoFState().setConfiguration(new YawPitchRoll(0.0, 0.0, 0.0),
         new Point3D(TABLE_X + TableModelParameters.TABLE_ARUCO_MARKER_FROM_LEFT_EDGE + RigidBodySceneObjectDefinitions.MARKER_TO_CAN_OF_SOUP_X_WORLD,
                     TABLE_Y + RigidBodySceneObjectDefinitions.CAN_OF_SOUP_FROM_TABLE_EDGE,
                     TABLE_SURFACE_Z + SPACE_TO_ALLOW_IT_TO_FALL_ONTO_SURFACE));
      return canOfSoupDefinition;
   }
}
