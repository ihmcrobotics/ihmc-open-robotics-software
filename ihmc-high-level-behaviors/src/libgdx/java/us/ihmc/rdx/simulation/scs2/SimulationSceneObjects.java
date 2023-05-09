package us.ihmc.rdx.simulation.scs2;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.perception.sceneGraph.multiBodies.door.DoorDefinition;
import us.ihmc.rdx.simulation.environment.object.objects.TableDefinition;

/**
 * These are preplaced objects used for behaviors, manipulation,
 * navigation simulations with {@link RDXSCS2SimulationSession}.
 *
 * TODO: This is a very early version of this. If you see an improvement,
 *   like better parameterizing initial state, that's good, let's do it.
 *   Eventually we should make an SCS 2 scene editor.
 */
public class SimulationSceneObjects
{
   public static RestartableSCS2SessionRobot pushDoor()
   {
      return new RestartableSCS2SessionRobot(() ->
      {
         DoorDefinition doorDefinition = buildDoorWithArUcoMarkers();
         // Rotate the door so the push side is facing
         doorDefinition.getInitialSixDoFState().setConfiguration(new YawPitchRoll(Math.PI, 0.0, 0.0), new Point3D(1.3, 0.5, 0.01));
         return doorDefinition;
      }, DoorDefinition::applyPDController);
   }

   public static RestartableSCS2SessionRobot pullDoor()
   {
      return new RestartableSCS2SessionRobot(() ->
      {
         DoorDefinition doorDefinition = buildDoorWithArUcoMarkers();
         doorDefinition.getInitialSixDoFState().setConfiguration(new YawPitchRoll(0.0, 0.0, 0.0), new Point3D(1.0, -0.5, 0.01));
         return doorDefinition;
      }, DoorDefinition::applyPDController);
   }

   private static DoorDefinition buildDoorWithArUcoMarkers()
   {
      DoorDefinition doorDefinition = new DoorDefinition();
      doorDefinition.getDoorPanelDefinition().setAddArUcoMarkers(true);
      doorDefinition.build();
      // doorDefinition.getInitialHingeState().setEffort(15.0);
      return doorDefinition;
   }

   public static RestartableSCS2SessionRobot table()
   {
      return new RestartableSCS2SessionRobot(() ->
      {
         TableDefinition tableDefinition = new TableDefinition();
         tableDefinition.setAddArUcoMarkers(true);
         tableDefinition.build();
         tableDefinition.getInitialSixDoFState().setConfiguration(new YawPitchRoll(0.0, 0.0, 0.0), new Point3D(-1.0, 2.0, 0.86));
         return tableDefinition;
      }, robot -> {});
   }
}
