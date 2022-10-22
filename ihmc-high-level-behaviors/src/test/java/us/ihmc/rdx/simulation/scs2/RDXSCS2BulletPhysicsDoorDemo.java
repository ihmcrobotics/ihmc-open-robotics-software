package us.ihmc.rdx.simulation.scs2;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.simulation.environment.object.objects.LabFloorDefinition;
import us.ihmc.rdx.simulation.environment.object.objects.door.DoorDefinition;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.tools.UnitConversions;

public class RDXSCS2BulletPhysicsDoorDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI(getClass(),
                                                  "ihmc-open-robotics-software",
                                                  "ihmc-high-level-behaviors/src/test/resources");
   private final RDXSCS2BulletSimulationSession scs2SimulationSession = new RDXSCS2BulletSimulationSession();

   public RDXSCS2BulletPhysicsDoorDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            DoorDefinition doorDefinition = new DoorDefinition();
            doorDefinition.getDoorPanelDefinition().setAddFiducials(false);
            doorDefinition.build();
            doorDefinition.getInitialSixDoFState().setConfiguration(new YawPitchRoll(0.1, 0.1, 0.1), new Point3D(0.0, 0.0, 0.5));
            doorDefinition.getInitialHingeState().setEffort(15.0);
            Robot doorRobot = scs2SimulationSession.addRobot(doorDefinition);
            doorDefinition.applyPDController(doorRobot);

            scs2SimulationSession.addTerrainObject(new LabFloorDefinition());

            scs2SimulationSession.create(baseUI);
            scs2SimulationSession.setDT(UnitConversions.hertzToSeconds(240.0));
            baseUI.getImGuiPanelManager().addPanel(scs2SimulationSession.getControlPanel());
         }

         @Override
         public void render()
         {
            scs2SimulationSession.update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXSCS2BulletPhysicsDoorDemo();
   }
}
