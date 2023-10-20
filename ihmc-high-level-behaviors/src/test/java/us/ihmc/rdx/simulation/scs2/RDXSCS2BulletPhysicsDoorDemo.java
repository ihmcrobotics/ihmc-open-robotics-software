package us.ihmc.rdx.simulation.scs2;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.behaviors.simulation.LabFloorDefinition;
import us.ihmc.perception.sceneGraph.multiBodies.door.DoorDefinition;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletPhysicsEngine;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.tools.UnitConversions;

public class RDXSCS2BulletPhysicsDoorDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final RDXSCS2SimulationSession scs2SimulationSession = new RDXSCS2SimulationSession();

   public RDXSCS2BulletPhysicsDoorDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            scs2SimulationSession.create(baseUI);
            SimulationSession simulationSession = new SimulationSession(BulletPhysicsEngine::new);

            DoorDefinition doorDefinition = new DoorDefinition();
            doorDefinition.getDoorPanelDefinition().setAddArUcoMarkers(false);
            doorDefinition.build();
            doorDefinition.getInitialSixDoFState().setConfiguration(new YawPitchRoll(0.1, 0.1, 0.1), new Point3D(0.0, 0.0, 0.5));
            doorDefinition.getInitialHingeState().setEffort(15.0);
            Robot doorRobot = simulationSession.addRobot(doorDefinition);
            doorDefinition.applyPDController(doorRobot);

            simulationSession.addTerrainObject(new LabFloorDefinition());

            scs2SimulationSession.startSession(simulationSession);
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
