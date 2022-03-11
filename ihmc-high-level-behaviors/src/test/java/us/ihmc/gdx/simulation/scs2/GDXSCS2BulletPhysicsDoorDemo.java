package us.ihmc.gdx.simulation.scs2;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.simulation.environment.object.objects.LabFloorDefinition;
import us.ihmc.gdx.simulation.environment.object.objects.door.DoorFrameDefinition;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.scs2.definition.robot.RobotDefinition;

public class GDXSCS2BulletPhysicsDoorDemo
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/test/resources");
   private final GDXSCS2PhysicsSimulator physicsSimulator = new GDXSCS2PhysicsSimulator();

   public GDXSCS2BulletPhysicsDoorDemo()
   {
      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            RobotDefinition robotDefinition = SCS2Tools.singleBodyRobot(new DoorFrameDefinition());
            SCS2Tools.setInitialPoseOfRobot(robotDefinition, new YawPitchRoll(), new Point3D(0.0, 0.0, 2.0));
            physicsSimulator.addRobot(robotDefinition);

            physicsSimulator.addTerrainObject(new LabFloorDefinition());

            physicsSimulator.create(baseUI);
            baseUI.getImGuiPanelManager().addPanel(physicsSimulator.getControlPanel());
         }

         @Override
         public void render()
         {
            physicsSimulator.update();

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
      new GDXSCS2BulletPhysicsDoorDemo();
   }
}
