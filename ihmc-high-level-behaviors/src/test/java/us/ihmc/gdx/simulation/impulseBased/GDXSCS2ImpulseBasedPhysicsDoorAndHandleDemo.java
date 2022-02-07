package us.ihmc.gdx.simulation.impulseBased;

import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.simulation.scs2.GDXSCS2PhysicsSimulator;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.mecano.tools.MomentOfInertiaFactory;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.visual.*;

public class GDXSCS2ImpulseBasedPhysicsDoorAndHandleDemo
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/test/resources");
   private final GDXSCS2PhysicsSimulator physicsSimulator = new GDXSCS2PhysicsSimulator();

   public GDXSCS2ImpulseBasedPhysicsDoorAndHandleDemo()
   {


      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

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
      new GDXSCS2ImpulseBasedPhysicsDoorAndHandleDemo();
   }
}
