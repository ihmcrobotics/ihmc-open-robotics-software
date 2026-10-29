package us.ihmc.rdx.ui.plotting;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.scs2.RDXSCS2PlotPanel;
import us.ihmc.rdx.simulation.scs2.RDXSCS2SimulationSession;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.examples.simulations.ExampleExperimentalSimulationTools;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngineFactory;

import java.util.Random;

public class RDXSCS2StyleChartDemo
{
   private final RDXBaseUI baseUI;
   private RDXSCS2SimulationSession session;

   public RDXSCS2StyleChartDemo()
   {
      baseUI = new RDXBaseUI();

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create(RDXSceneLevel.values());

            session = new RDXSCS2SimulationSession(baseUI);
            session.startSession(createStackOfBlocksSession());
            session.changeBufferDuration(1.0);
            session.getSession().setSessionMode(SessionMode.RUNNING);

            RDXSCS2PlotPanel plotPanel = session.getPlottingManager().addPlotPanel("Poses");
            plotPanel.setRows(7);
            plotPanel.setColumns(5);
            for (int i = 0; i < 5; i++)
               plotPanel.plot(i, 0, "root.Block%d.q_Block%d_x".formatted(i, i));
            for (int i = 0; i < 5; i++)
               plotPanel.plot(i, 1, "root.Block%d.q_Block%d_y".formatted(i, i));
            for (int i = 0; i < 5; i++)
               plotPanel.plot(i, 2, "root.Block%d.q_Block%d_z".formatted(i, i));
            for (int i = 0; i < 5; i++)
               plotPanel.plot(i, 3, "root.Block%d.q_Block%d_qx".formatted(i, i));
            for (int i = 0; i < 5; i++)
               plotPanel.plot(i, 4, "root.Block%d.q_Block%d_qy".formatted(i, i));
            for (int i = 0; i < 5; i++)
               plotPanel.plot(i, 5, "root.Block%d.q_Block%d_qz".formatted(i, i));
            for (int i = 0; i < 5; i++)
               plotPanel.plot(i, 6, "root.Block%d.q_Block%d_qs".formatted(i, i));

            int block = 1;
            plotPanel.plot(0, 0, "root.Block%d.q_Block%d_x".formatted(block, block));
            plotPanel.plot(0, 0, "root.Block%d.q_Block%d_x".formatted(2, 2));
            plotPanel.plot(0, 1, "root.Block%d.q_Block%d_y".formatted(block, block));
            plotPanel.plot(0, 2, "root.Block%d.q_Block%d_z".formatted(block, block));
            plotPanel.plot(0, 3, "root.Block%d.q_Block%d_qx".formatted(block, block));
            plotPanel.plot(0, 4, "root.Block%d.q_Block%d_qy".formatted(block, block));
            plotPanel.plot(0, 5, "root.Block%d.q_Block%d_qz".formatted(block, block));
            plotPanel.plot(0, 6, "root.Block%d.q_Block%d_qs".formatted(block, block));
         }

         @Override
         public void render()
         {
            session.update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            session.destroy(baseUI);
            baseUI.dispose();
         }
      });
   }

   private static SimulationSession createStackOfBlocksSession()
   {
      SimulationSession simulationSession = new SimulationSession(PhysicsEngineFactory.newImpulseBasedPhysicsEngineFactory());

      Random random = new Random(1886L);
      double boxSizeX = 0.1;
      double boxSizeY = 0.08;
      double boxSizeZ = 0.1;
      for (int i = 0; i < 6; i++)
      {
         String name = "Block" + i;
         RobotDefinition boxRobot = ExampleExperimentalSimulationTools.newBoxRobot(name, boxSizeX, boxSizeY, boxSizeZ, 0.2, 0.5, ColorDefinition.rgb(random.nextInt()));
         boxRobot.getRigidBodyDefinition(name + "RigidBody").addCollisionShapeDefinition(new CollisionShapeDefinition(new Box3DDefinition(boxSizeX, boxSizeY, boxSizeZ)));
         double pitch = (random.nextDouble() - 0.5) * Math.PI / 45.0;
         double roll = (random.nextDouble() - 0.5) * Math.PI / 45.0;
         Point3D position = new Point3D(0.0, i * 0.02, boxSizeZ * 2.1 * (i + 1.0));
         boxRobot.getRootJointDefinitions().get(0).setInitialJointState(new SixDoFJointState(new YawPitchRoll(0.0, pitch, roll), position));
         simulationSession.addRobot(boxRobot);
      }

      Box3DDefinition terrainGeometry = new Box3DDefinition(5.0, 5.0, 0.1);
      RigidBodyTransform terrainPose = new RigidBodyTransform();
      terrainPose.getTranslation().subZ(0.05);
      simulationSession.addTerrainObject(new TerrainObjectDefinition(new VisualDefinition(terrainPose,
                                                                                          terrainGeometry,
                                                                                          new MaterialDefinition(ColorDefinitions.DarkKhaki())),
                                                                     new CollisionShapeDefinition(terrainPose, terrainGeometry)));
      return simulationSession;
   }

   public static void main(String[] args)
   {
      new RDXSCS2StyleChartDemo();
   }
}
