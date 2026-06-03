package us.ihmc.zulu.rdx.apps;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.behaviors.simulation.FlatGroundDefinition;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.scs2.RDXSCS2HumanoidSimulationManager;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.zulu.ZuluRobotModel;
import us.ihmc.zulu.ZuluVersion;

/**
 * Walking demo for Zulu with RDX UI.
 * Click the Walk CSG checkbox to start walking around.
 */
public class ZuluRDXFlatGroundWalkingTrack
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private RDXSCS2HumanoidSimulationManager scs2HumanoidSimulationManager;

   public ZuluRDXFlatGroundWalkingTrack()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create(RDXSceneLevel.values());
            baseUI.getPrimary3DPanel().getCamera3D().setCameraFocusPoint(new Point3D(0.7, 0.0, 0.4));
            baseUI.getPrimary3DPanel().getCamera3D().changeCameraPosition(-3.0, -4.0, 4.0);

            ZuluRobotModel robotModel = new ZuluRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS);

            scs2HumanoidSimulationManager = new RDXSCS2HumanoidSimulationManager(baseUI, robotModel);
            FlatGroundDefinition flatGroundDefinition = new FlatGroundDefinition();
            flatGroundDefinition.translate(0.0, 0.0, -0.01); // So feet aren't colliding at the beginning
            scs2HumanoidSimulationManager.getTerrainObjectDefinitions().add(flatGroundDefinition);
            scs2HumanoidSimulationManager.getOnSessionStartedRunnables().add(() ->
            {
               scs2HumanoidSimulationManager.setPauseAtEndOfBuffer(false);
               SimulationConstructionSet2 scs = scs2HumanoidSimulationManager.getAvatarSimulation().getSimulationConstructionSet();
               YoBoolean ignoreWalkInputProvider = (YoBoolean) scs.findVariable("ignoreWalkInputProviderCSG");
               YoBoolean overrideHeartbeat = (YoBoolean) scs.findVariable("overrideHeartbeat_StepGeneratorCommandInputManager");
               if (ignoreWalkInputProvider != null)
                  ignoreWalkInputProvider.set(true);
               if (overrideHeartbeat != null)
                  overrideHeartbeat.set(true);
            });
            scs2HumanoidSimulationManager.setExternalFactorySetup(factory ->
            {
               factory.setUseBulletPhysicsEngine(false);
               factory.setLogToFile(false);
               factory.setSimulationDataRecordTickPeriod(20);
               factory.setSimulationDataBufferDuration(5.0);
            });
            scs2HumanoidSimulationManager.addVariableWidget(
                  "root.Zulu.DRCSimulation.csgRegistry.HumanoidSteppingManager.ContinuousStepGenerator.walkCSG");
            scs2HumanoidSimulationManager.buildSimulation();
         }

         @Override
         public void render()
         {
            scs2HumanoidSimulationManager.update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            scs2HumanoidSimulationManager.destroySessionForRebuild();
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new ZuluRDXFlatGroundWalkingTrack();
   }
}
