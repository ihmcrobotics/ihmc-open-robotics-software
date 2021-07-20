package us.ihmc.gdx.simulation;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.simulation.environment.GDXPhysicsSimulator;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.sharedMemory.LinkedYoRegistry;
import us.ihmc.scs2.sharedMemory.tools.YoMirroredRegistryTools;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.yoVariables.registry.YoRegistry;

public class GDXPhysicsDemo
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/test/resources");

   private final GDXPhysicsSimulator physicsSimulator = new GDXPhysicsSimulator();
   private ModelInstance slopeModelInstance;
   private GDXRigidBody rootBody;
   private boolean initialize = true;
   private LinkedYoRegistry robotLinkedYoRegistry;

   public GDXPhysicsDemo()
   {
      BoxRobotDefinition boxRobot = new BoxRobotDefinition();
      SixDoFJointState initialJointState = new SixDoFJointState();
      initialJointState.setConfiguration(new Pose3D(0.0, 0.0, 1.0, 0.0, 0.0, 0.0));
      boxRobot.getRootJointDefinitions().get(0).setInitialJointState(initialJointState);

      physicsSimulator.getSession().addRobot(boxRobot);
      SlopeGroundDefinition slopeTerrain = new SlopeGroundDefinition();
      physicsSimulator.getSession().addTerrainObject(slopeTerrain);

      YoRegistry rootRegistry = physicsSimulator.getSession().getPhysicsEngine().getPhysicsEngineRegistry();
      YoRegistry mirroredBoxRegistry = YoMirroredRegistryTools.newRegistryFromNamespace(SimulationSession.ROOT_REGISTRY_NAME, boxRobot.getName());
      GDXYoManager yoManager = new GDXYoManager();
      yoManager.startSession(physicsSimulator.getSession());
      RigidBodyBasics originalRigidBody = boxRobot.newIntance(ReferenceFrameTools.constructARootFrame("dummy"));
      ReferenceFrame cloneStationaryFrame = ReferenceFrame.getWorldFrame(); // TODO: Check

      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            baseUI.get3DSceneManager().addModelInstance(new ModelInstance(GDXModelPrimitives.createCoordinateFrame(0.3)));

            rootBody = GDXMultiBodySystemFactories.toYoGDXMultiBodySystem(originalRigidBody,
                                                                          cloneStationaryFrame,
                                                                          boxRobot,
                                                                          mirroredBoxRegistry);
            robotLinkedYoRegistry = yoManager.newLinkedYoRegistry(mirroredBoxRegistry);
            yoManager.linkNewYoVariables();
            for (GDXRigidBody rigidBody : rootBody.subtreeIterable())
            {
               if (rigidBody.getGraphics() != null)
               {
                  baseUI.get3DSceneManager().addModelInstance(rigidBody.getGraphics().getModelInstance());
               }
            }

            VisualDefinition slopeVisual = slopeTerrain.getVisualDefinitions().get(0);
            Box3DDefinition slopeBox = (Box3DDefinition) slopeVisual.getGeometryDefinition();
            slopeModelInstance = GDXModelPrimitives.buildModelInstance(meshBuilder ->
            {
               ColorDefinition diffuseColor = slopeVisual.getMaterialDefinition().getDiffuseColor();
               Color color = GDXTools.toGDX(diffuseColor.getRed(), diffuseColor.getGreen(), diffuseColor.getBlue(), diffuseColor.getAlpha());
               meshBuilder.addBox(slopeBox.getSizeX(), slopeBox.getSizeY(), slopeBox.getSizeZ(), color);
            }, slopeBox.getName());
            baseUI.get3DSceneManager().addModelInstance(slopeModelInstance);
            GDXTools.toGDX(slopeVisual.getOriginPose(), slopeModelInstance.transform);

            physicsSimulator.getSession().startSessionThread();
            baseUI.getImGuiPanelManager().addPanel(physicsSimulator.getControlPanel());
         }

         @Override
         public void render()
         {
            yoManager.update();
            if (robotLinkedYoRegistry.pull() || initialize)
            {
               rootBody.updateFramesRecursively();
               rootBody.updateSubtreeGraphics();
               initialize = false;
            }

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
      new GDXPhysicsDemo();
   }
}
