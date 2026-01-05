package us.ihmc.rdx.ui.modelViewer;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.files.FileHandle;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import jakarta.xml.bind.JAXBException;
import net.mgsx.gltf.loaders.gltf.GLTFLoader;
import net.mgsx.gltf.scene3d.scene.SceneAsset;
import org.apache.commons.lang3.StringUtils;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotDefinitionTreeRenderer;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.scs2.RDXFrameNodePart;
import us.ihmc.rdx.simulation.scs2.RDXRigidBody;
import us.ihmc.rdx.simulation.scs2.RDXVisualTools;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.tools.RDXModelLoader;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.collidables.RDXRobotCollisionModel;
import us.ihmc.rdx.ui.graphics.RDXMultiBodyGraphic;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.rdx.ui.interactable.RDXInteractableSensor;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.SCS2DefinitionMissingTools;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.urdf.URDFTools;
import us.ihmc.scs2.definition.robot.urdf.items.URDFModel;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.simulation.collision.CollidableHelper;
import us.ihmc.tools.gui.YoAppearanceTools;

import java.awt.*;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Set;
import java.util.TreeMap;

/**
 * Useful for checking robot visual models, collision meshes, hand transforms and more in RDX.
 */
public class RDXRobotModelViewer
{
   private final RDXBaseUI baseUI;
   private final DRCRobotModel robotModel;
   private HumanoidReferenceFrames humanoidReferenceFrames;
   private RDXMultiBodyGraphic robotModelGraphic;
   private RDXMultiBodyGraphic ghostRobotModelGraphic;
   private RDXRobotCollisionModel simulationCollisionModelGraphic;
   private RDXRobotCollisionModel kinematicsCollisionModelGraphic;
   private RDXRobotCollisionModel selectionCollisionModelGraphic;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean showRobot = new ImBoolean(true);
   private final ImBoolean showFrameAfterJoints = new ImBoolean(true);
   private final ImBoolean showGhostRobot = new ImBoolean(true);
   private final ImBoolean showSimulationCollisionModel = new ImBoolean(false);
   private final ImBoolean showKinematicsCollisionModel = new ImBoolean(false);
   private final ImBoolean showSelectionCollisionModel = new ImBoolean(false);
   private final ImBoolean showHandFrames = new ImBoolean(true);
   private final ImBoolean showHandControlFrames = new ImBoolean(true);
   private final ImBoolean showHandGraphicFrames = new ImBoolean(true);
   private final ImBoolean showHandGraphics = new ImBoolean(true);
   private final ImBoolean showHandMultiBodies = new ImBoolean(true);
   private final SideDependentList<RDXRobotModelViewerHand> handViewers = new SideDependentList<>();
   private record RigidBodyVertexAnalysis(RDXRigidBody rdxRigidBody, long numberOfVisualVertices, long numberOfCollisionVertices) { };
   private boolean generatingGraphviz = false;
   private boolean generatedGraphviz = false;
   private final ArrayList<RDXInteractableSensor> interactableSensors = new ArrayList<>();

   public RDXRobotModelViewer(DRCRobotModel robotModel,
                              RobotCollisionModel simulationCollisionModel,
                              RDXRobotModelViewerSensorSupplier... interactableSensorSuppliers)
   {
      this.robotModel = robotModel;

      baseUI = new RDXBaseUI("%s Robot Model Viewer".formatted(StringUtils.capitalize(robotModel.getSimpleRobotName())));

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
            RigidBodyBasics rootBody = fullRobotModel.getElevator();

            robotModel.getDefaultRobotInitialSetup(0.0, 0.0, 0.0, 0.0).initializeFullRobotModel(fullRobotModel);
            humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel, robotModel.getSensorInformation());
            humanoidReferenceFrames.updateFrames();

            // This block is to test to see if create models with other materials modifies the next model
            {
               RobotDefinition ghostRobotDefinition = new RobotDefinition(robotModel.getRobotDefinition());
               MaterialDefinition material = new MaterialDefinition(ColorDefinitions.parse("0xDEE934").derive(0.0, 1.0, 1.0, 0.5));
               SCS2DefinitionMissingTools.forEachRigidBodyDefinitionIncludingFourBars(ghostRobotDefinition.getRootBodyDefinition(),
                                                                                      body -> body.getVisualDefinitions().forEach(visual -> visual.setMaterialDefinition(material)));
               FullHumanoidRobotModel ghostFullRobotModel = robotModel.createFullRobotModel();
               robotModel.getDefaultRobotInitialSetup(0.0, 0.0, 0.0, 2.0).initializeFullRobotModel(ghostFullRobotModel);
               ghostFullRobotModel.updateFrames();
               ghostRobotModelGraphic = new RDXMultiBodyGraphic(robotModel.getSimpleRobotName() + " (IK Preview Ghost)");
               ghostRobotModelGraphic.loadRobotModelAndGraphics(ghostRobotDefinition, ghostFullRobotModel.getElevator());
               ghostRobotModelGraphic.setActive(true);
               ghostRobotModelGraphic.create();
            }

            robotModelGraphic = new RDXMultiBodyGraphic(robotModel.getSimpleRobotName());
            robotModelGraphic.setActive(true);
            robotModelGraphic.loadRobotModelAndGraphics(robotModel.getRobotDefinition(), rootBody, RDXVisualTools.NO_SCALING, true);

            for (RDXRobotModelViewerSensorSupplier interactableSensorSupplier : interactableSensorSuppliers)
               interactableSensors.add(interactableSensorSupplier.build(baseUI, humanoidReferenceFrames, robotModel.getSensorInformation()));

            simulationCollisionModel.setCollidableHelper(new CollidableHelper(), robotModel.getJointMap().getModelName(), "ground");
            simulationCollisionModelGraphic = new RDXRobotCollisionModel(simulationCollisionModel);
            simulationCollisionModelGraphic.create(rootBody, YoAppearanceTools.makeTransparent(YoAppearance.DarkRed(), 0.4));

            RobotCollisionModel kinematicsCollisionModel = robotModel.getHumanoidRobotKinematicsCollisionModel();
            kinematicsCollisionModelGraphic = new RDXRobotCollisionModel(kinematicsCollisionModel);
            kinematicsCollisionModelGraphic.create(rootBody, YoAppearanceTools.makeTransparent(YoAppearance.DarkGreen(), 0.4));

            RobotCollisionModel selectionCollisionModel = robotModel.getHumanoidRobotKinematicsCollisionModel();
            selectionCollisionModelGraphic = new RDXRobotCollisionModel(selectionCollisionModel);
            selectionCollisionModelGraphic.create(rootBody, YoAppearanceTools.makeTransparent(YoAppearance.DarkBlue(), 0.4));

            for (RobotSide side : RobotSide.values)
            {
               handViewers.put(side, new RDXRobotModelViewerHand(side, robotModel, fullRobotModel, baseUI));
            }

            baseUI.getPrimaryScene().addRenderableProvider(this::getRenderables);
            baseUI.getImGuiPanelManager().addPanel("Settings", this::renderImGuiWidgets);
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(this::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(this::process3DViewInput);
            baseUI.getPrimary3DPanel().getCamera3D().changeCameraPosition(2.0, 2.0, 1.5);
            baseUI.getPrimary3DPanel().getCamera3D().setCameraFocusPoint(new Point3D(0.0, 0.0, 0.75));

            {
               FileHandle fileHandle = Gdx.files.internal("models/BoomBox.gltf");
               SceneAsset sceneAsset = new GLTFLoader().load(fileHandle, true);
               ModelInstance modelInstance = new ModelInstance(sceneAsset.scene.model);
               modelInstance.transform.setToRotationRad(1.0f, 0.0f, 0.0f, (float) Math.PI / 2.0f);
               modelInstance.transform.translate(0.2f, 0.7f, 1.0f);
               modelInstance.transform.scale(20.0f, 20.0f, 20.0f);
               baseUI.getPrimaryScene().addModelInstance(modelInstance);
            }

            {
               Model couchModel = RDXModelLoader.load("environmentObjects/couch/Couch.glb");
               Pose3D pose = new Pose3D();
               pose.appendYawRotation(Math.PI / 2.0);
               pose.getPosition().addX(-1.5);
               pose.getPosition().addY(-1.0);
               pose.getPosition().addZ(0.2);
               RDXModelInstance modelInstance = new RDXModelInstance(couchModel);
               modelInstance.setPoseInWorldFrame(pose);
               baseUI.getPrimaryScene().addModelInstance(modelInstance);
            }

            {
               Model model = RDXModelLoader.load("environmentObjects/mustard/mustard.glb");
               Pose3D pose = new Pose3D();
               pose.getPosition().addX(-1.0);
               pose.getPosition().addY(-0.5);
               pose.getPosition().addZ(1.2);
               RDXModelInstance modelInstance = new RDXModelInstance(model);
               modelInstance.setPoseInWorldFrame(pose);
               baseUI.getPrimaryScene().addModelInstance(modelInstance);
               RDXReferenceFrameGraphic frameGraphic = new RDXReferenceFrameGraphic(0.2);
               frameGraphic.setPoseInWorldFrame(pose);
               baseUI.getPrimaryScene().addModelInstance(frameGraphic);
            }

            baseUI.getPrimaryScene().addModelInstance(new RDXModelInstance(RDXModelLoader.load("environmentObjects/flatGround/FlatGround.g3dj")));
         }

         @Override
         public void render()
         {
            humanoidReferenceFrames.updateFrames();
            robotModelGraphic.update();
            ghostRobotModelGraphic.update();
            simulationCollisionModelGraphic.update();
            kinematicsCollisionModelGraphic.update();
            selectionCollisionModelGraphic.update();

            for (RobotSide side : RobotSide.values)
            {
               handViewers.get(side).update();
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
         {
            if (showRobot.get())
            {
               robotModelGraphic.getRenderables(renderables, pool, sceneLevels);
            }
            if (showFrameAfterJoints.get())
            {
               robotModelGraphic.getVisualReferenceFrameRenderables(renderables, pool, sceneLevels);
            }
            if (showGhostRobot.get())
            {
               ghostRobotModelGraphic.getRenderables(renderables, pool, sceneLevels);
            }
            if (showSimulationCollisionModel.get())
            {
               simulationCollisionModelGraphic.getRenderables(renderables, pool);
            }
            if (showKinematicsCollisionModel.get())
            {
               kinematicsCollisionModelGraphic.getRenderables(renderables, pool);
            }
            if (showSelectionCollisionModel.get())
            {
               selectionCollisionModelGraphic.getRenderables(renderables, pool);
            }

            for (RobotSide side : RobotSide.values)
            {
               handViewers.get(side)
                          .getRenderables(renderables,
                                          pool,
                                          sceneLevels,
                                          showHandFrames,
                                          showHandControlFrames,
                                          showHandGraphicFrames,
                                          showHandGraphics,
                                          showHandMultiBodies);
            }
         }

         private void renderImGuiWidgets()
         {
            ImGui.checkbox(labels.get("Show robot"), showRobot);
            ImGui.checkbox(labels.get("Show frame after joints"), showFrameAfterJoints);
            ImGui.checkbox(labels.get("Show ghost robot"), showGhostRobot);
            ImGui.checkbox(labels.get("Show simulation collision model"), showSimulationCollisionModel);
            ImGui.checkbox(labels.get("Show kinematics collision model"), showKinematicsCollisionModel);
            ImGui.checkbox(labels.get("Show selection collision model"), showSelectionCollisionModel);

            ImGui.checkbox(labels.get("Show hand/wrist frame"), showHandFrames);
            ImGui.checkbox(labels.get("Show hand control frame"), showHandControlFrames);
            ImGui.checkbox(labels.get("Show hand graphic frame"), showHandGraphicFrames);
            ImGui.checkbox(labels.get("Show hand graphics"), showHandGraphics);
            ImGui.checkbox(labels.get("Show hand multi-bodies"), showHandMultiBodies);
            for (RobotSide side : RobotSide.values)
            {
               ImGui.text("%s hand:".formatted(side.getPascalCaseName()));
               handViewers.get(side).renderImGuiWidgets();
            }

            if (ImGui.button(labels.get("Perform vertex performance analysis")))
            {
               TreeMap<Long, RigidBodyVertexAnalysis> rigidBodiesByVertexCount = new TreeMap<>();
               for (JointBasics childrenJoint : robotModelGraphic.getMultiBody().getChildrenJoints())
               {
                  for (RigidBodyBasics rigidBodyBasics : MultiBodySystemTools.collectSubtreeSuccessors(childrenJoint))
                  {
                     if (rigidBodyBasics instanceof RDXRigidBody rdxRigidBody)
                     {
                        long numberOfVertices = 0;
                        long numberOfVisualVertices = 0;
                        if (rdxRigidBody.getVisualGraphicsNode() != null)
                        {
                           for (RDXFrameNodePart part : rdxRigidBody.getVisualGraphicsNode().getParts())
                           {
                              numberOfVertices += LibGDXTools.countVertices(part.getModelInstance());
                              numberOfVisualVertices += LibGDXTools.countVertices(part.getModelInstance());
                           }
                        }
                        long numberOfCollisionVertices = 0;
                        if (rdxRigidBody.getCollisionGraphicsNode() != null)
                        {
                           for (RDXFrameNodePart part : rdxRigidBody.getCollisionGraphicsNode().getParts())
                           {
                              numberOfVertices += LibGDXTools.countVertices(part.getModelInstance());
                              numberOfCollisionVertices += LibGDXTools.countVertices(part.getModelInstance());
                           }
                        }
                        rigidBodiesByVertexCount.put(numberOfVertices,
                                                     new RigidBodyVertexAnalysis(rdxRigidBody, numberOfVisualVertices, numberOfCollisionVertices));
                     }
                  }
               }

               for (RigidBodyVertexAnalysis rigidBodyVertexAnalysisEntry : rigidBodiesByVertexCount.values())
               {
                  LogTools.info("{}: Vertices: visual: {} collision: {}",
                                rigidBodyVertexAnalysisEntry.rdxRigidBody().getName(),
                                rigidBodyVertexAnalysisEntry.numberOfVisualVertices(),
                                rigidBodyVertexAnalysisEntry.numberOfCollisionVertices());
               }
            }

            if (ImGui.button(labels.get("Save URDF")))
            {
               try
               {
                  URDFModel urdfModel = URDFTools.toURDFModel(robotModel.getRobotDefinition());
                  FileOutputStream outputStream = new FileOutputStream("out.urdf");
                  URDFTools.saveURDFModel(outputStream, urdfModel);
               }
               catch (FileNotFoundException | JAXBException e)
               {
                  e.printStackTrace();
               }
            }

            if (ImGui.button(labels.get("Render Graphviz Tree")))
            {
               generatingGraphviz = true;
               ThreadTools.startAsDaemon(() ->
               {
                  RobotDefinitionTreeRenderer robotDefinitionTreeRenderer = new RobotDefinitionTreeRenderer(robotModel.getRobotDefinition(), "");
                  generatingGraphviz = false;
                  generatedGraphviz = true;
               }, "RenderGraphvizTree");
            }
            if (generatingGraphviz)
            {
               ImGui.sameLine();
               ImGui.text("Generating ...");
            }
            else if (generatedGraphviz)
            {
               ImGui.sameLine();
               String fileName = robotModel.getRobotDefinition().getName() + ".png";
               if (ImGui.button(labels.get("Open %s".formatted(fileName))))
               {
                  File file = new File(fileName);
                  boolean exists = file.exists();
                  boolean desktopSupported = Desktop.isDesktopSupported();
                  if (exists && desktopSupported)
                  {
                     try
                     {
                        Desktop.getDesktop().open(file);
                     }
                     catch (IOException e)
                     {
                        throw new RuntimeException(e);
                     }
                  }
               }
            }

            for (RobotSide side : RobotSide.values)
            {
               ImGui.text("%s hand:".formatted(side.getPascalCaseName()));
               handViewers.get(side).renderImGuiWidgetsLater();
            }
         }

         public void calculate3DViewPick(ImGui3DViewInput input)
         {
            if (showSimulationCollisionModel.get())
               simulationCollisionModelGraphic.calculate3DViewPick(input);
            if (showKinematicsCollisionModel.get())
               kinematicsCollisionModelGraphic.calculate3DViewPick(input);
            if (showSelectionCollisionModel.get())
               selectionCollisionModelGraphic.calculate3DViewPick(input);
         }

         public void process3DViewInput(ImGui3DViewInput input)
         {
            if (showSimulationCollisionModel.get())
               simulationCollisionModelGraphic.process3DViewInput(input);
            if (showKinematicsCollisionModel.get())
               kinematicsCollisionModelGraphic.process3DViewInput(input);
            if (showSelectionCollisionModel.get())
               selectionCollisionModelGraphic.process3DViewInput(input);
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
            robotModelGraphic.destroy();
            ghostRobotModelGraphic.destroy();
         }
      });
   }
}
