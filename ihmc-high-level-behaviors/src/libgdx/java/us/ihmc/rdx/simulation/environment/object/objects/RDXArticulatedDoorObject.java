package us.ihmc.rdx.simulation.environment.object.objects;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.behaviors.simulation.door.DoorSceneNodeDefinitions;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObject;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectFactory;
import us.ihmc.rdx.simulation.environment.object.RDXSimpleObject;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.tools.RDXModelLoader;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import static us.ihmc.behaviors.simulation.door.DoorModelParameters.*;

/**
 * One door: frame, panel, and lever.
 * <p>
 * The panel hinge origin sits on the frame. {@link #panelOffsetFromFrame} is where that joint
 * is, and where the panel sits when the hinge angle is zero. The lever hinge origin sits on the
 * panel. {@link #leverOffsetFromPanel} is where that joint is. Both joints are revolute.
 */
public class RDXArticulatedDoorObject extends RDXEnvironmentObject
{
   public static final String NAME = "Articulated Door";
   public static final RDXEnvironmentObjectFactory FACTORY = new RDXEnvironmentObjectFactory(NAME, RDXArticulatedDoorObject.class);

   private final RigidBodyTransform panelOffsetFromFrame = new RigidBodyTransform();
   private final RigidBodyTransform leverOffsetFromPanel = new RigidBodyTransform();
   private final RigidBodyTransform panelVisualOffset = new RigidBodyTransform();
   private final RigidBodyTransform leverOtherSideOffset = new RigidBodyTransform();

   private final RigidBodyTransform hingeRotation = new RigidBodyTransform();
   private final RigidBodyTransform leverRotation = new RigidBodyTransform();
   private final RigidBodyTransform panelToFrame = new RigidBodyTransform();
   private final RigidBodyTransform leverToPanel = new RigidBodyTransform();

   private final ReferenceFrame panelFrame;
   private final ReferenceFrame panelVisualFrame;
   private final ReferenceFrame leverFrame;
   private final ReferenceFrame leverOtherSideFrame;

   private final RDXModelInstance panelInstance;
   private final RDXModelInstance leverInstance;
   private final RDXModelInstance leverOtherSideInstance;

   private final File frameCollisionFile;
   private final File panelCollisionFile;
   private final File leverCollisionFile;
   private final RDXModelInstance frameCollisionInstance;
   private final RDXModelInstance panelCollisionInstance;
   private final RDXModelInstance leverCollisionInstance;
   private final RDXModelInstance leverOtherSideCollisionInstance;

   private static final double HINGE_LIMIT = 1.7;

   private double hingeAngle;
   private double leverAngle;
   private final ImBoolean overrideJointPositions = new ImBoolean(false);
   private final float[] hingeSliderValue = new float[1];
   private final float[] leverSliderValue = new float[1];
   private boolean wasOverridingJointPositions;

   public RDXArticulatedDoorObject()
   {
      super(NAME, FACTORY);

      // Hinge axis is Z, on the frame. Closed panel is this far from the frame origin.
      panelOffsetFromFrame.getTranslation().set(-DOOR_PANEL_THICKNESS, DOOR_PANEL_HINGE_OFFSET, DOOR_PANEL_GROUND_GAP_HEIGHT);
      // Lever axis is X, on the panel. Closed lever is this far from the panel origin.
      leverOffsetFromPanel.getTranslation().set(0.0, DOOR_PANEL_WIDTH - DOOR_OPENER_INSET, DOOR_OPENER_FROM_BOTTOM_OF_PANEL);
      panelVisualOffset.getTranslation().setX(DOOR_PANEL_THICKNESS / 2.0);
      leverOtherSideOffset.appendPitchRotation(Math.PI);
      leverOtherSideOffset.getTranslation().setX(DOOR_PANEL_THICKNESS);

      panelToFrame.set(panelOffsetFromFrame);
      leverToPanel.set(leverOffsetFromPanel);

      panelFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent(pascalCasedName + "PanelFrame" + objectIndex,
                                                                                    placementFrame,
                                                                                    panelToFrame);
      panelVisualFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent(pascalCasedName + "PanelVisualFrame" + objectIndex,
                                                                                         panelFrame,
                                                                                         panelVisualOffset);
      leverFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent(pascalCasedName + "LeverFrame" + objectIndex,
                                                                                   panelFrame,
                                                                                   leverToPanel);
      leverOtherSideFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent(pascalCasedName + "LeverOtherSideFrame" + objectIndex,
                                                                                            leverFrame,
                                                                                            leverOtherSideOffset);

      setRealisticModel(RDXModelLoader.load(DoorSceneNodeDefinitions.DOOR_FRAME_VISUAL_MODEL_FILE_PATH));
      panelInstance = new RDXModelInstance(RDXModelLoader.load(DoorSceneNodeDefinitions.DOOR_PANEL_VISUAL_MODEL_FILE_PATH));
      leverInstance = new RDXModelInstance(RDXModelLoader.load(DoorSceneNodeDefinitions.DOOR_LEVER_HANDLE_VISUAL_MODEL_FILE_PATH));
      leverOtherSideInstance = new RDXModelInstance(RDXModelLoader.load(DoorSceneNodeDefinitions.DOOR_LEVER_HANDLE_VISUAL_MODEL_FILE_PATH));

      // Frame stays the convex hull of DoorFrame.g3dj. Panel and lever use the mesh extracted from their g3dj.
      frameCollisionFile = RDXSimpleObject.findCollisionMeshFile(DoorSceneNodeDefinitions.DOOR_FRAME_VISUAL_MODEL_FILE_PATH);
      panelCollisionFile = RDXSimpleObject.findCollisionMeshFile(DoorSceneNodeDefinitions.DOOR_PANEL_VISUAL_MODEL_FILE_PATH);
      leverCollisionFile = RDXSimpleObject.findCollisionMeshFile(DoorSceneNodeDefinitions.DOOR_LEVER_HANDLE_VISUAL_MODEL_FILE_PATH);
      frameCollisionInstance = collisionInstance(frameCollisionFile, "frame");
      panelCollisionInstance = collisionInstance(panelCollisionFile, "panel");
      leverCollisionInstance = collisionInstance(leverCollisionFile, "lever");
      leverOtherSideCollisionInstance = collisionInstance(leverCollisionFile, "lever");

      double sizeX = 0.25;
      double sizeY = DOOR_PANEL_WIDTH + DOOR_FRAME_PILLAR_SIZE_X;
      double sizeZ = DOOR_FRAME_PILLAR_SIZE_Z;
      setMass(300.0f);
      getCollisionShapeOffset().getTranslation().set(0.0, (sizeY / 2.0) - DOOR_FRAME_HINGE_OFFSET, sizeZ / 2.0);
      getBoundingSphere().setRadius(3.0);
      Box3D collisionBox = new Box3D(sizeX, sizeY, sizeZ);
      setCollisionModel(meshBuilder ->
                        {
                           Color color = LibGDXTools.toLibGDX(YoAppearance.DarkGray());
                           meshBuilder.addBox((float) sizeX, (float) sizeY, (float) sizeZ, color);
                        });
      setCollisionGeometryObject(collisionBox);
      updateRenderablesPoses();
   }

   /** Panel hinge angle about Z, in radians. Zero is closed. */
   public void setHingeAngle(double hingeAngle)
   {
      this.hingeAngle = hingeAngle;
      updateRenderablesPoses();
   }

   public double getHingeAngle()
   {
      return hingeAngle;
   }

   /** Lever angle about X, in radians. Zero is horizontal. */
   public void setLeverAngle(double leverAngle)
   {
      this.leverAngle = leverAngle;
      updateRenderablesPoses();
   }

   public double getLeverAngle()
   {
      return leverAngle;
   }

   /**
    * Simulation and robot contact update this hinge. Ignored while the environment-panel override is checked.
    */
   public void setHingeAngleFromInteraction(double hingeAngle)
   {
      if (!overrideJointPositions.get())
         setHingeAngle(hingeAngle);
   }

   /**
    * Simulation and robot contact update this lever. Ignored while the environment-panel override is checked.
    */
   public void setLeverAngleFromInteraction(double leverAngle)
   {
      if (!overrideJointPositions.get())
         setLeverAngle(leverAngle);
   }

   public boolean isJointPositionOverridden()
   {
      return overrideJointPositions.get();
   }

   @Override
   public void renderImGuiWidgets()
   {
      ImGui.separator();
      ImGui.text("Joint positions");
      ImGui.checkbox("Override joint positions", overrideJointPositions);
      boolean overriding = overrideJointPositions.get();
      if (!overriding || !wasOverridingJointPositions)
      {
         hingeSliderValue[0] = (float) hingeAngle;
         leverSliderValue[0] = (float) leverAngle;
      }
      wasOverridingJointPositions = overriding;

      if (!overriding)
         ImGui.beginDisabled();
      ImGui.sliderFloat("Hinge##articulatedDoorHinge", hingeSliderValue, (float) -HINGE_LIMIT, (float) HINGE_LIMIT, "%.3f rad");
      ImGui.sliderFloat("Lever##articulatedDoorLever", leverSliderValue, (float) -DOOR_LEVER_MAX_TURN_ANGLE, (float) DOOR_LEVER_MAX_TURN_ANGLE, "%.3f rad");
      if (!overriding)
         ImGui.endDisabled();
      else
      {
         if (Math.abs(hingeAngle - hingeSliderValue[0]) > 1.0e-5)
            setHingeAngle(hingeSliderValue[0]);
         if (Math.abs(leverAngle - leverSliderValue[0]) > 1.0e-5)
            setLeverAngle(leverSliderValue[0]);
      }
   }

   /** Hinge origin on the frame. The panel is here when {@link #getHingeAngle()} is zero. */
   public RigidBodyTransform getPanelOffsetFromFrame()
   {
      return panelOffsetFromFrame;
   }

   /** Lever origin on the panel. The lever is here when {@link #getLeverAngle()} is zero. */
   public RigidBodyTransform getLeverOffsetFromPanel()
   {
      return leverOffsetFromPanel;
   }

   /**
    * Frame, panel, and both lever meshes in the world. The frame file is the convex hull of
    * {@code DoorFrame.g3dj}. Panel and lever files are the meshes extracted from their g3dj.
    */
   public List<CollisionMesh> getCollisionMeshes()
   {
      updateJointTransforms();
      List<CollisionMesh> meshes = new ArrayList<>();
      addCollisionMesh(meshes, "frame", frameCollisionFile, realisticModelFrame);
      addCollisionMesh(meshes, "panel", panelCollisionFile, panelVisualFrame);
      addCollisionMesh(meshes, "lever", leverCollisionFile, leverFrame);
      addCollisionMesh(meshes, "leverOtherside", leverCollisionFile, leverOtherSideFrame);
      return meshes;
   }

   @Override
   public void getRealRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      super.getRealRenderables(renderables, pool);
      panelInstance.getRenderables(renderables, pool);
      leverInstance.getRenderables(renderables, pool);
      leverOtherSideInstance.getRenderables(renderables, pool);
   }

   @Override
   public void getCollisionMeshRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      addCollisionRenderable(renderables, pool, frameCollisionInstance);
      addCollisionRenderable(renderables, pool, panelCollisionInstance);
      addCollisionRenderable(renderables, pool, leverCollisionInstance);
      addCollisionRenderable(renderables, pool, leverOtherSideCollisionInstance);
   }

   @Override
   public void updateRenderablesPoses()
   {
      updateJointTransforms();

      super.updateRenderablesPoses();
      place(panelInstance, panelVisualFrame);
      place(leverInstance, leverFrame);
      place(leverOtherSideInstance, leverOtherSideFrame);
      place(frameCollisionInstance, realisticModelFrame);
      place(panelCollisionInstance, panelVisualFrame);
      place(leverCollisionInstance, leverFrame);
      place(leverOtherSideCollisionInstance, leverOtherSideFrame);
   }

   private void updateJointTransforms()
   {
      hingeRotation.setToZero();
      hingeRotation.appendYawRotation(hingeAngle);
      panelToFrame.set(panelOffsetFromFrame);
      panelToFrame.multiply(hingeRotation);

      leverRotation.setToZero();
      leverRotation.appendRollRotation(leverAngle);
      leverToPanel.set(leverOffsetFromPanel);
      leverToPanel.multiply(leverRotation);
   }

   private static RDXModelInstance collisionInstance(File meshFile, String part)
   {
      if (meshFile == null)
      {
         LogTools.warn("Articulated door has no {} collision mesh", part);
         return null;
      }
      return new RDXModelInstance(RDXModelLoader.load(meshFile.getAbsolutePath()));
   }

   private void addCollisionMesh(List<CollisionMesh> meshes, String suffix, File meshFile, ReferenceFrame frame)
   {
      if (meshFile == null)
         return;
      RigidBodyTransform pose = new RigidBodyTransform();
      placementFramePose.setFromReferenceFrame(frame);
      placementFramePose.get(pose);
      meshes.add(new CollisionMesh(suffix, meshFile, pose));
   }

   private static void addCollisionRenderable(Array<Renderable> renderables, Pool<Renderable> pool, RDXModelInstance instance)
   {
      if (instance != null)
         instance.getRenderables(renderables, pool);
   }

   private void place(RDXModelInstance instance, ReferenceFrame frame)
   {
      if (instance == null)
         return;
      placementFramePose.setFromReferenceFrame(frame);
      LibGDXTools.toLibGDX(placementFramePose, tempTransform, instance.transform);
   }

   /** One door-part collision mesh, posed in the world. */
   public static final class CollisionMesh
   {
      private final String suffix;
      private final File meshFile;
      private final Pose3D poseInWorld;

      private CollisionMesh(String suffix, File meshFile, RigidBodyTransform poseInWorld)
      {
         this.suffix = suffix;
         this.meshFile = meshFile;
         this.poseInWorld = new Pose3D(poseInWorld);
      }

      public String getSuffix()
      {
         return suffix;
      }

      public File getMeshFile()
      {
         return meshFile;
      }

      public Pose3D getPoseInWorld()
      {
         return poseInWorld;
      }
   }
}
