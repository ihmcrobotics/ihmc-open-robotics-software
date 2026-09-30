package us.ihmc.rdx.simulation.environment.object.objects;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import org.bytedeco.javacpp.DoublePointer;
import us.ihmc.behaviors.simulation.door.DoorSceneNodeDefinitions;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.scs2.simulation.mujoco.Mujoco;
import us.ihmc.scs2.simulation.mujoco.Mujoco.mjData;
import us.ihmc.scs2.simulation.mujoco.Mujoco.mjModel;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObject;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectFactory;
import us.ihmc.rdx.simulation.environment.object.RDXSimpleObject;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.tools.RDXModelLoader;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static us.ihmc.behaviors.simulation.door.DoorModelParameters.*;

/**
 * One door: frame, panel, and lever.
 * <p>
 * The panel hinge origin sits on the frame. {@link #panelOffsetFromFrame} is where that joint
 * is, and where the panel sits when the hinge angle is zero. The lever hinge origin sits on the
 * panel. {@link #leverOffsetFromPanel} is where that joint is. Both joints are revolute.
 * <p>
 * Frame, panel, and each lever have their own collision. The frame is two boxes, one on each
 * side of the panel. A joint move updates the panel and lever visuals and their collision meshes.
 * Collision shapes are the transparent overlay drawn while the door is selected.
 */
public class RDXArticulatedDoorObject extends RDXEnvironmentObject
{
   public static final String NAME = "Articulated Door";
   public static final RDXEnvironmentObjectFactory FACTORY = new RDXEnvironmentObjectFactory(NAME, RDXArticulatedDoorObject.class);

   /** Each frame jamb, the dimension beside the panel. */
   private static final double FRAME_JAMB_WIDTH = 0.10;
   /** Each frame jamb, the dimension through the wall. */
   private static final double FRAME_JAMB_DEPTH = 0.10;

   private final RigidBodyTransform panelOffsetFromFrame = new RigidBodyTransform();
   private final RigidBodyTransform leverOffsetFromPanel = new RigidBodyTransform();
   private final RigidBodyTransform panelVisualOffset = new RigidBodyTransform();
   private final RigidBodyTransform leverOtherSideOffset = new RigidBodyTransform();
   /** Front lever mesh: door_handle.glb yawed onto the arm, rolled 180 about X, one panel thickness out. */
   private final RigidBodyTransform leverMeshOffset = new RigidBodyTransform();
   /** Other-side lever mesh. Same orientation, one panel thickness the other way. */
   private final RigidBodyTransform leverOtherSideMeshOffset = new RigidBodyTransform();

   private final RigidBodyTransform hingeRotation = new RigidBodyTransform();
   private final RigidBodyTransform leverRotation = new RigidBodyTransform();
   private final RigidBodyTransform panelToFrame = new RigidBodyTransform();
   private final RigidBodyTransform leverToPanel = new RigidBodyTransform();

   private final ReferenceFrame panelFrame;
   private final ReferenceFrame panelVisualFrame;
   private final ReferenceFrame leverFrame;
   private final ReferenceFrame leverMeshFrame;
   private final ReferenceFrame leverOtherSideFrame;
   private final ReferenceFrame leverOtherSideMeshFrame;

   private final RDXModelInstance panelInstance;
   private final RDXModelInstance leverInstance;
   private final RDXModelInstance leverOtherSideInstance;

   private final ReferenceFrame hingeJambFrame;
   private final ReferenceFrame latchJambFrame;
   private final RDXModelInstance hingeJambInstance;
   private final RDXModelInstance latchJambInstance;

   private final File panelCollisionFile;
   private final File leverCollisionFile;
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
      leverMeshOffset.appendYawRotation(-Math.PI / 2.0);
      leverMeshOffset.prependRollRotation(Math.PI);
      leverMeshOffset.getTranslation().setX(DOOR_PANEL_THICKNESS);
      leverOtherSideMeshOffset.appendYawRotation(-Math.PI / 2.0);
      leverOtherSideMeshOffset.prependRollRotation(Math.PI);
      leverOtherSideMeshOffset.getTranslation().setX(-DOOR_PANEL_THICKNESS);

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
      leverMeshFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(pascalCasedName + "LeverMeshFrame" + objectIndex,
                                                                                         leverFrame,
                                                                                         leverMeshOffset);
      leverOtherSideFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent(pascalCasedName + "LeverOtherSideFrame" + objectIndex,
                                                                                            leverFrame,
                                                                                            leverOtherSideOffset);
      leverOtherSideMeshFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(pascalCasedName + "LeverOtherSideMeshFrame" + objectIndex,
                                                                                                  leverOtherSideFrame,
                                                                                                  leverOtherSideMeshOffset);

      setRealisticModel(RDXModelLoader.load(DoorSceneNodeDefinitions.DOOR_FRAME_VISUAL_MODEL_FILE_PATH));
      panelInstance = new RDXModelInstance(RDXModelLoader.load(DoorSceneNodeDefinitions.DOOR_PANEL_VISUAL_MODEL_FILE_PATH));
      leverInstance = new RDXModelInstance(RDXModelLoader.load(DoorSceneNodeDefinitions.DOOR_LEVER_HANDLE_VISUAL_MODEL_FILE_PATH));
      leverOtherSideInstance = new RDXModelInstance(RDXModelLoader.load(DoorSceneNodeDefinitions.DOOR_LEVER_HANDLE_VISUAL_MODEL_FILE_PATH));

      // Two jambs beside the panel. Panel and lever keep the meshes extracted from their g3dj.
      RigidBodyTransform hingeJambOffset = new RigidBodyTransform();
      RigidBodyTransform latchJambOffset = new RigidBodyTransform();
      double jambCenterZ = DOOR_PANEL_GROUND_GAP_HEIGHT + DOOR_PANEL_HEIGHT / 2.0;
      hingeJambOffset.getTranslation().set(0.0, hingeJambCenterY(), jambCenterZ);
      latchJambOffset.getTranslation().set(0.0, latchJambCenterY(), jambCenterZ);
      hingeJambFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(pascalCasedName + "HingeJambFrame" + objectIndex,
                                                                                         realisticModelFrame,
                                                                                         hingeJambOffset);
      latchJambFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(pascalCasedName + "LatchJambFrame" + objectIndex,
                                                                                         realisticModelFrame,
                                                                                         latchJambOffset);
      Model jambModel = RDXModelBuilder.buildModel(meshBuilder ->
      {
         Color color = LibGDXTools.toLibGDX(YoAppearance.LightSkyBlue());
         meshBuilder.addBox((float) FRAME_JAMB_DEPTH, (float) FRAME_JAMB_WIDTH, (float) DOOR_PANEL_HEIGHT, color);
      }, pascalCasedName + "FrameJamb" + objectIndex);
      LibGDXTools.setOpacity(jambModel, 0.4f);
      hingeJambInstance = new RDXModelInstance(jambModel);
      latchJambInstance = new RDXModelInstance(jambModel);
      hingeJambInstance.setOpacity(0.4f);
      latchJambInstance.setOpacity(0.4f);

      panelCollisionFile = RDXSimpleObject.findCollisionMeshFile(DoorSceneNodeDefinitions.DOOR_PANEL_VISUAL_MODEL_FILE_PATH);
      leverCollisionFile = RDXSimpleObject.findCollisionMeshFile(DoorSceneNodeDefinitions.DOOR_LEVER_HANDLE_VISUAL_MODEL_FILE_PATH);
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
      this.hingeAngle = Math.min(HINGE_LIMIT, Math.max(0.0, hingeAngle));
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
      ImGui.sliderFloat("Hinge##articulatedDoorHinge", hingeSliderValue, 0.0f, (float) HINGE_LIMIT, "%.3f rad");
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

   /** Panel mesh origin relative to the hinge frame. Shared by the panel visual and the panel collision. */
   public RigidBodyTransform getPanelVisualOffset()
   {
      return panelVisualOffset;
   }

   /** Other-side lever mesh relative to the lever joint. Shared by that visual and its collision. */
   public RigidBodyTransform getLeverOtherSideOffset()
   {
      return leverOtherSideOffset;
   }

   /** Front lever mesh pose in the lever joint. */
   public RigidBodyTransform getLeverMeshOffset()
   {
      return leverMeshOffset;
   }

   /** Other-side lever mesh pose in the other-side frame. */
   public RigidBodyTransform getLeverOtherSideMeshOffset()
   {
      return leverOtherSideMeshOffset;
   }

   /** Panel and both lever meshes in the world. The frame jambs are boxes, not a mesh. */
   public List<CollisionMesh> getCollisionMeshes()
   {
      updateJointTransforms();
      List<CollisionMesh> meshes = new ArrayList<>();
      addCollisionMesh(meshes, "panel", panelCollisionFile, panelVisualFrame);
      addCollisionMesh(meshes, "lever", leverCollisionFile, leverMeshFrame);
      addCollisionMesh(meshes, "leverOtherside", leverCollisionFile, leverOtherSideMeshFrame);
      return meshes;
   }

   /** Center of the hinge-side jamb, in the frame, just outside the panel's hinge edge. */
   private static double hingeJambCenterY()
   {
      return DOOR_PANEL_HINGE_OFFSET - FRAME_JAMB_WIDTH / 2.0;
   }

   /** Center of the latch-side jamb, in the frame, just outside the panel's free edge. */
   private static double latchJambCenterY()
   {
      return DOOR_PANEL_HINGE_OFFSET + DOOR_PANEL_WIDTH + FRAME_JAMB_WIDTH / 2.0;
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
      addCollisionRenderable(renderables, pool, hingeJambInstance);
      addCollisionRenderable(renderables, pool, latchJambInstance);
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
      place(leverInstance, leverMeshFrame);
      place(leverOtherSideInstance, leverOtherSideMeshFrame);
      place(hingeJambInstance, hingeJambFrame);
      place(latchJambInstance, latchJambFrame);
      place(panelCollisionInstance, panelVisualFrame);
      place(leverCollisionInstance, leverMeshFrame);
      place(leverOtherSideCollisionInstance, leverOtherSideMeshFrame);
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

      // The frames keep a copy of these transforms. update() is what publishes a joint change
      // into that copy; without it the panel and lever stay at the closed pose.
      panelFrame.update();
      panelVisualFrame.update();
      leverFrame.update();
      leverMeshFrame.update();
      leverOtherSideFrame.update();
      leverOtherSideMeshFrame.update();
   }

   private static RDXModelInstance collisionInstance(File meshFile, String part)
   {
      if (meshFile == null)
      {
         LogTools.warn("Articulated door has no {} collision mesh", part);
         return null;
      }
      RDXModelInstance instance = new RDXModelInstance(RDXModelLoader.load(meshFile.getAbsolutePath()));
      instance.setDiffuseColor(LibGDXTools.toLibGDX(YoAppearance.LightSkyBlue()));
      instance.setOpacity(0.4f);
      return instance;
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

   /**
    * Physics snapshot of this placed door. The environment panel's Articulated Door button creates
    * this object; the simulation compiles this snapshot, it does not create a second door.
    */
   public Dynamics captureDynamics()
   {
      return Dynamics.from(this);
   }

   public static final class Dynamics
   {
      /** Lever angle, either direction, at which the panel is free to swing. */
      public static final double UNLATCH_LEVER_ANGLE = 0.5 * DOOR_LEVER_MAX_TURN_ANGLE;
      /** How close to closed the panel must be before a released lever catches it. */
      public static final double LATCH_CATCH_ANGLE = 0.05;

      private static final double PANEL_DENSITY = 400.0;
      private static final double LEVER_DENSITY = 800.0;
      private static final double HINGE_DAMPING = 2.0;
      private static final double LEVER_DAMPING = 0.1;
      private static final double LEVER_STIFFNESS = DOOR_LEVER_MAX_TORQUE / DOOR_LEVER_MAX_TURN_ANGLE;

      private final String baseName;
      private final String panelMeshPath;
      private final String leverMeshPath;
      private final Pose3D rootPose;
      private final RigidBodyTransform panelOffset;
      private final RigidBodyTransform leverOffset;
      private final RigidBodyTransform panelVisualOffset;
      private final RigidBodyTransform leverOtherSideOffset;
      private final RigidBodyTransform leverMeshOffset;
      private final RigidBodyTransform leverOtherSideMeshOffset;
      private final double hingeAngle;
      private final double leverAngle;
      private final boolean overrideJoints;

      private Dynamics(String baseName,
                     String panelMeshPath,
                     String leverMeshPath,
                     Pose3D rootPose,
                     RigidBodyTransform panelOffset,
                     RigidBodyTransform leverOffset,
                     RigidBodyTransform panelVisualOffset,
                     RigidBodyTransform leverOtherSideOffset,
                     RigidBodyTransform leverMeshOffset,
                     RigidBodyTransform leverOtherSideMeshOffset,
                     double hingeAngle,
                     double leverAngle,
                     boolean overrideJoints)
      {
         this.baseName = baseName;
         this.panelMeshPath = panelMeshPath;
         this.leverMeshPath = leverMeshPath;
         this.rootPose = rootPose;
         this.panelOffset = panelOffset;
         this.leverOffset = leverOffset;
         this.panelVisualOffset = panelVisualOffset;
         this.leverOtherSideOffset = leverOtherSideOffset;
         this.leverMeshOffset = leverMeshOffset;
         this.leverOtherSideMeshOffset = leverOtherSideMeshOffset;
         this.hingeAngle = hingeAngle;
         this.leverAngle = leverAngle;
         this.overrideJoints = overrideJoints;
      }

      public static Dynamics from(RDXArticulatedDoorObject door)
      {
         String baseName = "rdx_" + door.getPascalCasedName() + "_" + door.getObjectIndex();
         String panelMeshPath = null;
         String leverMeshPath = null;
         for (CollisionMesh part : door.getCollisionMeshes())
         {
            if ("panel".equals(part.getSuffix()))
               panelMeshPath = part.getMeshFile().getAbsolutePath();
            else if ("lever".equals(part.getSuffix()))
               leverMeshPath = part.getMeshFile().getAbsolutePath();
         }
         return new Dynamics(baseName,
                           panelMeshPath,
                           leverMeshPath,
                           new Pose3D(door.getObjectTransform()),
                           new RigidBodyTransform(door.getPanelOffsetFromFrame()),
                           new RigidBodyTransform(door.getLeverOffsetFromPanel()),
                           new RigidBodyTransform(door.getPanelVisualOffset()),
                           new RigidBodyTransform(door.getLeverOtherSideOffset()),
                           new RigidBodyTransform(door.getLeverMeshOffset()),
                           new RigidBodyTransform(door.getLeverOtherSideMeshOffset()),
                           door.getHingeAngle(),
                           door.getLeverAngle(),
                           door.isJointPositionOverridden());
      }

      public String getBaseName()
      {
         return baseName;
      }

      public String getPanelMeshPath()
      {
         return panelMeshPath;
      }

      public String getLeverMeshPath()
      {
         return leverMeshPath;
      }

      public String structureKey()
      {
         return baseName + "|door|jambs|" + panelMeshPath + "|" + leverMeshPath;
      }

      public String panelMeshFileName()
      {
         return baseName + "_panel.stl";
      }

      public String leverMeshFileName()
      {
         return baseName + "_lever.stl";
      }

      public void appendAssets(StringBuilder assets)
      {
         appendMeshAsset(assets, baseName + "_panel", panelMeshFileName(), panelMeshPath);
         appendMeshAsset(assets, baseName + "_lever", leverMeshFileName(), leverMeshPath);
      }

      public void appendBodies(StringBuilder bodies)
      {
         Quaternion rootRotation = new Quaternion(rootPose.getOrientation());
         bodies.append("    <body name=\"").append(baseName).append("_frame\" ")
               .append(pose(rootPose.getX(), rootPose.getY(), rootPose.getZ(), rootRotation))
               .append(">\n");
         appendFrameJamb(bodies, baseName + "_hingeJamb", hingeJambCenterY());
         appendFrameJamb(bodies, baseName + "_latchJamb", latchJambCenterY());
         bodies.append("      <body name=\"").append(baseName).append("_panel\" ")
               .append(transformPose(panelOffset)).append(">\n");
         bodies.append("        <joint name=\"").append(baseName).append("_hinge\" type=\"hinge\" axis=\"0 0 1\" limited=\"true\" range=\"")
               .append(fmt(0.0)).append(' ').append(fmt(HINGE_LIMIT))
               .append("\" damping=\"").append(fmt(HINGE_DAMPING)).append("\" armature=\"0.01\"/>\n");
         appendGeom(bodies, baseName + "_panel", baseName + "_panel_mesh", panelVisualOffset, panelMeshPath, PANEL_DENSITY);
         bodies.append("        <body name=\"").append(baseName).append("_lever\" ")
               .append(transformPose(leverOffset)).append(">\n");
         bodies.append("          <joint name=\"").append(baseName).append("_lever\" type=\"hinge\" axis=\"1 0 0\" limited=\"true\" range=\"")
               .append(fmt(-DOOR_LEVER_MAX_TURN_ANGLE)).append(' ').append(fmt(DOOR_LEVER_MAX_TURN_ANGLE))
               .append("\" damping=\"").append(fmt(LEVER_DAMPING))
               .append("\" stiffness=\"").append(fmt(LEVER_STIFFNESS))
               .append("\" armature=\"0.001\"/>\n");
         appendGeom(bodies, baseName + "_lever", baseName + "_lever_mesh", leverMeshOffset, leverMeshPath, LEVER_DENSITY);
         bodies.append("          <body name=\"").append(baseName).append("_leverOtherside\" ")
               .append(transformPose(leverOtherSideOffset)).append(">\n");
         appendGeom(bodies, baseName + "_leverOtherside", baseName + "_lever_mesh", leverOtherSideMeshOffset, leverMeshPath, LEVER_DENSITY);
         bodies.append("          </body>\n");
         bodies.append("        </body>\n");
         bodies.append("      </body>\n");
         bodies.append("    </body>\n");
      }

      public void appendExcludes(StringBuilder contact)
      {
         contact.append("    <exclude body1=\"").append(baseName).append("_panel\" body2=\"")
                .append(baseName).append("_leverOtherside\"/>\n");
         contact.append("    <exclude body1=\"").append(baseName).append("_frame\" body2=\"")
                .append(baseName).append("_leverOtherside\"/>\n");
      }

      /**
       * Writes the welded frame pose, then either forces the slider angles or lets contact and
       * gravity move the joints. Returns the hinge and lever angles after that update.
       */
      public double[] syncJoints(mjModel model, mjData data, JointIds ids, boolean seedFromCommand)
      {
         writeRootPose(model, ids);
         if (seedFromCommand || overrideJoints)
         {
            data.qpos().put(ids.hingeQpos, hingeAngle);
            data.qpos().put(ids.leverQpos, leverAngle);
            data.qvel().put(ids.hingeDof, 0.0);
            data.qvel().put(ids.leverDof, 0.0);
         }

         double hinge = data.qpos().get(ids.hingeQpos);
         double lever = data.qpos().get(ids.leverQpos);
         DoublePointer range = model.jnt_range();
         if (overrideJoints)
         {
            setHingeRange(range, ids.hingeJoint, 0.0, HINGE_LIMIT);
            return new double[] {hingeAngle, leverAngle};
         }

         boolean unlatched = Math.abs(lever) >= UNLATCH_LEVER_ANGLE;
         boolean caught = Math.abs(hinge) <= LATCH_CATCH_ANGLE;
         if (!unlatched && caught)
         {
            setHingeRange(range, ids.hingeJoint, 0.0, 1.0e-4);
            data.qpos().put(ids.hingeQpos, 0.0);
            data.qvel().put(ids.hingeDof, 0.0);
            hinge = 0.0;
         }
         else
         {
            setHingeRange(range, ids.hingeJoint, 0.0, HINGE_LIMIT);
         }
         return new double[] {hinge, lever};
      }

      private void writeRootPose(mjModel model, JointIds ids)
      {
         Quaternion rotation = new Quaternion(rootPose.getOrientation());
         DoublePointer position = model.body_pos();
         position.put(ids.frameBody * 3L, rootPose.getX());
         position.put(ids.frameBody * 3L + 1, rootPose.getY());
         position.put(ids.frameBody * 3L + 2, rootPose.getZ());
         DoublePointer quat = model.body_quat();
         quat.put(ids.frameBody * 4L, rotation.getS());
         quat.put(ids.frameBody * 4L + 1, rotation.getX());
         quat.put(ids.frameBody * 4L + 2, rotation.getY());
         quat.put(ids.frameBody * 4L + 3, rotation.getZ());
      }

      private static void appendFrameJamb(StringBuilder bodies, String name, double centerY)
      {
         double centerZ = DOOR_PANEL_GROUND_GAP_HEIGHT + DOOR_PANEL_HEIGHT / 2.0;
         bodies.append("          <geom class=\"terrain\" name=\"").append(name)
               .append("\" type=\"box\" pos=\"0 ").append(fmt(centerY)).append(' ').append(fmt(centerZ))
               .append("\" size=\"").append(fmt(FRAME_JAMB_DEPTH / 2.0)).append(' ')
               .append(fmt(FRAME_JAMB_WIDTH / 2.0)).append(' ')
               .append(fmt(DOOR_PANEL_HEIGHT / 2.0)).append("\"/>\n");
      }

      private static void appendMeshAsset(StringBuilder assets, String meshName, String fileName, String sourcePath)
      {
         if (sourcePath == null)
            return;
         assets.append("    <mesh name=\"").append(meshName).append("_mesh\" file=\"").append(fileName).append("\"/>\n");
      }

      private static void appendGeom(StringBuilder bodies, String geomName, String meshName, RigidBodyTransform localPose, String sourcePath, double density)
      {
         if (sourcePath == null)
            return;
         bodies.append("          <geom class=\"terrain\" name=\"").append(geomName).append("\" type=\"mesh\" mesh=\"").append(meshName).append("\"");
         if (localPose != null)
            bodies.append(' ').append(transformPose(localPose));
         if (density > 0.0)
            bodies.append(" density=\"").append(fmt(density)).append("\"");
         bodies.append("/>\n");
      }

      private static String transformPose(RigidBodyTransform transform)
      {
         Quaternion rotation = new Quaternion(transform.getRotation());
         return pose(transform.getTranslation().getX(),
                     transform.getTranslation().getY(),
                     transform.getTranslation().getZ(),
                     rotation);
      }

      private static String pose(double x, double y, double z, Quaternion rotation)
      {
         return "pos=\"" + fmt(x) + " " + fmt(y) + " " + fmt(z) + "\" quat=\""
                + fmt(rotation.getS()) + " " + fmt(rotation.getX()) + " "
                + fmt(rotation.getY()) + " " + fmt(rotation.getZ()) + "\"";
      }

      private static void setHingeRange(DoublePointer range, int jointId, double lower, double upper)
      {
         range.put(jointId * 2L, lower);
         range.put(jointId * 2L + 1, upper);
      }

      private static String fmt(double value)
      {
         return String.format(Locale.US, "%.8g", value);
      }
   }

   /** MuJoCo joint addresses for one placed articulated door. */
   public static final class JointIds
   {
      private final int frameBody;
      private final int hingeJoint;
      private final int leverJoint;
      private final int hingeQpos;
      private final int leverQpos;
      private final int hingeDof;
      private final int leverDof;

      private JointIds(int frameBody, int hingeJoint, int leverJoint, int hingeQpos, int leverQpos, int hingeDof, int leverDof)
      {
         this.frameBody = frameBody;
         this.hingeJoint = hingeJoint;
         this.leverJoint = leverJoint;
         this.hingeQpos = hingeQpos;
         this.leverQpos = leverQpos;
         this.hingeDof = hingeDof;
         this.leverDof = leverDof;
      }

      public static JointIds resolve(mjModel model, String baseName)
      {
         int frameBody = Mujoco.mj_name2id(model, Mujoco.mjOBJ_BODY, baseName + "_frame");
         int hingeJoint = Mujoco.mj_name2id(model, Mujoco.mjOBJ_JOINT, baseName + "_hinge");
         int leverJoint = Mujoco.mj_name2id(model, Mujoco.mjOBJ_JOINT, baseName + "_lever");
         if (frameBody < 0 || hingeJoint < 0 || leverJoint < 0)
            return null;
         return new JointIds(frameBody,
                             hingeJoint,
                             leverJoint,
                             model.jnt_qposadr().get(hingeJoint),
                             model.jnt_qposadr().get(leverJoint),
                             model.jnt_dofadr().get(hingeJoint),
                             model.jnt_dofadr().get(leverJoint));
      }
   }
}
