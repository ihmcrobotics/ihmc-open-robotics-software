package us.ihmc.rdx.ui.affordances;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.flag.ImGuiMouseButton;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameBox3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.PlannedFootstep;
import us.ihmc.footstepPlanning.PlannedFootstepReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.rdx.RDX3DSituatedText;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.input.ImGui3DViewPickResult;
import us.ihmc.rdx.simulation.scs2.RDXVisualTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.tools.RDXModelLoader;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.rdx.vr.RDXVRDragData;
import us.ihmc.rdx.vr.RDXVRPickResult;
import us.ihmc.robotics.interaction.MouseCollidable;
import us.ihmc.rdx.ui.graphics.RDXFootstepGraphic;
import us.ihmc.rdx.visualizers.RDXPolynomial;
import us.ihmc.robotics.interaction.PointCollidable;
import us.ihmc.robotics.math.trajectories.core.Polynomial;
import us.ihmc.robotics.math.trajectories.interfaces.PolynomialReadOnly;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.tools.Timer;

import java.util.*;
import java.util.concurrent.atomic.AtomicReference;

public class RDXInteractableFootstep
{
   // Intended to reuse text renderables, as they are relatively expensive to create
   private static final Map<String, RDX3DSituatedText> textRenderablesMap = new HashMap<>();
   // Measured footstep.fbx in Blender
   private static final double FOOTSTEP_GRAPHIC_HEIGHT = 0.0287;
   private static final double FOOTSTEP_GRAPHIC_LENGTH = 0.2497;
   private static final double FOOTSTEP_GRAPHIC_WIDTH = 0.1185;
   private static final double FOOTSTEP_GRAPHIC_SOLE_OFFSET_X = 0.01;
   private static final double FOOTSTEP_GRAPHIC_SOLE_OFFSET_Y = 0.0;
   private static final double FOOTSTEP_GRAPHIC_SOLE_OFFSET_Z = FOOTSTEP_GRAPHIC_HEIGHT / 2.0;
   private RDX3DSituatedText footstepIndexText;
   private ModelInstance footstepModelInstance;
   private RDXSelectablePose3DGizmo selectablePose3DGizmo;
   private final FrameBox3D selectionCollisionBox;
   private final MutableReferenceFrame collisionBoxFrame;
   private final MouseCollidable mouseCollidable;
   private final PointCollidable pointCollidable;
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private boolean isMouseHovering;
   private boolean isClickedOn;
   private final FramePose3D textFramePose = new FramePose3D();
   private final Timer timerFlashingFootsteps = new Timer();
   private boolean flashingFootStepsColorHigh = false;
   private final ImGui3DViewPickResult pickResult = new ImGui3DViewPickResult();

   private final List<ModelInstance> trajectoryWaypointModel = new ArrayList<>();
   private final RDXPolynomial swingTrajectoryModel = new RDXPolynomial(0.03, 25);

   private final SideDependentList<ConvexPolygon2D> defaultPolygons;
   private final AtomicReference<Pair<PlannedFootstep, EnumMap<Axis3D, List<PolynomialReadOnly>>>> plannedFootstepInput = new AtomicReference<>(null);
   private final PlannedFootstep plannedFootstepInternal;
   private final EnumMap<Axis3D, List<PolynomialReadOnly>> plannedFootstepTrajectory = new EnumMap<>(Axis3D.class);

   private boolean wasPoseUpdated = false;

   private final SideDependentList<RDXVRPickResult> vrPickResult = new SideDependentList<>(RDXVRPickResult::new);
   private final SideDependentList<Boolean> isVRHovering = new SideDependentList<>(false, false);
   private int index;

   public RDXInteractableFootstep(RDXBaseUI baseUI, RobotSide footstepSide, int index, SideDependentList<ConvexPolygon2D> defaultPolygons)
   {
      this.defaultPolygons = defaultPolygons;
      plannedFootstepInternal = new PlannedFootstep(footstepSide);

      if (footstepSide.equals(RobotSide.LEFT))
      {
         footstepModelInstance = new RDXModelInstance(RDXModelLoader.load("models/footsteps/footstep_left.g3dj"));
      }
      else if (footstepSide.equals(RobotSide.RIGHT))
      {
         footstepModelInstance = new RDXModelInstance(RDXModelLoader.load("models/footsteps/footstep_right.g3dj"));
      }

      selectablePose3DGizmo = new RDXSelectablePose3DGizmo();
      selectablePose3DGizmo.create(baseUI.getPrimary3DPanel());
      selectionCollisionBox = new FrameBox3D(selectablePose3DGizmo.getPoseGizmo().getGizmoFrame());
      selectionCollisionBox.getSize().set(FOOTSTEP_GRAPHIC_LENGTH, FOOTSTEP_GRAPHIC_WIDTH, FOOTSTEP_GRAPHIC_HEIGHT);
      selectionCollisionBox.getPose().getTranslation().add(FOOTSTEP_GRAPHIC_SOLE_OFFSET_X, FOOTSTEP_GRAPHIC_SOLE_OFFSET_Y, FOOTSTEP_GRAPHIC_SOLE_OFFSET_Z);
      collisionBoxFrame = new MutableReferenceFrame("collisionBoxFrame", selectablePose3DGizmo.getPoseGizmo().getGizmoFrame());
      collisionBoxFrame.update(transformToParent -> transformToParent.set(selectionCollisionBox.getPose()));
      mouseCollidable = new MouseCollidable(selectionCollisionBox);
      pointCollidable = new PointCollidable(selectionCollisionBox);

      updateFootstepIndexText(index);
   }

   /**
    * Adds the footstep index text to the footstep (e.g. "R0" or "L4")
    *
    * @param index Index of the footstep within the array of footsteps being planned - determines the number displayed
    */
   public void updateFootstepIndexText(int index)
   {
      this.index = index;
      String text = plannedFootstepInternal.getRobotSide().getSideNameFirstLetter() + this.index;
      if (!textRenderablesMap.containsKey(text))
      {
         float textHeight = 0.08f;
         footstepIndexText = new RDX3DSituatedText(text, textHeight);
         textRenderablesMap.put(text, footstepIndexText);
      }
      else
      {
         footstepIndexText = textRenderablesMap.get(text);
      }
   }

   public void reset()
   {
      getFootstepModelInstance().transform.val[Matrix4.M03] = Float.NaN;
      plannedFootstepInternal.reset();
      plannedFootstepTrajectory.clear();
      updateTrajectoryModel(plannedFootstepInternal, plannedFootstepTrajectory);
   }

   public PlannedFootstepReadOnly getPlannedFootstep()
   {
      return plannedFootstepInternal;
   }

   public void updateFromPlannedStep(RDXBaseUI baseUI, PlannedFootstep plannedFootstep, EnumMap<Axis3D, List<PolynomialReadOnly>> swingTrajectory, int footstepIndex)
   {
      plannedFootstepInput.set(null);
      plannedFootstepInternal.set(plannedFootstep);
      plannedFootstepInternal.limitFootholdVertices();
      plannedFootstepTrajectory.clear();
      if (swingTrajectory != null)
         swingTrajectory.keySet().forEach(key -> plannedFootstepTrajectory.put(key, RDXVisualTools.copyPolynomialList(swingTrajectory.get(key))));

      boolean setCustomFoothold = plannedFootstepInternal.hasFoothold();
      if (setCustomFoothold)
      {
         if (defaultPolygons == null || plannedFootstep.getFoothold().epsilonEquals(defaultPolygons.get(plannedFootstep.getRobotSide()), 1e-3))
            setCustomFoothold = false;
      }

      if (setCustomFoothold)
      {
         Color regionColor = RDXFootstepGraphic.FOOT_COLORS.get(plannedFootstep.getRobotSide());
         List<Point3DReadOnly> points = new ArrayList<>();
         for (int i = 0; i < plannedFootstep.getFoothold().getNumberOfVertices(); i++)
         {
            points.add(new Point3D(plannedFootstep.getFoothold().getVertex(i)));
         }
         footstepModelInstance = RDXModelBuilder.createLinedPolygon(points, 0.05, regionColor, true);
      }
      else
      {
         if (plannedFootstepInternal.getRobotSide().equals(RobotSide.LEFT))
         {
            footstepModelInstance = new RDXModelInstance(RDXModelLoader.load("models/footsteps/footstep_left.g3dj"));
         }
         else if (plannedFootstepInternal.getRobotSide().equals(RobotSide.RIGHT))
         {
            footstepModelInstance = new RDXModelInstance(RDXModelLoader.load("models/footsteps/footstep_right.g3dj"));
         }
      }

      selectablePose3DGizmo = new RDXSelectablePose3DGizmo();
      selectablePose3DGizmo.create(baseUI.getPrimary3DPanel());

      updateFootstepIndexText(footstepIndex);

      updatePose(plannedFootstep.getFootstepPose());

      updateTrajectoryModel(plannedFootstepInternal, plannedFootstepTrajectory);
   }

   public void updatePlannedTrajectory(Pair<PlannedFootstep, EnumMap<Axis3D, List<PolynomialReadOnly>>> other)
   {
      plannedFootstepInput.set(other);
   }

   private void updatePlannedTrajectoryInternal(PlannedFootstep other, EnumMap<Axis3D, List<PolynomialReadOnly>> otherTrajectory)
   {
      plannedFootstepInternal.setTrajectoryType(other.getTrajectoryType());
      plannedFootstepInternal.getCustomWaypointProportions().clear();
      for (int i = 0; i < other.getCustomWaypointProportions().size(); i++)
      {
         plannedFootstepInternal.getCustomWaypointProportions().add(other.getCustomWaypointProportions().get(i));
      }
      plannedFootstepInternal.getCustomWaypointPositions().clear();
      for (int i = 0; i < other.getCustomWaypointPositions().size(); i++)
      {
         plannedFootstepInternal.getCustomWaypointPositions().add(new Point3D(other.getCustomWaypointPositions().get(i)));
      }

      plannedFootstepTrajectory.clear();
      if (otherTrajectory != null)
      {
         for (Axis3D axis : Axis3D.values)
         {
            List<PolynomialReadOnly> polynomialListCopy = RDXVisualTools.copyPolynomialList(otherTrajectory.get(axis));
            plannedFootstepTrajectory.put(axis, polynomialListCopy);
         }
      }

      updateTrajectoryModel(plannedFootstepInternal, plannedFootstepTrajectory);
   }

   public void update()
   {
      // Update the internally held planned footstep pose
      if (!plannedFootstepInternal.getFootstepPose().epsilonEquals(selectablePose3DGizmo.getPoseGizmo().getPose(), 1e-2))
         wasPoseUpdated = true;
      plannedFootstepInternal.getFootstepPose().set(selectablePose3DGizmo.getPoseGizmo().getPose());

      textFramePose.setIncludingFrame(getFootPose());
      textFramePose.appendYawRotation(-Math.PI / 2.0);
      textFramePose.appendTranslation(-0.04, 0.0, 0.035); // The text is higher in Z direction so it's not inside the foot
      textFramePose.changeFrame(ReferenceFrame.getWorldFrame());
      LibGDXTools.toLibGDX(textFramePose, tempTransform, footstepIndexText.getModelTransform());

      if (plannedFootstepInput.get() != null)
      {
         Pair<PlannedFootstep, EnumMap<Axis3D, List<PolynomialReadOnly>>> pair = plannedFootstepInput.getAndSet(null);
         updatePlannedTrajectoryInternal(pair.getLeft(), pair.getRight());
      }

      if (collisionBoxFrame.getReferenceFrame().getParent() != selectablePose3DGizmo.getPoseGizmo().getGizmoFrame())
         collisionBoxFrame.setParentFrame(selectablePose3DGizmo.getPoseGizmo().getGizmoFrame());
      if (selectionCollisionBox.getReferenceFrame() != selectablePose3DGizmo.getPoseGizmo().getGizmoFrame())
         selectionCollisionBox.setReferenceFrame(selectablePose3DGizmo.getPoseGizmo().getGizmoFrame());

      updateHoverState();
   }

   public void updateHoverState()
   {
      if (isMouseHovering || isVRHovering.get(RobotSide.RIGHT) || isVRHovering.get(RobotSide.LEFT))
      {
         if (getFootstepSide() == RobotSide.LEFT)
            footstepModelInstance.materials.get(0).set(new ColorAttribute(ColorAttribute.Diffuse, 1.0f, 0.0f, 0.0f, 0.0f));
         else
            footstepModelInstance.materials.get(0).set(new ColorAttribute(ColorAttribute.Diffuse, 0.0f, 1.0f, 0.0f, 0.0f));
      }
      else
      {
         if (plannedFootstepInternal.getRobotSide() == RobotSide.LEFT)
            footstepModelInstance.materials.get(0).set(new ColorAttribute(ColorAttribute.Diffuse, 0.5f, 0.0f, 0.0f, 0.0f));
         else
            footstepModelInstance.materials.get(0).set(new ColorAttribute(ColorAttribute.Diffuse, 0.0f, 0.5f, 0.0f, 0.0f));
      }
   }

   public void calculateVRPick(RDXVRContext vrContext)
   {
      for (RobotSide side : RobotSide.values)
      {
         vrContext.getController(side).runIfConnected(controller ->
         {
            if (pointCollidable.collide(controller.getPickPointPose().getPosition()))
            {
               vrPickResult.get(side).setHoveringCollsion(controller.getPickPointPose().getPosition(), pointCollidable.getClosestPointOnSurface());
               vrPickResult.get(side).setPickedObjectID(this, "Footstep " + footstepIndexText.getCurrentText());
               controller.addPickResult(vrPickResult.get(side));
            }
         });
      }
   }

   public void processVRInput(RDXVRContext vrContext)
   {
      for (RobotSide side : RobotSide.values)
      {
         vrContext.getController(side).runIfConnected(controller ->
         {
            RDXVRDragData gripDragData = controller.getGripDragData();

            if (gripDragData.getDragJustStarted() && vrPickResult.get(side)== controller.getSelectedPick())
            {
               gripDragData.setObjectBeingDragged(this);
               gripDragData.setInteractableFrameOnDragStart(selectablePose3DGizmo.getPoseGizmo().getGizmoFrame());
            }
            if (gripDragData.isDraggingSomething() && gripDragData.getObjectBeingDragged() == this && vrContext.getController(side.getOppositeSide()).getGripDragData().getObjectBeingDragged() != this)
            {
               gripDragData.getDragFrame().getTransformToDesiredFrame(selectablePose3DGizmo.getPoseGizmo().getTransformToParent(), ReferenceFrame.getWorldFrame());
            }
            isVRHovering.put(side, vrPickResult.get(side) == controller.getSelectedPick());
         });
      }
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      selectablePose3DGizmo.calculate3DViewPick(input);
      Line3DReadOnly pickRayInWorld = input.getPickRayInWorld();
      double collision = mouseCollidable.collide(pickRayInWorld, collisionBoxFrame.getReferenceFrame());
      if (!Double.isNaN(collision))
      {
         pickResult.addPickCollision(collision);
         input.addPickResult(pickResult);
      }
   }

   public void process3DViewInput(ImGui3DViewInput input, boolean currentlyPlacingFootstep)
   {
      isMouseHovering = pickResult == input.getClosestPick();
      isClickedOn = isMouseHovering && input.mouseReleasedWithoutDrag(ImGuiMouseButton.Left);

      // FIXME:
      if (currentlyPlacingFootstep)
      {
         selectablePose3DGizmo.process3DViewInput(input);
      }
      else
      {
         selectablePose3DGizmo.process3DViewInput(input, isMouseHovering);
      }

      footstepModelInstance.transform.setToRotationRad(selectablePose3DGizmo.getPoseGizmo().getPose().getRotation().getX32(),
                                                       selectablePose3DGizmo.getPoseGizmo().getPose().getRotation().getY32(),
                                                       selectablePose3DGizmo.getPoseGizmo().getPose().getRotation().getZ32(),
                                                       (float) selectablePose3DGizmo.getPoseGizmo().getPose().getRotation().angle());
      footstepModelInstance.transform.setTranslation(selectablePose3DGizmo.getPoseGizmo().getPose().getPosition().getX32(),
                                                     selectablePose3DGizmo.getPoseGizmo().getPose().getPosition().getY32(),
                                                     selectablePose3DGizmo.getPoseGizmo().getPose().getPosition().getZ32());
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      selectablePose3DGizmo.getVirtualRenderables(renderables, pool);
      footstepIndexText.getRenderables(renderables, pool);
      footstepModelInstance.getRenderables(renderables, pool);

      for (ModelInstance trajectoryPoint : trajectoryWaypointModel)
         trajectoryPoint.getRenderables(renderables, pool);

      swingTrajectoryModel.getRenderables(renderables, pool);
   }

   // Sets the gizmo's position and rotation
   public void setGizmoPose(double x, double y, double z, RigidBodyTransformReadOnly transform)
   {
      RigidBodyTransform gizmoTransform = selectablePose3DGizmo.getPoseGizmo().getTransformToParent();
      gizmoTransform.getTranslation().set(x, y, z);
      gizmoTransform.getRotation().set(transform.getRotation());
      plannedFootstepInternal.getFootstepPose().set(gizmoTransform);
      wasPoseUpdated = true;
   }

   public void flashFootstepWhenBadPlacement(BipedalFootstepPlannerNodeRejectionReason reason)
   {
      if (reason == null)
      {
         if (getFootstepSide() == RobotSide.LEFT)
         {
            if (isMouseHovering())
               setColor(1.0f, 0.0f, 0.0f, 0.0f);
            else
               setColor(0.5f, 0.0f, 0.0f, 0.0f);
         }
         else
         {
            if (isMouseHovering())
               setColor(0.0f, 1.0f, 0.0f, 0.0f);
            else
               setColor(0.0f, 0.5f, 0.0f, 0.0f);
         }
      }
      else
      {
         if (!timerFlashingFootsteps.hasBeenSet())
         {
            timerFlashingFootsteps.reset();
            flashingFootStepsColorHigh = false;
         }
         if (timerFlashingFootsteps.isExpired(0.1))
         {
            flashingFootStepsColorHigh = !flashingFootStepsColorHigh;
            timerFlashingFootsteps.reset();
         }
         if (getFootstepSide() == RobotSide.LEFT)
         {
            if (flashingFootStepsColorHigh)
               setColor(1.0f, 0.0f, 0.0f, 0.0f);
            else
               setColor(0.5f, 0.0f, 0.0f, 0.0f);
         }
         else
         {
            if (flashingFootStepsColorHigh)
               setColor(0.0f, 1.0f, 0.0f, 0.0f);
            else
               setColor(0.0f, 0.5f, 0.0f, 0.0f);
         }
      }
   }

   // sets color of the corresponding footstep in the list
   public void setColor(float r, float g, float b, float a)
   {
      footstepModelInstance.materials.get(0).set(new ColorAttribute(ColorAttribute.Diffuse, r, g, b, a));
   }

   public void setFootstepModelInstance(RDXModelInstance footstepModelInstance)
   {
      this.footstepModelInstance = footstepModelInstance;
   }

   public RobotSide getFootstepSide()
   {
      return plannedFootstepInternal.getRobotSide();
   }

   public ModelInstance getFootstepModelInstance()
   {
      return footstepModelInstance;
   }

   public boolean isMouseHovering()
   {
      return isMouseHovering;
   }

   public double getYaw()
   {
      return plannedFootstepInternal.getFootstepPose().getYaw();
   }

   public FramePose3DReadOnly getFootPose()
   {
      return plannedFootstepInternal.getFootstepPose();
   }

   public void updatePose(RigidBodyTransformReadOnly footstepPose)
   {
      selectablePose3DGizmo.getPoseGizmo().getTransformToParent().set(footstepPose);
      selectablePose3DGizmo.getPoseGizmo().update();

      wasPoseUpdated = !plannedFootstepInternal.getFootstepPose().epsilonEquals(footstepPose, 1e-2);
      plannedFootstepInternal.getFootstepPose().set(footstepPose);
   }

   public boolean pollWasPoseUpdated()
   {
      boolean value = wasPoseUpdated;
      wasPoseUpdated = false;
      return value;
   }

   private void updateTrajectoryModel(PlannedFootstep footstep, EnumMap<Axis3D, List<PolynomialReadOnly>> trajectory)
   {
      trajectoryWaypointModel.clear();
      if (footstep.getTrajectoryType() == TrajectoryType.CUSTOM)
      {
         for (Point3D trajectoryPosition : footstep.getCustomWaypointPositions())
         {
            ModelInstance sphere = RDXModelBuilder.createSphere(0.03f, Color.WHITE);
            LibGDXTools.toLibGDX(trajectoryPosition, sphere.transform);
            trajectoryWaypointModel.add(sphere);
         }
      }
      else if (footstep.getTrajectoryType() == TrajectoryType.WAYPOINTS)
      {
         for (FrameSE3TrajectoryPoint trajectoryPosition : footstep.getSwingTrajectory())
         {
            ModelInstance sphere = RDXModelBuilder.createSphere(0.03f, Color.BLACK);
            LibGDXTools.toLibGDX(trajectoryPosition.getPosition(), sphere.transform);
            trajectoryWaypointModel.add(sphere);
         }
      }

      swingTrajectoryModel.clear();
      List<RDXPolynomial.Polynomial3DVariableHolder> polynomials = RDXVisualTools.createPolynomial3DList(trajectory.get(Axis3D.X), trajectory.get(Axis3D.Y), trajectory.get(Axis3D.Z));
      swingTrajectoryModel.compute(polynomials);
   }

   /**
    * TODO: Evaluate the use of this method.
    */
   public void copyFrom(RDXBaseUI baseUI, RDXInteractableFootstep manuallyPlacedFootstep)
   {
      this.footstepIndexText = manuallyPlacedFootstep.footstepIndexText;
      this.footstepModelInstance = manuallyPlacedFootstep.footstepModelInstance;
      this.plannedFootstepInternal.set(manuallyPlacedFootstep.plannedFootstepInternal);
      this.selectablePose3DGizmo = manuallyPlacedFootstep.selectablePose3DGizmo;
      this.isMouseHovering = manuallyPlacedFootstep.isMouseHovering;
      this.isClickedOn = manuallyPlacedFootstep.isClickedOn;
      this.textFramePose.setIncludingFrame(manuallyPlacedFootstep.textFramePose);
      this.flashingFootStepsColorHigh = manuallyPlacedFootstep.flashingFootStepsColorHigh;
   }

   public RDXSelectablePose3DGizmo getSelectablePose3DGizmo()
   {
      return selectablePose3DGizmo;
   }

   public int getIndex()
   {
      return index;
   }
}
