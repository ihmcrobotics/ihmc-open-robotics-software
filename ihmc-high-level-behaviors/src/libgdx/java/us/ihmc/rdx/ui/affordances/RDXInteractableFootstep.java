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
import us.ihmc.commons.thread.Notification;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.shape.primitives.Sphere3D;
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
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.tools.RDXModelLoader;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.rdx.vr.RDXVRControllerModel;
import us.ihmc.robotics.interaction.StepCheckIsPointInsideAlgorithm;
import us.ihmc.rdx.ui.graphics.RDXFootstepGraphic;
import us.ihmc.rdx.visualizers.RDXPolynomial;
import us.ihmc.robotics.math.trajectories.core.Polynomial;
import us.ihmc.robotics.math.trajectories.interfaces.PolynomialReadOnly;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.tools.Timer;

import java.util.*;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;

public class RDXInteractableFootstep
{
   // Intended to reuse text renderables, as they are relatively expensive to create
   private static final Map<String, RDX3DSituatedText> textRenderablesMap = new HashMap<>();
   private RDX3DSituatedText footstepIndexText;
   private ModelInstance footstepModelInstance;
   private RDXSelectablePose3DGizmo selectablePose3DGizmo;
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private boolean isHovered;
   private final Sphere3D boundingSphere = new Sphere3D(0.1);
   private boolean isClickedOn;
   private final FramePose3D textFramePose = new FramePose3D();
   private final Timer timerFlashingFootsteps = new Timer();
   private boolean flashingFootStepsColorHigh = false;
   private final ImGui3DViewPickResult pickResult = new ImGui3DViewPickResult();
   private boolean isMouseHovering;
   private final Notification contextMenuNotification = new Notification();
   private boolean isVRHovering;
   private boolean isVRPointing;
   private double sizeChange = 0;
   private RDXVRControllerModel controllerModel = RDXVRControllerModel.UNKNOWN;
   private SideDependentList<FramePose3D> frontController = new SideDependentList<>();
   private final SideDependentList<Boolean> isVRDragging = new SideDependentList<>(false, false);
   private final SideDependentList<Boolean> isVRLasering = new SideDependentList<>(false, false);
   private final SideDependentList<Boolean> modified = new SideDependentList<>(false, false);

   private final SideDependentList<ModifiableReferenceFrame> dragReferenceFrame = new SideDependentList<>();
   private final SideDependentList<ModifiableReferenceFrame> laserReferenceFrame = new SideDependentList<>();

   private final List<ModelInstance> trajectoryWaypointModel = new ArrayList<>();
   private final RDXPolynomial swingTrajectoryModel = new RDXPolynomial(0.03, 25);

   private final SideDependentList<ConvexPolygon2D> defaultPolygons;
   private final AtomicReference<Pair<PlannedFootstep, EnumMap<Axis3D, List<PolynomialReadOnly>>>> plannedFootstepInput = new AtomicReference<>(null);
   private final PlannedFootstep plannedFootstepInternal;
   private final EnumMap<Axis3D, List<PolynomialReadOnly>> plannedFootstepTrajectory = new EnumMap<>(Axis3D.class);

   private boolean wasPoseUpdated = false;

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

      updateFootstepIndexText(index);
   }

   /**
    * Adds the footstep index text to the footstep (e.g. "R0" or "L4")
    *
    * @param index Index of the footstep within the array of footsteps being planned - determines the number displayed
    */
   public void updateFootstepIndexText(int index)
   {
      String text = plannedFootstepInternal.getRobotSide().getSideNameFirstLetter() + index;
      if (!textRenderablesMap.containsKey(text))
      {
         footstepIndexText = new RDX3DSituatedText(text);
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
         swingTrajectory.keySet().forEach(key -> plannedFootstepTrajectory.put(key, copyPolynomialList(swingTrajectory.get(key))));

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
            List<PolynomialReadOnly> polynomialListCopy = copyPolynomialList(otherTrajectory.get(axis));
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

      double textHeight = 0.08;
      textFramePose.setIncludingFrame(getFootPose());

      textFramePose.appendYawRotation(-Math.PI / 2.0);
      textFramePose.appendTranslation(-0.03, 0.0, 0.035); //note: Make text higher in z direction, so it's not inside the foot
      textFramePose.changeFrame(ReferenceFrame.getWorldFrame());
      LibGDXTools.toLibGDX(textFramePose, tempTransform, footstepIndexText.getModelInstance().transform);
      footstepIndexText.scale((float) textHeight);

      if (plannedFootstepInput.get() != null)
      {
         Pair<PlannedFootstep, EnumMap<Axis3D, List<PolynomialReadOnly>>> pair = plannedFootstepInput.getAndSet(null);
         updatePlannedTrajectoryInternal(pair.getLeft(), pair.getRight());
      }
   }

   public void processVRInput(RDXVRContext vrContext)
   {
      isVRHovering = false;
      isVRPointing = false;

      for (RobotSide side : RobotSide.values)
      {
         vrContext.getController(side).runIfConnected(controller ->
         {
            boolean isHovering = false;
               isHovering |= controller.getPickPointPose().getTranslation().distance(getFootPose().getPosition()) < 0.1;
            isVRHovering |= isHovering;

            boolean gripped = controller.getGripped();
            boolean newlyGripped = gripped && controller.getGrippedChanged();
            boolean newlyUnGripped = !gripped && controller.getGrippedChanged();

            if (dragReferenceFrame.get(side) == null)
            {
               dragReferenceFrame.put(side, new ModifiableReferenceFrame(controller.getPickPoseFrame()));
            }

            if (isHovering && newlyGripped)
            {
               modified.put(side, true);
               selectablePose3DGizmo.getPoseGizmo().getGizmoFrame().getTransformToDesiredFrame(dragReferenceFrame.get(side).getTransformToParent(),
                                                                                               controller.getPickPoseFrame());
               dragReferenceFrame.get(side).getReferenceFrame().update();
               isVRDragging.put(side, true);
            }

            if (isVRDragging.get(side))
            {
               dragReferenceFrame.get(side).getReferenceFrame().getTransformToDesiredFrame(selectablePose3DGizmo.getPoseGizmo().getTransformToParent(),
                                                                                           ReferenceFrame.getWorldFrame());
            }

            if (newlyUnGripped || !gripped)
               isVRDragging.put(side, false);

            boolean isPointing = false;
            calculateVR(vrContext);
            if(frontController.get(side) != null)
            {
               isPointing |= frontController.get(side).getPositionDistance(getFootPose().getPosition()) < 0.1;
               isVRPointing |= isPointing;

               boolean triggered = controller.getClickTriggerActionData().bState();
               boolean newPressedTrigger = triggered && controller.getClickTriggerActionData().bChanged();
               boolean newReleasedTrigger = !triggered && controller.getClickTriggerActionData().bChanged();

               if (isPointing && newPressedTrigger)
               {
                     modified.put(side, true);
                     selectablePose3DGizmo.getPoseGizmo()
                                          .getGizmoFrame()
                                          .getTransformToDesiredFrame(dragReferenceFrame.get(side).getTransformToParent(), frontController.get(side).getReferenceFrame());
                     dragReferenceFrame.get(side).getReferenceFrame().update();
                     isVRLasering.put(side, true);
               }
               else if (newPressedTrigger && !isPointing)
               {

               }

               if (isVRLasering.get(side))
               {
                  setGizmoPose(frontController.get(side).getTranslationX(), frontController.get(side).getTranslationY(), frontController.get(side).getTranslationZ(), frontController.get(side).getYaw(), 0, 0);
               }

               if (newReleasedTrigger)
               {
                  isVRLasering.put(side, false);
               }
            }
            if (isHovering || isPointing)
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
         });
      }
   }
   public void calculateVR (RDXVRContext vrContext)
   {
      if (controllerModel == RDXVRControllerModel.UNKNOWN)
         controllerModel = vrContext.getControllerModel();
      boolean noSelectedPick = true;
      for (RobotSide side : RobotSide.values)
         noSelectedPick = noSelectedPick && vrContext.getSelectedPick().get(side) == null;
      if (noSelectedPick)
      {
         for(RobotSide side : RobotSide.values)
         {
            if (side != null)
            {
               vrContext.getController(side).runIfConnected(controller ->
               {
                  frontController.put(side, new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                            vrContext.getController(side).getXForwardZUpPose()));
                  frontController.get(side).changeFrame(vrContext.getController(side).getXForwardZUpControllerFrame());
                  frontController.get(side).setToZero(vrContext.getController(side).getXForwardZUpControllerFrame());
                  frontController.get(side).getPosition().addX(.1);
                  frontController.get(side).changeFrame(ReferenceFrame.getWorldFrame());
                  sizeChange = vrContext.getController(side).getXForwardZUpPose().getZ() / (
                        vrContext.getController(side).getXForwardZUpPose().getZ() - frontController.get(side).getTranslationZ());
                  frontController.get(side).changeFrame(vrContext.getController(side).getXForwardZUpControllerFrame());
                  frontController.get(side).getPosition().addX(sizeChange * frontController.get(side).getX());
                  frontController.get(side).changeFrame(ReferenceFrame.getWorldFrame());
                  frontController.get(side).getOrientation()
                                 .setToYawOrientation(-vrContext.getController(side).getXForwardZUpPose().getRoll());
                  frontController.get(side).getRotation().setYawPitchRoll(frontController.get(side).getYaw(), 0, 0);
                  frontController.get(side).getPosition().setZ(0);
               });
            }
         }
      }

   }
   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      selectablePose3DGizmo.calculate3DViewPick(input);

      StepCheckIsPointInsideAlgorithm stepCheckIsPointInsideAlgorithm = new StepCheckIsPointInsideAlgorithm();
      stepCheckIsPointInsideAlgorithm.update(boundingSphere.getRadius(), boundingSphere.getPosition());

      Function<Point3DReadOnly, Boolean> isPointInside = boundingSphere::isPointInside;
      boolean pickIntersected = !Double.isNaN(stepCheckIsPointInsideAlgorithm.intersect(input.getPickRayInWorld(), 100, isPointInside));
      if (pickIntersected)
      {
         pickResult.setDistanceToCamera(stepCheckIsPointInsideAlgorithm.getClosestIntersection().distance(input.getPickRayInWorld().getPoint()));
         input.addPickResult(pickResult);
      }
   }

   public void process3DViewInput(ImGui3DViewInput input, boolean currentlyPlacingFootstep)
   {
      isHovered = pickResult == input.getClosestPick();
      isClickedOn = isHovered && input.mouseReleasedWithoutDrag(ImGuiMouseButton.Left);

      // TODO: mouse hovering on the footstep. (get foot validity warning text when this happens)
      if (isHovered)
      {
         if (plannedFootstepInternal.getRobotSide() == RobotSide.LEFT)
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

      // FIXME:
      if (currentlyPlacingFootstep)
      {
         selectablePose3DGizmo.process3DViewInput(input);
      }
      else
      {
         selectablePose3DGizmo.process3DViewInput(input, isHovered);
      }

      footstepModelInstance.transform.setToRotationRad(selectablePose3DGizmo.getPoseGizmo().getPose().getRotation().getX32(),
                                                       selectablePose3DGizmo.getPoseGizmo().getPose().getRotation().getY32(),
                                                       selectablePose3DGizmo.getPoseGizmo().getPose().getRotation().getZ32(),
                                                       (float) selectablePose3DGizmo.getPoseGizmo().getPose().getRotation().angle());
      footstepModelInstance.transform.setTranslation(selectablePose3DGizmo.getPoseGizmo().getPose().getPosition().getX32(),
                                                     selectablePose3DGizmo.getPoseGizmo().getPose().getPosition().getY32(),
                                                     selectablePose3DGizmo.getPoseGizmo().getPose().getPosition().getZ32());
      boundingSphere.getPosition().set(selectablePose3DGizmo.getPoseGizmo().getPose().getPosition());
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

      boundingSphere.getPosition().set(x, y, z);
   }
   public void setGizmoPose(double x, double y, double z, double yaw, double pitch, double roll)
   {
      RigidBodyTransform gizmoTransform = selectablePose3DGizmo.getPoseGizmo().getTransformToParent();
      gizmoTransform.getTranslation().set(x, y, z);
      gizmoTransform.getRotation().setYawPitchRoll(yaw, pitch, roll);
      plannedFootstepInternal.getFootstepPose().set(gizmoTransform);
      wasPoseUpdated = true;

      boundingSphere.getPosition().set(x, y, z);
   }

   public void flashFootstepWhenBadPlacement(BipedalFootstepPlannerNodeRejectionReason reason)
   {
      if (reason == null)
      {
         if (getFootstepSide() == RobotSide.LEFT)
         {
            if (isHovered())
               setColor(1.0f, 0.0f, 0.0f, 0.0f);
            else
               setColor(0.5f, 0.0f, 0.0f, 0.0f);
         }
         else
         {
            if (isHovered())
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

   public boolean isHovered()
   {
      return isHovered;
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
      List<RDXPolynomial.Polynomial3DVariableHolder> polynomials = createPolynomial3DList(trajectory.get(Axis3D.X), trajectory.get(Axis3D.Y), trajectory.get(Axis3D.Z));
      swingTrajectoryModel.compute(polynomials);
   }

   private static List<RDXPolynomial.Polynomial3DVariableHolder> createPolynomial3DList(List<PolynomialReadOnly> xPolynomial, List<PolynomialReadOnly> yPolynomial, List<PolynomialReadOnly> zPolynomial)
   {
      List<RDXPolynomial.Polynomial3DVariableHolder> polynomials = new ArrayList<>();
      if (xPolynomial == null || yPolynomial == null || zPolynomial == null)
         return polynomials;

      for (int i = 0; i < xPolynomial.size(); i++)
      {
         polynomials.add(new RDXPolynomial.Polynomial3DVariables(xPolynomial.get(i), yPolynomial.get(i), zPolynomial.get(i)));
      }
      return polynomials;
   }

   private static List<PolynomialReadOnly> copyPolynomialList(List<PolynomialReadOnly> other)
   {
      List<PolynomialReadOnly> copy = new ArrayList<>();
      other.forEach(poly -> copy.add(copyPolynomial(poly)));

      return copy;
   }

   private static PolynomialReadOnly copyPolynomial(PolynomialReadOnly other)
   {
      Polynomial polynomial = new Polynomial(other.getNumberOfCoefficients());
      polynomial.set(other);
      return polynomial;
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
      this.isHovered = manuallyPlacedFootstep.isHovered;
      this.isClickedOn = manuallyPlacedFootstep.isClickedOn;
      this.textFramePose.setIncludingFrame(manuallyPlacedFootstep.textFramePose);
      this.flashingFootStepsColorHigh = manuallyPlacedFootstep.flashingFootStepsColorHigh;
   }
}
