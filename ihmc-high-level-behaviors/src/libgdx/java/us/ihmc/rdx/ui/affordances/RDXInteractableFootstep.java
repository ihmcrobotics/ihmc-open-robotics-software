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
import us.ihmc.rdx.ui.gizmo.StepCheckIsPointInsideAlgorithm;
import us.ihmc.rdx.ui.graphics.RDXFootstepPlanGraphic;
import us.ihmc.rdx.visualizers.RDXPolynomial;
import us.ihmc.robotics.math.trajectories.core.Polynomial;
import us.ihmc.robotics.math.trajectories.core.Polynomial3D;
import us.ihmc.robotics.math.trajectories.interfaces.PolynomialReadOnly;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSE3TrajectoryPoint;
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

      String txt = footstepSide.getSideNameFirstLetter() + (index + 1);
      if (!textRenderablesMap.containsKey(txt))
      {
         footstepIndexText = new RDX3DSituatedText("" + txt);
         textRenderablesMap.put(txt, footstepIndexText);
      }
      else
      {
         footstepIndexText = textRenderablesMap.get(txt);
      }
   }

   public void reset()
   {
      getFootstepModelInstance().transform.val[Matrix4.M03] = Float.NaN;
      plannedFootstepInternal.reset();
      plannedFootstepTrajectory.clear();
   }

   public PlannedFootstepReadOnly getPlannedFootstep()
   {
      return plannedFootstepInternal;
   }

   public void updateFromPlannedStep(RDXBaseUI baseUI, PlannedFootstep plannedFootstep, EnumMap<Axis3D, List<PolynomialReadOnly>> swingTrajectory, int footstepIndex)
   {
      plannedFootstepInput.set(null);
      plannedFootstepInternal.set(plannedFootstep);
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
         Color regionColor = RDXFootstepPlanGraphic.footstepColors.get(plannedFootstep.getRobotSide());
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

      String text = plannedFootstepInternal.getRobotSide().getSideNameFirstLetter() + (footstepIndex + 1);
      if (!textRenderablesMap.containsKey(text))
      {
         footstepIndexText = new RDX3DSituatedText("" + text);
         textRenderablesMap.put(text, footstepIndexText);
      }
      else
      {
         footstepIndexText = textRenderablesMap.get(text);
      }

      updatePose(plannedFootstep.getFootstepPose());
   }

   public void updatePlannedTrajectory(Pair<PlannedFootstep, EnumMap<Axis3D, List<PolynomialReadOnly>>> other)
   {
      wasPoseUpdated = true;
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
      updateTrajectoryModel(plannedFootstepInternal, plannedFootstepTrajectory);
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

      wasPoseUpdated = plannedFootstepInternal.getFootstepPose().epsilonEquals(footstepPose, 1e-2);
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
