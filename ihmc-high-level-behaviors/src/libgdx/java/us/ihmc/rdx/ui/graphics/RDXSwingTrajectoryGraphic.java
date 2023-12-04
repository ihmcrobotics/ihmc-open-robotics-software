package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.PlannedFootstep;
import us.ihmc.rdx.simulation.scs2.RDXVisualTools;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.visualizers.RDXPolynomial;
import us.ihmc.robotics.math.trajectories.interfaces.PolynomialReadOnly;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.trajectories.TrajectoryType;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;

public class RDXSwingTrajectoryGraphic implements RenderableProvider
{
   private final List<ModelInstance> trajectoryWaypointModel = new ArrayList<>();
   private final RDXPolynomial swingTrajectoryModel = new RDXPolynomial(0.03, 25);
   private final EnumMap<Axis3D, List<PolynomialReadOnly>> plannedFootstepTrajectory = new EnumMap<>(Axis3D.class);

   public RDXSwingTrajectoryGraphic()
   {
   }

   public void updateFromPlan(FootstepPlan footstepPlan, List<EnumMap<Axis3D, List<PolynomialReadOnly>>> swingTrajectories)
   {
      for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)
      {
         PlannedFootstep plannedStep = footstepPlan.getFootstep(i);
         EnumMap<Axis3D, List<PolynomialReadOnly>> swingTrajectory;
         if (swingTrajectories == null)
         {
            swingTrajectory = null;
         }
         else
         {
            if (i < swingTrajectories.size())
               swingTrajectory = swingTrajectories.get(i);
            else
               swingTrajectory = null;
         }
         plannedFootstepTrajectory.clear();
         if (swingTrajectory != null)
            swingTrajectory.keySet().forEach(key -> plannedFootstepTrajectory.put(key, RDXVisualTools.copyPolynomialList(swingTrajectory.get(key))));

         updateTrajectoryModel(plannedStep, plannedFootstepTrajectory);
      }
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
      List<RDXPolynomial.Polynomial3DVariableHolder> polynomials = RDXVisualTools.createPolynomial3DList(trajectory.get(Axis3D.X),
                                                                                                         trajectory.get(Axis3D.Y),
                                                                                                         trajectory.get(Axis3D.Z));
      swingTrajectoryModel.compute(polynomials);
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (ModelInstance trajectoryPoint : trajectoryWaypointModel)
         trajectoryPoint.getRenderables(renderables, pool);

      swingTrajectoryModel.getRenderables(renderables, pool);
   }

   public void destroy()
   {
      for (ModelInstance trajectoryPoint : trajectoryWaypointModel)
         trajectoryPoint.model.dispose();

      swingTrajectoryModel.dispose();

   }
}
