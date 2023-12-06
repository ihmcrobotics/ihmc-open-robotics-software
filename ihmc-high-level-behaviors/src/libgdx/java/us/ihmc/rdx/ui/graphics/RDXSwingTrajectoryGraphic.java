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
import us.ihmc.log.LogTools;
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
   private final RDXPolynomial swingTrajectoryModel = new RDXPolynomial(0.02, 25);
   private final EnumMap<Axis3D, List<PolynomialReadOnly>> plannedFootstepTrajectory = new EnumMap<>(Axis3D.class);

   public RDXSwingTrajectoryGraphic()
   {
   }

   public void updateTrajectoryModel(PlannedFootstep footstep, EnumMap<Axis3D, List<PolynomialReadOnly>> trajectory)
   {
      plannedFootstepTrajectory.clear();
      if (trajectory != null)
         trajectory.keySet().forEach(key -> plannedFootstepTrajectory.put(key, RDXVisualTools.copyPolynomialList(trajectory.get(key))));

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

      LogTools.warn("Swing trajectory type: " + footstep.getTrajectoryType() + ", number of waypoints: " + trajectoryWaypointModel.size() + ", number of polynomials: " + polynomials.size());
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
