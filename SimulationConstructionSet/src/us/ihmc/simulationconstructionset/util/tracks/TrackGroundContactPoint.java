package us.ihmc.simulationconstructionset.util.tracks;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class TrackGroundContactPoint extends GroundContactPoint
{
   private static final long serialVersionUID = -936610163728292801L;
   public DoubleYoVariable dx_track, dy_track, dz_track;

   public TrackGroundContactPoint(String name, YoVariableRegistry registry)
   {
      super(name, registry);

      dx_track = new DoubleYoVariable(name + "_dx_track", registry);
      dy_track = new DoubleYoVariable(name + "_dy_track", registry);
      dz_track = new DoubleYoVariable(name + "_dz_track", registry);
   }

   public void getTrackVelocity(Vector3d trackVelocityToPack)
   {
      trackVelocityToPack.set(dx_track.getDoubleValue(), dy_track.getDoubleValue(), dz_track.getDoubleValue());
   }
   @Override
   protected void updatePointPosition(RigidBodyTransform transform3D)
   {
      // +++JEP: OPTIMIZE: Don't compute if point is turned off...
      if (this.isDisabled())
         return;
      else
         super.updatePointPosition(transform3D);
   }

   @Override
   public void updatePointVelocity(Matrix3d R0_i, Vector3d comOffset, Vector3d v_i, Vector3d w_i)
   {
      // +++JEP: OPTIMIZE: Don't compute if point is turned off...
      if (this.isDisabled())
         return;
      else
         super.updatePointVelocity(R0_i, comOffset, v_i, w_i);

   }


}
