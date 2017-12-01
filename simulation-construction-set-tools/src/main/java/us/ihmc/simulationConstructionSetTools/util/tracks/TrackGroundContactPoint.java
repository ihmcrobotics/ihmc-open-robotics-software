package us.ihmc.simulationConstructionSetTools.util.tracks;

import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class TrackGroundContactPoint extends GroundContactPoint
{
   private static final long serialVersionUID = -936610163728292801L;
   public YoDouble dx_track, dy_track, dz_track;

   public TrackGroundContactPoint(String name, YoVariableRegistry registry)
   {
      super(name, registry);

      dx_track = new YoDouble(name + "_dx_track", registry);
      dy_track = new YoDouble(name + "_dy_track", registry);
      dz_track = new YoDouble(name + "_dz_track", registry);
   }

   public void getTrackVelocity(Vector3D trackVelocityToPack)
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
   public void updatePointVelocity(RotationMatrixReadOnly R0_i, Vector3DReadOnly comOffset, Vector3DReadOnly v_i, Vector3DReadOnly w_i)
   {
      // +++JEP: OPTIMIZE: Don't compute if point is turned off...
      if (this.isDisabled())
         return;
      else
         super.updatePointVelocity(R0_i, comOffset, v_i, w_i);

   }


}
