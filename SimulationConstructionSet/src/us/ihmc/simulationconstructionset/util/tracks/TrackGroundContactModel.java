package us.ihmc.simulationconstructionset.util.tracks;

import java.util.ArrayList;

import javax.vecmath.Vector3d;

import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.simulationconstructionset.GroundContactModel;

public class TrackGroundContactModel implements GroundContactModel
{
   private static final long serialVersionUID = 7659055431615061425L;
   private TrackGroundContactPoint[] points;
   private GroundContactModel baseModel;

   public TrackGroundContactModel(GroundContactModel baseModel, Track[] tracks)
   {
      ArrayList<TrackGroundContactPoint> allPoints = new ArrayList<TrackGroundContactPoint>();

      for (int i = 0; i < tracks.length; i++)
      {
         Track track = tracks[i];
         if (track != null)
         {
            ArrayList<TrackGroundContactPoint> track_points = tracks[i].getTrackGroundContactPoints();
            allPoints.addAll(track_points);
         }
      }

      points = new TrackGroundContactPoint[allPoints.size()];
      allPoints.toArray(points);

      this.baseModel = baseModel;
   }

   private final Vector3d tempVelocity = new Vector3d();
   private final Vector3d tempTrackVelocity = new Vector3d();
   @Override
   public void doGroundContact()
   {
      // Change the velocities to take into account the track.
      for (int i = 0; i < points.length; i++)
      {
         points[i].getVelocity(tempVelocity);
         points[i].getTrackVelocity(tempTrackVelocity);

         tempVelocity.add(tempTrackVelocity);
         points[i].setVelocity(tempVelocity);
      }

      // Compute the forces and such.
      baseModel.doGroundContact();

      // Change the velocities back.

      for (int i = 0; i < points.length; i++)
      {
         points[i].getVelocity(tempVelocity);
         points[i].getTrackVelocity(tempTrackVelocity);

         tempVelocity.sub(tempTrackVelocity);
         points[i].setVelocity(tempVelocity);
      }


      // +++JEP OPTIMIZE: Only check to see if a point should be on every CHECK_TICKS cycles.

      /*
       * check_tick++;
       * if (check_tick == CHECK_TICKS-1)
       * {
       * for(int i=0; i<points.length; i++)
       * {
       *   if (points[i].fs.getDoubleValue() < -0.5) points[i].setNotInContact();
       * }
       * }
       *
       * if (check_tick > CHECK_TICKS)
       * {
       * for(int i=0; i<points.length; i++)
       * {
       *   if (points[i].isNotInContact()) points[i].fs.set(-1.0);
       * }
       * check_tick = 0;
       * }
       */
   }

   @Override
   public GroundProfile3D getGroundProfile3D()
   {
      return baseModel.getGroundProfile3D();
   }
   
   @Override
   public void setGroundProfile3D(GroundProfile3D groundProfile3D)
   {
      baseModel.setGroundProfile3D(groundProfile3D);
   }
}
