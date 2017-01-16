package us.ihmc.simulationconstructionset.util.tracks;

import java.util.ArrayList;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.simulationconstructionset.FunctionToIntegrate;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class Track implements FunctionToIntegrate
{
   private TrackGroundContactPoint[][] trackPoints;

   private String name;
   private Joint joint;
   private Vector3d track_offset;
   private Matrix3d track_rotation;

   private double trackLength, trackRadius, trackWidth, trackPerimeter;

   private int numPointsPerTread, numTreads;

   private final YoVariableRegistry registry;
   private final DoubleYoVariable track_linear_velocity, track_linear_position, track_linear_force;

   public Track(String name, Joint joint, Robot rob, Vector3d offset, Matrix3d rotation, double trackLength, double trackRadius, double trackWidth,
                int numPointsPerTread, int numTreads)
   {
      registry = new YoVariableRegistry(name);
      rob.addYoVariableRegistry(registry);
      
      this.name = name;
      this.joint = joint;
      this.track_offset = offset;
      if (rotation != null)
         this.track_rotation = new Matrix3d(rotation);

      this.trackLength = trackLength;
      this.trackRadius = trackRadius;
      this.trackWidth = trackWidth;

      trackPerimeter = 2.0 * trackLength + 2.0 * Math.PI * trackRadius;

      this.numPointsPerTread = numPointsPerTread;
      this.numTreads = numTreads;

      trackPoints = new TrackGroundContactPoint[numTreads][numPointsPerTread];

      // Track GC Points:

      for (int i = 0; i < numTreads; i++)
      {
         for (int j = 0; j < numPointsPerTread; j++)
         {
            TrackGroundContactPoint trackGroundContactPoint = new TrackGroundContactPoint(name + i + "_" + j, registry);
            joint.addGroundContactPoint(trackGroundContactPoint);
            trackPoints[i][j] = trackGroundContactPoint;
         }
      }


      track_linear_velocity = new DoubleYoVariable(name + "_vel", registry);
      track_linear_position = new DoubleYoVariable(name + "_pos", registry);
      track_linear_force = new DoubleYoVariable(name + "_force", registry);

      this.setGroundContactOffsetsAndVelocities(0.0, 0.0);
      rob.addFunctionToIntegrate(this);
   }

   public String getName()
   {
      return name;
   }

   public DoubleYoVariable getLinearVelocity()
   {
      return this.track_linear_velocity;
   }

   public DoubleYoVariable getLinearPosition()
   {
      return this.track_linear_velocity;
   }

   public DoubleYoVariable getLinearForce()
   {
      return this.track_linear_velocity;
   }


   public void addTrackGraphics(Link link, AppearanceDefinition appearance)
   {
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.identity();
      linkGraphics.translate(track_offset);
      linkGraphics.rotate(track_rotation);
      linkGraphics.translate(0.0, 0.0, -trackRadius);
      linkGraphics.addCube(trackLength, trackWidth, 2.0 * trackRadius, appearance);

      linkGraphics.identity();
      linkGraphics.translate(track_offset);
      linkGraphics.rotate(track_rotation);
      linkGraphics.translate(trackLength / 2.0, trackWidth / 2.0, 0.0);
      linkGraphics.rotate(Math.PI / 2.0, Axis.X);
      linkGraphics.addCylinder(trackWidth, trackRadius, appearance);

      linkGraphics.identity();
      linkGraphics.translate(track_offset);
      linkGraphics.rotate(track_rotation);
      linkGraphics.translate(-trackLength / 2.0, trackWidth / 2.0, 0.0);
      linkGraphics.rotate(Math.PI / 2.0, Axis.X);
      linkGraphics.addCylinder(trackWidth, trackRadius, appearance);

      linkGraphics.identity();
      link.setLinkGraphics(linkGraphics);
   }

   private Matrix3d temp_rotation = new Matrix3d();
   private Vector3d temp_vector = new Vector3d();

   private Vector3d bottom_velocity = new Vector3d();
   private Vector3d velocity = new Vector3d();

   private void setGroundContactOffsetsAndVelocities(double track_linear_position, double track_linear_velocity)
   {
      joint.getRotationToWorld(temp_rotation);
      if (track_rotation != null)
         temp_rotation.mul(track_rotation);    // temp_rotation = temp_rotation * rotation;


      bottom_velocity.set(-track_linear_velocity, 0.0, 0.0);
      temp_rotation.transform(bottom_velocity);

      for (int i = 0; i < numTreads; i++)
      {
         double distanceAlongTrack = -track_linear_position + trackPerimeter * (i) / (numTreads);
         distanceAlongTrack = distanceAlongTrack % trackPerimeter;
         if (distanceAlongTrack < 0.0)
            distanceAlongTrack = distanceAlongTrack + trackPerimeter;

         // System.out.println(distanceAlongTrack);

         double x = 0.0, z = 0.0;

         if (distanceAlongTrack < trackLength)
         {
            x = -trackLength / 2.0 + distanceAlongTrack;
            z = -trackRadius;

            // velocity.set(-track_linear_velocity, 0.0, 0.0);
            // temp_rotation.transform(velocity);

            for (int j = 0; j < numPointsPerTread; j++)
            {
               TrackGroundContactPoint tgc = trackPoints[i][j];
               tgc.dx_track.set(bottom_velocity.getX());
               tgc.dy_track.set(bottom_velocity.getY());
               tgc.dz_track.set(bottom_velocity.getZ());
            }
         }

         else if (distanceAlongTrack < trackLength + Math.PI * trackRadius)
         {
            double angle = (distanceAlongTrack - trackLength) / trackRadius;
            x = trackLength / 2.0 + Math.sin(angle) * trackRadius;
            z = -trackRadius * Math.cos(angle);

            velocity.set(-track_linear_velocity * Math.cos(angle), 0.0, -track_linear_velocity * Math.sin(angle));
            temp_rotation.transform(velocity);

            for (int j = 0; j < numPointsPerTread; j++)
            {
               TrackGroundContactPoint tgc = trackPoints[i][j];

               tgc.dx_track.set(velocity.getX());
               tgc.dy_track.set(velocity.getY());
               tgc.dz_track.set(velocity.getZ());
            }
         }

         else if (distanceAlongTrack < trackLength + Math.PI * trackRadius + trackLength)
         {
            x = trackLength / 2.0 - (distanceAlongTrack - trackLength - Math.PI * trackRadius);
            z = trackRadius;

            // velocity.set(track_linear_velocity, 0.0, 0.0);
            // temp_rotation.transform(velocity);

            for (int j = 0; j < numPointsPerTread; j++)
            {
               TrackGroundContactPoint tgc = trackPoints[i][j];
               tgc.dx_track.set(-bottom_velocity.getX());
               tgc.dy_track.set(-bottom_velocity.getY());
               tgc.dz_track.set(-bottom_velocity.getZ());
            }
         }

         else
         {
            double angle = (distanceAlongTrack - 2.0 * trackLength - Math.PI * trackRadius) / trackRadius;
            x = -trackLength / 2.0 - Math.sin(angle) * trackRadius;
            z = trackRadius * Math.cos(angle);

            velocity.set(track_linear_velocity * Math.cos(angle), 0.0, track_linear_velocity * Math.sin(angle));
            temp_rotation.transform(velocity);

            for (int j = 0; j < numPointsPerTread; j++)
            {
               TrackGroundContactPoint tgc = trackPoints[i][j];

               tgc.dx_track.set(velocity.getX());
               tgc.dy_track.set(velocity.getY());
               tgc.dz_track.set(velocity.getZ());
            }
         }

         // System.out.println("(" + x + ", " + y + ", " + z + ")");
         for (int j = 0; j < numPointsPerTread; j++)
         {
            TrackGroundContactPoint tgc = trackPoints[i][j];

            double y = -trackWidth / 2.0 + (j) / ((numPointsPerTread - 1.0)) * trackWidth;

            temp_vector.set(x, y, z);
            if (track_rotation != null)
               track_rotation.transform(temp_vector);

            tgc.setOffsetJoint(temp_vector.getX() + track_offset.getX(), temp_vector.getY() + track_offset.getY(), temp_vector.getZ() + track_offset.getZ());
         }
      }
   }

   private Vector3d force = new Vector3d();

   private double computeTotalTrackLinearForce(double track_linear_position)
   {
      double total_force = 0.0;
      joint.getRotationToWorld(temp_rotation);
      if (track_rotation != null)
         temp_rotation.mul(track_rotation);    // temp_rotation = temp_rotation * rotation;
      temp_rotation.transpose();

      for (int i = 0; i < numTreads; i++)
      {
         double distanceAlongTrack = -track_linear_position + trackPerimeter * (i) / (numTreads);
         distanceAlongTrack = distanceAlongTrack % trackPerimeter;
         if (distanceAlongTrack < 0.0)
            distanceAlongTrack = distanceAlongTrack + trackPerimeter;

         // System.out.println(distanceAlongTrack);

         if (distanceAlongTrack < trackLength)
         {
            for (int j = 0; j < numPointsPerTread; j++)
            {
               GroundContactPoint gc = trackPoints[i][j];

               if (gc.isInContact())
               {
                  gc.getForce(force);
                  temp_rotation.transform(force);

                  total_force = total_force + force.getX();
               }
            }
         }

         else if (distanceAlongTrack < trackLength + Math.PI * trackRadius)
         {
            double angle = (distanceAlongTrack - trackLength) / trackRadius;

            for (int j = 0; j < numPointsPerTread; j++)
            {
               GroundContactPoint gc = trackPoints[i][j];

               if (gc.isInContact())
               {
                  gc.getForce(force);
                  temp_rotation.transform(force);

                  total_force = total_force + force.getX() * Math.cos(angle) + force.getZ() * Math.sin(angle);
               }
            }
         }

         else if (distanceAlongTrack < trackLength + Math.PI * trackRadius + trackLength)
         {
            for (int j = 0; j < numPointsPerTread; j++)
            {
               GroundContactPoint gc = trackPoints[i][j];

               if (gc.isInContact())
               {
                  gc.getForce(force);
                  temp_rotation.transform(force);

                  total_force = total_force - force.getX();
               }
            }
         }

         else
         {
            double angle = (distanceAlongTrack - 2.0 * trackLength - Math.PI * trackRadius) / trackRadius;

            for (int j = 0; j < numPointsPerTread; j++)
            {
               GroundContactPoint gc = trackPoints[i][j];

               if (gc.isInContact())
               {
                  gc.getForce(force);

                  temp_rotation.transform(force);
                  total_force = total_force - force.getX() * Math.cos(angle) - force.getZ() * Math.sin(angle);
               }
            }
         }
      }

      return total_force;
   }

   private double[] derivativeVector = new double[1];

   public double[] computeDerivativeVector()
   {
      // +++JEP OPTIMIZE
      track_linear_force.set(computeTotalTrackLinearForce(track_linear_position.getDoubleValue()));
      setGroundContactOffsetsAndVelocities(track_linear_position.getDoubleValue(), track_linear_velocity.getDoubleValue());

      // return new double[]{track_linear_velocity.val};

      derivativeVector[0] = track_linear_velocity.getDoubleValue();

      return derivativeVector;
   }

   public int getVectorSize()
   {
      return 1;
   }

   public DoubleYoVariable[] getOutputVariables()
   {
      return new DoubleYoVariable[] {track_linear_position};
   }


   public ArrayList<TrackGroundContactPoint> getTrackGroundContactPoints()
   {
      ArrayList<TrackGroundContactPoint> ret = new ArrayList<TrackGroundContactPoint>();

      for (int i = 0; i < trackPoints.length; i++)
      {
         for (int j = 0; j < trackPoints[i].length; j++)
         {
            ret.add(trackPoints[i][j]);
         }
      }

      return ret;
   }


}
