package optiTrack;

import java.util.ArrayList;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.vicon.QuaternionPose;

public class MocapRigidBody extends QuaternionPose
{
   private int id;

   private ArrayList<MocapMarker> listOfAssociatedMarkers;

   public MocapRigidBody(int id, Vector3d position, Quat4d orientation, ArrayList<MocapMarker> listOfAssociatedMarkers, boolean isTracked)
   {
      this.id = id;
      this.xPosition = (float) position.getX();
      this.yPosition = (float) position.getY();
      this.zPosition = (float) position.getZ();
      this.qw = (float) orientation.getW();
      this.qx = (float) orientation.getX();
      this.qy = (float) orientation.getY();
      this.qz = (float) orientation.getZ();

      this.listOfAssociatedMarkers = listOfAssociatedMarkers;
      this.dataValid = isTracked;
   }

   public int getId()
   {
      return id;
   }

   public Vector3d getPosition()
   {
      return new Vector3d(xPosition, yPosition, zPosition);
   }

   public Quat4d getOrientation()
   {
      return new Quat4d(qx, qy, qz, qw);
   }

   public ArrayList<MocapMarker> getListOfAssociatedMarkers()
   {
      return listOfAssociatedMarkers;
   }

   public String toString()
   {
      String message = "\n";
      message = message + "RigidBody ID: " + id;
      message = message + "\nTracked : " + dataValid;
      message = message + "\nX: " + this.xPosition + " - Y: " + this.yPosition + " - Z: " + this.zPosition;
      message = message + "\nqX: " + this.qx + " - qY: " + this.qy + " - qZ: " + this.qz + " - qW: " + this.qw;
      message = message + "\n# of Markers in rigid body: " + listOfAssociatedMarkers.size();

      for (int i = 0; i < listOfAssociatedMarkers.size(); i++)
      {
         message = message + "\nMarker " + i + " is at: " + listOfAssociatedMarkers.get(i).getPosition() + "  and has size: "
               + listOfAssociatedMarkers.get(i).getMarkerSize() + "m";
      }

      return message;
   }

   public void packPose(RigidBodyTransform pose)
   {
      pose.setRotationWithQuaternion(qx, qy, qz, qw);
      pose.setTranslation(xPosition, yPosition, zPosition);
   }
}

//~ Formatted by Jindent --- http://www.jindent.com
