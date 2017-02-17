package optiTrack;

import java.util.ArrayList;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.vicon.QuaternionPose;

public class MocapRigidBody extends QuaternionPose
{
   private int id;

   private ArrayList<MocapMarker> listOfAssociatedMarkers;

   public MocapRigidBody(int id, Vector3D position, Quaternion orientation, ArrayList<MocapMarker> listOfAssociatedMarkers, boolean isTracked)
   {
      this.id = id;
      this.xPosition = (float) position.getX();
      this.yPosition = (float) position.getY();
      this.zPosition = (float) position.getZ();
      this.qw = (float) orientation.getS();
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

   public Vector3D getPosition()
   {
      return new Vector3D(xPosition, yPosition, zPosition);
   }

   public Quaternion getOrientation()
   {
      return new Quaternion(qx, qy, qz, qw);
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

   private final Quaternion tempQuaternion = new Quaternion();

   public void packPose(RigidBodyTransform pose)
   {
      tempQuaternion.set(qx, qy, qz, qw);
      pose.setRotation(tempQuaternion);
      pose.setTranslation(xPosition, yPosition, zPosition);
   }
}

//~ Formatted by Jindent --- http://www.jindent.com
