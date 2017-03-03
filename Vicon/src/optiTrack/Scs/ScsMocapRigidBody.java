package optiTrack.Scs;

import java.util.ArrayList;

import optiTrack.MocapMarker;
import optiTrack.MocapRigidBody;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.tools.thread.ThreadTools;

public class ScsMocapRigidBody
{
   private int id;

   private ArrayList<MocapMarker> listOfAssociatedMarkers;
   private YoVariableRegistry registry;
   private DoubleYoVariable xPos;
   private DoubleYoVariable yPos;
   private DoubleYoVariable zPos;
   private DoubleYoVariable qx;
   private DoubleYoVariable qy;
   private DoubleYoVariable qz;
   private DoubleYoVariable qw;
   private BooleanYoVariable isTracked;
   
   private DoubleYoVariable xVel;
   private DoubleYoVariable yVel;
   private DoubleYoVariable zVel;
   
   private Vector3D lastPosition = new Vector3D();
   private Vector3D currentPosition = new Vector3D();
   long lastTimeUpdated = System.nanoTime();
   
   boolean pause = false;

   public ScsMocapRigidBody(int id, Vector3D position, Quaternion orientation, ArrayList<MocapMarker> listOfAssociatedMarkers, boolean isTracked)
   {
      registry = new YoVariableRegistry("rb_" + id);
      xPos = new DoubleYoVariable("xPos", registry);
      yPos = new DoubleYoVariable("yPos", registry);
      zPos = new DoubleYoVariable("zPos", registry);
      qx = new DoubleYoVariable("qx", registry);
      qy = new DoubleYoVariable("qy", registry);
      qz = new DoubleYoVariable("qz", registry);
      qw = new DoubleYoVariable("qw", registry);
      this.isTracked = new BooleanYoVariable("", registry);
      
      
      xVel = new DoubleYoVariable("xVel", registry);
      yVel = new DoubleYoVariable("yVel", registry);
      zVel = new DoubleYoVariable("zVel", registry);

      this.id = id;

      xPos.set(position.getX());
      yPos.set(position.getY());
      zPos.set(position.getZ());
      qx.set(orientation.getX());
      qy.set(orientation.getY());
      qz.set(orientation.getZ());
      qw.set(orientation.getS());

      this.listOfAssociatedMarkers = listOfAssociatedMarkers;
      this.isTracked.set(isTracked);
   }

   public ScsMocapRigidBody(MocapRigidBody mocapRigidBody)
   {
      this(mocapRigidBody.getId(), mocapRigidBody.getPosition(), mocapRigidBody.getOrientation(), mocapRigidBody.getListOfAssociatedMarkers(),
            mocapRigidBody.dataValid);
   }

   public int getId()
   {
      return id;
   }
   
   public YoVariableRegistry getRegistry()
   {
      return registry;
   }
   
   public void pause(boolean pause)
   {
      this.pause = pause;
   }

   public void update(MocapRigidBody rb)
   {
      if(!pause)
      {
         xPos.set(rb.xPosition);
         yPos.set(rb.yPosition);
         zPos.set(rb.zPosition);
         qx.set(rb.qx);
         qy.set(rb.qy);
         qz.set(rb.qz);
         qw.set(rb.qw);
         
         currentPosition.set(rb.xPosition, rb.yPosition, rb.zPosition);
         
         long thisTime = System.currentTimeMillis();
         long timeStep = thisTime - lastTimeUpdated;
         lastTimeUpdated = System.currentTimeMillis();
         
         xVel.set((currentPosition.getX() - lastPosition.getX())/(timeStep/1000.0));
         yVel.set((currentPosition.getY() - lastPosition.getY())/(timeStep/1000.0));
         zVel.set((currentPosition.getZ() - lastPosition.getZ())/(timeStep/1000.0));
         
         lastPosition = new Vector3D(currentPosition.getX(), currentPosition.getY(), currentPosition.getZ());
         ThreadTools.sleep(3);
      }
   }

   public ArrayList<MocapMarker> getListOfAssociatedMarkers()
   {
      return listOfAssociatedMarkers;
   }

   public String toString()
   {
      String message = "\n";
      message = message + "RigidBody ID: " + id;
      message = message + "\nTracked : " + isTracked.getBooleanValue();
      message = message + "\nX: " + this.xPos.getDoubleValue() + " - Y: " + this.yPos.getDoubleValue() + " - Z: " + this.zPos.getDoubleValue();
      message = message + "\nqX: " + this.qx + " - qY: " + this.qy + " - qZ: " + this.qz + " - qW: " + this.qw;
      message = message + "\n# of Markers in rigid body: " + listOfAssociatedMarkers.size();

      for (int i = 0; i < listOfAssociatedMarkers.size(); i++)
      {
         message = message + "\nMarker " + i + " is at: " + listOfAssociatedMarkers.get(i).getPosition() + "  and has size: "
               + listOfAssociatedMarkers.get(i).getMarkerSize() + "m";
      }

      return message;
   }

   private final Quaternion quaternion = new Quaternion();

   public void getPose(RigidBodyTransform pose)
   {
      quaternion.set(qx.getDoubleValue(), qy.getDoubleValue(), qz.getDoubleValue(), qw.getDoubleValue());
      pose.setRotation(quaternion);
      pose.setTranslation(xPos.getDoubleValue(), yPos.getDoubleValue(), zPos.getDoubleValue());
   }
}

//~ Formatted by Jindent --- http://www.jindent.com
