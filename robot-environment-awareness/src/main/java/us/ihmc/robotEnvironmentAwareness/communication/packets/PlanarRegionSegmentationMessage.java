package us.ihmc.robotEnvironmentAwareness.communication.packets;

import java.util.Arrays;
import java.util.List;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.jOctoMap.key.OcTreeKey;

public class PlanarRegionSegmentationMessage extends Packet<PlanarRegionSegmentationMessage>
{
   public int id;
   public Point3D32 origin;
   public Vector3D32 normal;
   public OcTreeKeyMessage[] nodeKeys;
   public Point3D32[] hitLocations;

   public PlanarRegionSegmentationMessage()
   {
   }

   public PlanarRegionSegmentationMessage(int id, Point3D origin, Vector3D normal, OcTreeKeyMessage[] regionNodeKeys, List<Point3D> hitLocations)
   {
      this.id = id;
      this.origin = new Point3D32(origin);
      this.normal = new Vector3D32(normal);
      this.nodeKeys = regionNodeKeys;
      this.hitLocations = hitLocations.stream().map(Point3D32::new).toArray(Point3D32[]::new);
   }

   public PlanarRegionSegmentationMessage(int id, Point3D32 origin, Vector3D32 normal, OcTreeKeyMessage[] regionNodeKeys, Point3D32[] hitLocations)
   {
      this.id = id;
      this.origin = origin;
      this.normal = normal;
      this.nodeKeys = regionNodeKeys;
      this.hitLocations = hitLocations;
   }

   @Override
   public void set(PlanarRegionSegmentationMessage other)
   {
      id = other.id;
      origin = new Point3D32(other.origin);
      normal = new Vector3D32(other.normal);
      nodeKeys = new OcTreeKeyMessage[other.nodeKeys.length];
      for (int i = 0; i < nodeKeys.length; i++)
      {
         nodeKeys[i] = new OcTreeKeyMessage();
         nodeKeys[i].set(other.nodeKeys[i]);
      }
      hitLocations = Arrays.stream(other.hitLocations).map(Point3D32::new).toArray(Point3D32[]::new);
      setPacketInformation(other);
   }

   public int getRegionId()
   {
      return id;
   }

   public Point3D32 getOrigin()
   {
      return origin;
   }

   public Vector3D32 getNormal()
   {
      return normal;
   }

   public int getNumberOfNodes()
   {
      return nodeKeys.length;
   }

   public OcTreeKeyMessage getNodeKey(int index)
   {
      return nodeKeys[index];
   }

   public Point3D32[] getHitLocations()
   {
      return hitLocations;
   }

   public Point3D32 getHitLocation(int index)
   {
      return hitLocations[index];
   }

   public void getNodeKey(int index, OcTreeKey nodeKeyToPack)
   {
      nodeKeyToPack.set(nodeKeys[index]);
   }

   public void getHitLocation(int index, Point3D hitLocationToPack)
   {
      hitLocationToPack.set(hitLocations[index]);
   }

   @Override
   public boolean epsilonEquals(PlanarRegionSegmentationMessage other, double epsilon)
   {
      if (id != other.id)
         return false;
      if (!origin.epsilonEquals(other.origin, (float) epsilon))
         return false;
      if (!normal.epsilonEquals(other.normal, (float) epsilon))
         return false;
      if (!Arrays.equals(nodeKeys, other.nodeKeys))
         return false;
      if (hitLocations.length != other.hitLocations.length)
         return false;
      for (int i = 0; i < hitLocations.length; i++)
      {
         if (!hitLocations[i].epsilonEquals(other.hitLocations[i], (float) epsilon))
            return false;
      }
      return true;
   }
}
