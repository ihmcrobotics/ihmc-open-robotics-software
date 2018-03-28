package us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory;

import gnu.trove.list.array.TByteArrayList;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;

/**
 * Point, Cylinder, Sphere, Box will be converted into this message.
 */
public class ReachingManifoldMessage extends Packet<ReachingManifoldMessage>
{
   public static final byte CONFIGURATION_SPACE_NAME_X = 0;
   public static final byte CONFIGURATION_SPACE_NAME_Y = 1;
   public static final byte CONFIGURATION_SPACE_NAME_Z = 2;
   public static final byte CONFIGURATION_SPACE_NAME_ROLL = 3;
   public static final byte CONFIGURATION_SPACE_NAME_PITCH = 4;
   public static final byte CONFIGURATION_SPACE_NAME_YAW = 5;

   public long endEffectorNameBasedHashCode;
   public Point3D manifoldOriginPosition;
   public Quaternion manifoldOriginOrientation;

   public TByteArrayList manifoldConfigurationSpaceNames = new TByteArrayList();
   public TDoubleArrayList manifoldLowerLimits = new TDoubleArrayList();
   public TDoubleArrayList manifoldUpperLimits = new TDoubleArrayList();

   public ReachingManifoldMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   @Override
   public void set(ReachingManifoldMessage other)
   {
      endEffectorNameBasedHashCode = other.endEffectorNameBasedHashCode;
      manifoldOriginPosition = new Point3D(other.manifoldOriginPosition);
      manifoldOriginOrientation = new Quaternion(other.manifoldOriginOrientation);
      MessageTools.copyData(other.manifoldConfigurationSpaceNames, manifoldConfigurationSpaceNames);
      MessageTools.copyData(other.manifoldLowerLimits, manifoldLowerLimits);
      MessageTools.copyData(other.manifoldUpperLimits, manifoldUpperLimits);
      setPacketInformation(other);
   }

   public void setOrigin(Point3D position, Quaternion orientation)
   {
      manifoldOriginPosition = new Point3D(position);
      manifoldOriginOrientation = new Quaternion(orientation);
   }

   public void setMenifoldAPoint()
   {
      byte[] configurations = {};
      double[] lowerLimits = {};
      double[] upperLimits = {};

      setManifold(configurations, lowerLimits, upperLimits);
   }

   public void setManifold(byte[] configurationSpaces, double[] lowerLimits, double[] upperLimits)
   {
      if (configurationSpaces.length != lowerLimits.length || configurationSpaces.length != upperLimits.length || lowerLimits.length != upperLimits.length)
         throw new RuntimeException("Inconsistent array lengths: configurationSpaces = " + configurationSpaces.length);

      this.manifoldConfigurationSpaceNames.reset();
      this.manifoldLowerLimits.reset();
      this.manifoldUpperLimits.reset();
      this.manifoldConfigurationSpaceNames.add(configurationSpaces);
      this.manifoldLowerLimits.add(lowerLimits);
      this.manifoldUpperLimits.add(upperLimits);
   }

   @Override
   public boolean epsilonEquals(ReachingManifoldMessage other, double epsilon)
   {
      if (endEffectorNameBasedHashCode != other.endEffectorNameBasedHashCode)
         return false;
      if (!manifoldOriginPosition.epsilonEquals(other.manifoldOriginPosition, epsilon))
         return false;
      if (!manifoldOriginOrientation.epsilonEquals(other.manifoldOriginOrientation, epsilon))
         return false;
      if (!manifoldConfigurationSpaceNames.equals(other.manifoldConfigurationSpaceNames))
         return false;
      if (!MessageTools.epsilonEquals(manifoldLowerLimits, other.manifoldLowerLimits, epsilon))
         return false;
      if (!MessageTools.epsilonEquals(manifoldUpperLimits, other.manifoldUpperLimits, epsilon))
         return false;
      return true;
   }

   public Point3D getOriginPosition()
   {
      return manifoldOriginPosition;
   }

   public Quaternion getOriginOrientation()
   {
      return manifoldOriginOrientation;
   }

   public long getRigidBodyNameBasedHashCode()
   {
      return endEffectorNameBasedHashCode;
   }

   public int getDimensionOfManifold()
   {
      if (manifoldConfigurationSpaceNames == null)
         return 0;
      return manifoldConfigurationSpaceNames.size();
   }

   public byte getDegreeOfManifold(int i)
   {
      return manifoldConfigurationSpaceNames.get(i);
   }

   public double getUpperLimit(int i)
   {
      return manifoldUpperLimits.get(i);
   }

   public double getLowerLimit(int i)
   {
      return manifoldLowerLimits.get(i);
   }
}
