package us.ihmc.atlas;

import java.io.InputStream;

import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.wholeBodyController.DRCHandType;

import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;

public enum AtlasRobotVersion
{
   ATLAS_INVISIBLE_CONTACTABLE_PLANE_HANDS,
   DRC_NO_HANDS,
   ATLAS_DUAL_ROBOTIQ,
   GAZEBO_ATLAS_NO_HANDS;

   private static String[] resourceDirectories;
   private final SideDependentList<Transform> offsetHandFromAttachmentPlate = new SideDependentList<Transform>();

   public DRCHandType getHandModel()
   {
      switch (this)
      {
         case ATLAS_DUAL_ROBOTIQ:
            return DRCHandType.ROBOTIQ;

         case DRC_NO_HANDS:
         case ATLAS_INVISIBLE_CONTACTABLE_PLANE_HANDS:
         case GAZEBO_ATLAS_NO_HANDS:
         default:
            return DRCHandType.NONE;
      }
   }
   
   public double getDistanceAttachmentPlateHand()
   {
      switch (this)
      {
         case ATLAS_DUAL_ROBOTIQ:
            return 0.16;
         default:
            return 0.0;
      }
   }

   public boolean hasRobotiqHands()
   {
      return getHandModel() == DRCHandType.ROBOTIQ;
   }

   public String getSdfFile()
   {
      switch (this)
      {
         case ATLAS_INVISIBLE_CONTACTABLE_PLANE_HANDS:
         case DRC_NO_HANDS:
            return "models/GFE/atlas_unplugged_v4.sdf";
         case ATLAS_DUAL_ROBOTIQ:
            return "models/GFE/atlas_unplugged_v4_dual_robotiq.sdf";
         case GAZEBO_ATLAS_NO_HANDS:
            return "models/GFE/atlas_unplugged_v4.sdf";
         default:
            throw new RuntimeException("AtlasRobotVersion: Unimplemented enumeration case : " + this);
      }
   }

   public String[] getResourceDirectories()
   {

      if (resourceDirectories == null)
      {
         resourceDirectories = new String[] { "models/GFE/" };
      }
      return resourceDirectories;
   }

   public InputStream getSdfFileAsStream()
   {
      return getClass().getClassLoader().getResourceAsStream(getSdfFile());
   }

   public Transform getOffsetFromAttachmentPlate(RobotSide side)
   {
      if (offsetHandFromAttachmentPlate.get(side) == null)
      {
         createTransforms();
      }
      return offsetHandFromAttachmentPlate.get(side);
   }

   private void createTransforms()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         Vector3f centerOfHandToWristTranslation = new Vector3f();
         float[] angles = new float[3];
         if (hasRobotiqHands())
         {
            centerOfHandToWristTranslation = new Vector3f((float)getDistanceAttachmentPlateHand(), robotSide.negateIfLeftSide(0f), 0f);
            angles[0] = (float) robotSide.negateIfLeftSide(Math.toRadians(0));
            angles[1] = 0.0f;
            angles[2] = (float) robotSide.negateIfLeftSide(Math.toRadians(0));
         }
         Quaternion centerOfHandToWristRotation = new Quaternion(angles);
         offsetHandFromAttachmentPlate.set(robotSide, new Transform(centerOfHandToWristTranslation, centerOfHandToWristRotation));
      }
   }

   public String getModelName()
   {
      return "atlas";
   }
}

