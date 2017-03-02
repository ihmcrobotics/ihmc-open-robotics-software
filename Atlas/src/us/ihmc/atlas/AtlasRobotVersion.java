package us.ihmc.atlas;

import java.io.InputStream;

import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;

import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.wholeBodyController.DRCHandType;

public enum AtlasRobotVersion
{
   ATLAS_UNPLUGGED_V5_NO_FOREARMS,
   ATLAS_UNPLUGGED_V5_NO_HANDS,
   ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ,
   ATLAS_UNPLUGGED_V5_INVISIBLE_CONTACTABLE_PLANE_HANDS,
   ATLAS_UNPLUGGED_V5_ROBOTIQ_AND_SRI,
   ATLAS_UNPLUGGED_V5_TROOPER;

   private static String[] resourceDirectories;
   private final SideDependentList<Transform> offsetHandFromAttachmentPlate = new SideDependentList<Transform>();

   public DRCHandType getHandModel()
   {
      switch (this)
      {
      case ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ:
      case ATLAS_UNPLUGGED_V5_TROOPER:
         return DRCHandType.ROBOTIQ;
      case ATLAS_UNPLUGGED_V5_ROBOTIQ_AND_SRI:
         return DRCHandType.ROBOTIQ_AND_SRI;
      case ATLAS_UNPLUGGED_V5_NO_HANDS:
      case ATLAS_UNPLUGGED_V5_NO_FOREARMS:
      case ATLAS_UNPLUGGED_V5_INVISIBLE_CONTACTABLE_PLANE_HANDS:
      default:
         return DRCHandType.NONE;
      }
   }

   public double getDistanceAttachmentPlateHand()
   {
      switch (this)
      {
      case ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ:
      case ATLAS_UNPLUGGED_V5_ROBOTIQ_AND_SRI:
         return 0.16;
      default:
         return 0.0;
      }
   }

   public boolean hasRobotiqHands()
   {
      return getHandModel() == DRCHandType.ROBOTIQ || getHandModel() == DRCHandType.ROBOTIQ_AND_SRI;
   }

   public String getSdfFile()
   {
      switch (this)
      {
      case ATLAS_UNPLUGGED_V5_INVISIBLE_CONTACTABLE_PLANE_HANDS:
      case ATLAS_UNPLUGGED_V5_NO_HANDS:
         return "models/GFE/atlas_unplugged_v5.sdf";
      case ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ:
      case ATLAS_UNPLUGGED_V5_ROBOTIQ_AND_SRI:
         return "models/GFE/atlas_unplugged_v5_dual_robotiq.sdf";
      case ATLAS_UNPLUGGED_V5_NO_FOREARMS:
         return "models/GFE/atlas_unplugged_v5_no_forearms.sdf";
      case ATLAS_UNPLUGGED_V5_TROOPER:
         return "models/GFE/atlas_unplugged_v5_trooper.sdf";
      default:
         throw new RuntimeException("AtlasRobotVersion: Unimplemented enumeration case : " + this);
      }
   }

   public String[] getResourceDirectories()
   {
      if (resourceDirectories == null)
      {
         resourceDirectories = new String[] {"models/GFE/"};
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
            centerOfHandToWristTranslation = new Vector3f((float) getDistanceAttachmentPlateHand(), robotSide.negateIfLeftSide(0f), 0f);
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
