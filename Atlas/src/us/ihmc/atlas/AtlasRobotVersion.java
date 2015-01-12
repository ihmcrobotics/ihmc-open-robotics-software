package us.ihmc.atlas;

import java.io.InputStream;

import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.atlas.parameters.AtlasPhysicalProperties;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.math.geometry.TransformTools;
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
   
   private final SideDependentList<RigidBodyTransform> handToWristTransform = new SideDependentList<RigidBodyTransform>();

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
            return "models/GFE/atlas_v4.sdf";
         case ATLAS_DUAL_ROBOTIQ:
            return "models/GFE/atlas_v4_robotiq_hands.sdf";
         case GAZEBO_ATLAS_NO_HANDS:
            return "models/GFE/atlas_v4.sdf";
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

   public RigidBodyTransform getHandToWristTransform(RobotSide side)
   {
      if (handToWristTransform.get(side) == null)
      {
         createTransforms();
      }
      return handToWristTransform.get(side);
   }
   
   /* Note: the class AtlasPhysicalProperties contains the offset between 
    * the last joint of the wrist and attachment plate where the gripper is bolted.
   *  Add another offset if you have a gripper.
   */
   private void createTransforms()
   {  
      for (RobotSide robotSide : RobotSide.values)
      {    
         RigidBodyTransform handToWrist = new RigidBodyTransform();
         handToWrist.set( AtlasPhysicalProperties.handAttachmentPlateToWristTransforms.get(robotSide) );
         
         if (hasRobotiqHands())
         {       
            // the previous transform actually represents the plate, NOT the hand.
            RigidBodyTransform handToPlate = new RigidBodyTransform( handToWrist );
            RigidBodyTransform plateToWrist =  TransformTools.createTransformFromTranslationAndEulerAngles(
                   
                  0.16, 0, 0,
                  robotSide.negateIfLeftSide(Math.toRadians(90)), 0, 0 ); 
            
            handToWrist.multiply( handToPlate, plateToWrist);
         }
         handToWristTransform.set(robotSide, handToWrist );
      }
   }

   public String getModelName()
   {
      return "atlas";
   }
}
