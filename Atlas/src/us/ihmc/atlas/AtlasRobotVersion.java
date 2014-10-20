package us.ihmc.atlas;

import java.io.InputStream;

import us.ihmc.darpaRoboticsChallenge.handControl.DRCHandType;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;

import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;

public enum AtlasRobotVersion {
	ATLAS_INVISIBLE_CONTACTABLE_PLANE_HANDS, DRC_NO_HANDS, DRC_HANDS, DRC_EXTENDED_HANDS, DRC_HOOKS, DRC_TASK_HOSE, DRC_EXTENDED_HOOKS, ATLAS_ROBOTIQ, GAZEBO_ATLAS_NO_HANDS;

	private static String[] resourceDirectories;
	private final SideDependentList<Transform> offsetHandFromWrist = new SideDependentList<Transform>(); 
	
	public DRCHandType getHandModel() {
		switch (this) 
		{
		case DRC_HANDS:
		case DRC_EXTENDED_HANDS:
		case DRC_TASK_HOSE:
			return DRCHandType.IROBOT;

		case DRC_HOOKS:
			return DRCHandType.HOOK;
			
		case ATLAS_ROBOTIQ:
			return DRCHandType.ROBOTIQ;
			
		case DRC_NO_HANDS:
		case ATLAS_INVISIBLE_CONTACTABLE_PLANE_HANDS:
		case DRC_EXTENDED_HOOKS:
      case GAZEBO_ATLAS_NO_HANDS:
		default:
			return DRCHandType.NONE;
		}
	}

	public boolean hasArmExtensions() {
      switch (this)
      {
      case DRC_EXTENDED_HANDS:
         return true;

      default:
         return false;
      }
	}
	
	public boolean hasIrobotHands() {
		return getHandModel() == DRCHandType.IROBOT;
	}
	
	public boolean hasHookHands() {
		return getHandModel() == DRCHandType.HOOK;
	}
	
	public boolean hasRobotiqHands() {
		return getHandModel() == DRCHandType.ROBOTIQ;
	}

	public String getSdfFile() {
		switch (this)
	      {
	         case ATLAS_INVISIBLE_CONTACTABLE_PLANE_HANDS :
	         case DRC_NO_HANDS:
	            return "models/GFE/drc_no_hands.sdf";
	         case ATLAS_ROBOTIQ:
	            return "models/GFE/drc_task_hose_robotiq.sdf";
	         case DRC_HANDS:
	            return "models/GFE/drc_hands.sdf";
	         case DRC_EXTENDED_HANDS:
	            return "models/GFE/drc_extended_hands.sdf";
	         case DRC_HOOKS:
	            return "models/GFE/drc_hooks.sdf";
	         case DRC_TASK_HOSE:
	            return "models/GFE/drc_task_hose.sdf";
	         case DRC_EXTENDED_HOOKS:
	            return "models/GFE/drc_extended_hooks.sdf";
            case GAZEBO_ATLAS_NO_HANDS:
               return "models/GFE/gazebo_atlas.sdf";
	         default :
	            throw new RuntimeException("AtlasRobotVersion: Unimplemented enumeration case : " + this);
	      }
	}

	public String[] getResourceDirectories() {

		if(resourceDirectories == null)
		{
			resourceDirectories = new String[] {
               "models/GFE/gazebo_models/",
               "models/GFE/gazebo_models/atlas_description/",
               "models/GFE/gazebo_models/multisense_sl_description/",
               "models/GFE/gazebo_models/irobot_hand_description/",
               "models/GFE/gazebo_models/robotiq/"
				};
		}
		return resourceDirectories;
	}

	public InputStream getSdfFileAsStream() {
		return getClass().getClassLoader().getResourceAsStream(getSdfFile());
	}

	public Transform getOffsetFromWrist(RobotSide side) {
		
		if(offsetHandFromWrist.get(side) == null)
		{
			createTransforms();
		}
		return offsetHandFromWrist.get(side);
	}
	
	private void createTransforms()
	{
		for(RobotSide robotSide : RobotSide.values)
		{
			Vector3f centerOfHandToWristTranslation = new Vector3f();
			float[] angles = new float[3];
			if (hasArmExtensions())
			{
				centerOfHandToWristTranslation = new Vector3f(0.31f, (float) robotSide.negateIfLeftSide(0f), 0f);
				angles[0] = (float) robotSide.negateIfLeftSide(Math.toRadians(90));
				angles[1] = 0.0f;
				angles[2] = (float) robotSide.negateIfLeftSide(Math.toRadians(0));
			}
			
			else if (hasIrobotHands() )
			{
				centerOfHandToWristTranslation = new Vector3f(0.1f, (float) robotSide.negateIfLeftSide(0f), 0f);
				angles[0] = (float) robotSide.negateIfLeftSide(Math.toRadians(-90));
				angles[1] = 0.0f;    
				angles[2] = (float) robotSide.negateIfLeftSide(Math.toRadians(0));
			}
			else if (hasRobotiqHands())
         {
            centerOfHandToWristTranslation = new Vector3f(0.16f, (float) robotSide.negateIfLeftSide(0f), 0f);
            angles[0] = (float) robotSide.negateIfLeftSide(Math.toRadians(90));
            angles[1] = 0.0f;    
            angles[2] = (float) robotSide.negateIfLeftSide(Math.toRadians(0));
         }
			else if (hasHookHands())
			{
				centerOfHandToWristTranslation = new Vector3f(0.1f, (float) robotSide.negateIfLeftSide(0f), 0f);
				angles[0] = (float) robotSide.negateIfLeftSide(Math.toRadians(-90));
				angles[1] = 0.0f;   
				angles[2] = (float) robotSide.negateIfLeftSide(Math.toRadians(0));
			}
			Quaternion centerOfHandToWristRotation = new Quaternion(angles);
			offsetHandFromWrist.set(robotSide, new Transform(centerOfHandToWristTranslation, centerOfHandToWristRotation));
		}
	}

   public String getModelName()
   {
      return "atlas";
   }
}
