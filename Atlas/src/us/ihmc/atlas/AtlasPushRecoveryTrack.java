package us.ihmc.atlas;

import us.ihmc.darpaRoboticsChallenge.DRCPushRecoveryTrack;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.graphics3DAdapter.GroundProfile;

import com.martiansoftware.jsap.JSAPException;
import com.yobotics.simulationconstructionset.util.FlatGroundProfile;
import com.yobotics.simulationconstructionset.util.ground.BumpyGroundProfile;

public class AtlasPushRecoveryTrack 
{
	private static final DRCRobotModel defaultModelForGraphicSelector = new AtlasRobotModel(AtlasRobotVersion.DRC_NO_HANDS, false, false);
	private static final boolean USE_BUMPY_GROUND = false;
	private final static double forceDuration = 100;
	
	public static void main(String[] args) throws JSAPException
	{
		  DRCRobotModel model = null;
		  final double groundHeight = 0.0;
		  
	      model = AtlasRobotModelFactory.selectModelFromFlag(args, false, false);
	      
	      if (model == null)
	         model = AtlasRobotModelFactory.selectModelFromGraphicSelector(defaultModelForGraphicSelector);

	      if (model == null)
	          throw new RuntimeException("No robot model selected");

	      GroundProfile groundProfile;
	      if (USE_BUMPY_GROUND)
	      {
	         groundProfile = createBumpyGroundProfile();
	      }
	      else
	      {
	         groundProfile = new FlatGroundProfile(groundHeight);
	      }	
	      
	      new DRCPushRecoveryTrack (groundProfile, model, forceDuration, groundHeight);
	}
	
	private static BumpyGroundProfile createBumpyGroundProfile()
    {
      double xAmp1 = 0.05, xFreq1 = 0.5, xAmp2 = 0.01, xFreq2 = 0.5;
      double yAmp1 = 0.01, yFreq1 = 0.07, yAmp2 = 0.05, yFreq2 = 0.37;
      BumpyGroundProfile groundProfile = new BumpyGroundProfile(xAmp1, xFreq1, xAmp2, xFreq2, yAmp1, yFreq1, yAmp2, yFreq2);
      return groundProfile;
    }

}
