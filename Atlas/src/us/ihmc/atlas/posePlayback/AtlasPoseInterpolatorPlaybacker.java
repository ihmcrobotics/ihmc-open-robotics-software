package us.ihmc.atlas.posePlayback;

import java.io.Reader;
import java.io.StringReader;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.posePlayback.PlaybackPose;
import us.ihmc.avatar.posePlayback.PlaybackPoseSequence;
import us.ihmc.avatar.posePlayback.PlaybackPoseSequenceReader;
import us.ihmc.avatar.posePlayback.PoseCheckerCallback;
import us.ihmc.avatar.posePlayback.PoseInterpolatorPlaybacker;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;

public class AtlasPoseInterpolatorPlaybacker
{
   private final DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, DRCRobotModel.RobotTarget.SCS, false);
   
   public AtlasPoseInterpolatorPlaybacker()
   {
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      HumanoidFloatingRootJointRobot sdfRobot = robotModel.createHumanoidFloatingRootJointRobot(false);
      
      StringBuffer stringBuffer = new StringBuffer();
      stringBuffer
            .append("delayBeforePose poseDuration back_bkz back_bky back_bkx l_arm_shz l_arm_shx l_arm_ely l_arm_elx l_arm_wry l_arm_wrx neck_ry hokuyo_joint r_arm_shz r_arm_shx r_arm_ely r_arm_elx r_arm_wry r_arm_wrx l_leg_hpz l_leg_hpx l_leg_hpy l_leg_kny l_leg_aky l_leg_akx r_leg_hpz r_leg_hpx r_leg_hpy r_leg_kny r_leg_aky r_leg_akx\n");
      stringBuffer.append("0.300    1.000    0.130    -0.062   -0.363   -0.197   -0.091   1.906    0.510    0.190    -0.986   0.692    -2.269   -0.542   0.189    0.291    -1.044   1.148    0.101    0.728    0.430    -0.395   2.094    0.128    0.506    -0.903   0.239    0.300    1.454    0.677    0.091\n");    
      stringBuffer.append("0.300    1.000    -0.136   -0.164   0.278    -0.046   0.514    1.657    1.039    2.347    -0.957   -0.084   2.694    -1.251   0.225    0.867    -0.294   2.931    -0.528   1.132    -0.057   -0.082   2.050    -0.369   -0.790   -0.242   -0.177   0.464    2.139    -0.665   -0.531\n");   
      stringBuffer.append("0.300    1.000    -0.369   -0.552   -0.092   -0.592   -1.099   2.944    0.762    2.440    0.440    -0.063   0.947    -1.544   -1.404   0.670    -0.469   0.806    0.273    0.618    0.489    -1.159   0.287    0.071    -0.292   -0.080   -0.501   0.167    2.153    -0.324   0.340\n");    
      stringBuffer.append("0.300    1.000    -0.212   -0.408   0.465    0.330    0.979    3.128    0.277    0.886    -0.812   -0.314   -1.167   -1.106   1.316    3.033    -1.422   2.905    -0.448   0.047    -0.188   -1.058   0.260    0.491    -0.379   -0.036   0.290    -1.365   1.373    -0.302   0.118\n");    
      stringBuffer.append("0.300    1.000    0.059    0.096    0.068    -0.214   0.040    2.465    0.200    0.150    0.327    -0.260   1.340    0.219    -0.716   1.272    -0.095   1.491    -0.649   0.344    -0.352   -1.660   0.996    0.497    0.659    -1.067   0.208    0.269    0.826    -0.745   -0.694\n");   

      PlaybackPoseSequence sequence = new PlaybackPoseSequence(fullRobotModel);
      Reader reader = new StringReader(stringBuffer.toString());
      PlaybackPoseSequenceReader.appendFromFile(sequence, reader);

      //sequence.writeToOutputStream(fullRobotModel, System.out);

      boolean showGui = true;
      PoseInterpolatorPlaybacker.playASequence(sdfRobot, sequence, showGui, new PoseCheckerCallback()
      {
         @Override
         public void checkPose(PlaybackPose pose, PlaybackPose previousPose)
         {
            
         }
      });
   }
   
   public static void main(String[] args)
   {
      new AtlasPoseInterpolatorPlaybacker();
   }
}
