package us.ihmc.darpaRoboticsChallenge.posePlayback;

public class PosePlaybackExampleSequence
{
   private static final double[] pose1 = new double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.10748031496062997, -0.8547244094488189, 0.0, 0.0, 0.0, -0.2165354330708662, 0.0, -0.8547244094488189, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
   private static final double[] pose2 = new double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.10748031496062997, -0.8547244094488189, 0.0, 0.0, 0.0, -0.2165354330708662, 0.0, -0.8547244094488189, 0.0, 0.0, 0.0, 0.0, -0.7283629133858267, 0.0, 0.0, 0.0, 0.0, 0.0, 0.7063789763779529, 0.0, 0.0, 0.0, 0.0};
   private static final double[] pose3 = new double[]{0.0, 0.8113385826771654, 0.0, 0.0, 0.0, 0.10748031496062997, -0.8547244094488189, 0.0, 0.0, 0.0, -0.2165354330708662, 0.0, -0.8547244094488189, 0.0, 0.0, 0.0, 0.0, -0.7283629133858267, 0.0, 0.0, 0.0, 0.0, 0.0, 0.7063789763779529, 0.0, 0.0, 0.0, 0.0};
   private static final double[] pose4 = new double[]{0.0, 0.8113385826771654, 0.0, 0.0, 0.5307086614173226, 0.10748031496062997, -0.8547244094488189, 0.0, 0.0, 0.0, -0.32000000000000006, 0.0, -0.8547244094488189, 0.0, 0.0, 0.0, 0.0, -0.7283629133858267, 0.0, 0.0, 0.0, 0.0, 0.0, 0.7063789763779529, 0.0, 0.0, 0.0, 0.0};

   
   public static PosePlaybackRobotPoseSequence createExampleSequenceEmpty()
   {
      PosePlaybackRobotPoseSequence sequence = new PosePlaybackRobotPoseSequence();
      return sequence;
   }
   
   public static PosePlaybackRobotPoseSequence createExampleSequenceOnePose()
   {
      PosePlaybackRobotPoseSequence sequence = new PosePlaybackRobotPoseSequence();
      
      sequence.addPose(new PosePlaybackRobotPose(pose1));
      
      return sequence;
   }
   
   public static PosePlaybackRobotPoseSequence createExampleSequenceTwoPoses()
   {
      PosePlaybackRobotPoseSequence sequence = new PosePlaybackRobotPoseSequence();
      
      sequence.addPose(new PosePlaybackRobotPose(pose1));
      sequence.addPose(new PosePlaybackRobotPose(pose2));
      
      return sequence;
   }
   
   public static PosePlaybackRobotPoseSequence createExampleSequenceThreePoses()
   {
      PosePlaybackRobotPoseSequence sequence = new PosePlaybackRobotPoseSequence();
      
      sequence.addPose(new PosePlaybackRobotPose(pose1));
      sequence.addPose(new PosePlaybackRobotPose(pose2));
      sequence.addPose(new PosePlaybackRobotPose(pose3));
      
      return sequence;
   }
   
   public static PosePlaybackRobotPoseSequence createExampleSequenceFourPoses()
   {
      PosePlaybackRobotPoseSequence sequence = new PosePlaybackRobotPoseSequence();
      
      sequence.addPose(new PosePlaybackRobotPose(pose1));
      sequence.addPose(new PosePlaybackRobotPose(pose2));
      sequence.addPose(new PosePlaybackRobotPose(pose3));
      sequence.addPose(new PosePlaybackRobotPose(pose4));
      
      return sequence;
   }
}
