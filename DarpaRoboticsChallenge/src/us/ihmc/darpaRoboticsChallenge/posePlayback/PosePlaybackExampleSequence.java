package us.ihmc.darpaRoboticsChallenge.posePlayback;

public class PosePlaybackExampleSequence
{
   private static final double[] pose1 = new double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.10748031496062997, -0.8547244094488189, 0.0, 0.0, 0.0, -0.2165354330708662, 0.0, -0.8547244094488189, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
   private static final double[] pose2 = new double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.10748031496062997, -0.8547244094488189, 0.0, 0.0, 0.0, -0.2165354330708662, 0.0, -0.8547244094488189, 0.0, 0.0, 0.0, 0.0, -0.7283629133858267, 0.0, 0.0, 0.0, 0.0, 0.0, 0.7063789763779529, 0.0, 0.0, 0.0, 0.0};
   private static final double[] pose3 = new double[]{0.0, 0.8113385826771654, 0.0, 0.0, 0.0, 0.10748031496062997, -0.8547244094488189, 0.0, 0.0, 0.0, -0.2165354330708662, 0.0, -0.8547244094488189, 0.0, 0.0, 0.0, 0.0, -0.7283629133858267, 0.0, 0.0, 0.0, 0.0, 0.0, 0.7063789763779529, 0.0, 0.0, 0.0, 0.0};
   private static final double[] pose4 = new double[]{0.0, 0.8113385826771654, 0.0, 0.0, 0.5307086614173226, 0.10748031496062997, -0.8547244094488189, 0.0, 0.0, 0.0, -0.32000000000000006, 0.0, -0.8547244094488189, 0.0, 0.0, 0.0, 0.0, -0.7283629133858267, 0.0, 0.0, 0.0, 0.0, 0.0, 0.7063789763779529, 0.0, 0.0, 0.0, 0.0};

   public static PosePlaybackRobotPoseSequence createExampleSequenceOne()
   {
      PosePlaybackRobotPoseSequence sequence = new PosePlaybackRobotPoseSequence();
      
      PosePlaybackRobotPose pose = new PosePlaybackRobotPose(pose1);
      
      sequence.addPose(pose);
      
      return sequence;
   }
}
