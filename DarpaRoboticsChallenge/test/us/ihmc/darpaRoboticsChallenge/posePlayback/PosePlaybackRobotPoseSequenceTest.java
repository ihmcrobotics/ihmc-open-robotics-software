package us.ihmc.darpaRoboticsChallenge.posePlayback;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;

import org.junit.Test;

public class PosePlaybackRobotPoseSequenceTest
{
   @Test
   public void testSimpleReadAndWrite()
   {
      String firstFileWrite = "firstFileWrite";
      String secondFileWrite = "secondFileWrite";
      
      PosePlaybackRobotPoseSequence poseSequenceWrite = new PosePlaybackRobotPoseSequence();

      double[][] poses = {
            { 0.0, 0.0, 0.0, 0.0, 0.0, 0.10748031496062997, -0.8547244094488189, 0.0, 0.0, 0.0, -0.2165354330708662, 0.0, -0.8547244094488189, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
            { 0.0, 0.0, 0.0, 0.0, 0.0, 0.10748031496062997, -0.8547244094488189, 0.0, 0.0, 0.0, -0.2165354330708662, 0.0, -0.8547244094488189, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
            { 0.0, 0.8113385826771654, 0.0, 0.0, 0.0, 0.10748031496062997, -0.8547244094488189, 0.0, 0.0, 0.0, -0.2165354330708662, 0.0, -0.8547244094488189,
                  0.0, 0.0, 0.0, 0.0, -0.7283629133858267, 0.0, 0.0, 0.0, 0.0, 0.0, 0.7063789763779529, 0.0, 0.0, 0.0, 0.0 },
            { 0.0, 0.8113385826771654, 0.0, 0.0, 0.5307086614173226, 0.10748031496062997, -0.8547244094488189, 0.0, 0.0, 0.0, -0.32000000000000006, 0.0,
                  -0.8547244094488189, 0.0, 0.0, 0.0, 0.0, -0.7283629133858267, 0.0, 0.0, 0.0, 0.0, 0.0, 0.7063789763779529, 0.0, 0.0, 0.0, 0.0 } };
      
      for(double[] pose : poses)
      {
         poseSequenceWrite.addPose(pose);
      }
      
      poseSequenceWrite.writeToFile(firstFileWrite);

      //create the object
      PosePlaybackRobotPoseSequence poseSequenceFirstRead = new PosePlaybackRobotPoseSequence();
      
      //read data from file into the object
      poseSequenceFirstRead.appendFromFile(firstFileWrite);
      
      assertTrue(poseSequencesAreEqual(poseSequenceWrite, poseSequenceFirstRead));
      
      double[] additionalPose = new double[] { 3.14, 0.0, 0.0, 0.0, 0.0, 0.10748031496062997, -0.8547244094488189, 0.0, 0.0, 0.0, -0.2165354330708662, 0.0,
            -0.8547244094488189, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.5 };
      
      poseSequenceFirstRead.addPose(additionalPose);
      
      //write to file
      poseSequenceFirstRead.writeToFile(secondFileWrite);
      
      //copy to another sequence and verify read works after appending file
      PosePlaybackRobotPoseSequence poseSequenceSecondRead = new PosePlaybackRobotPoseSequence();
      
      poseSequenceSecondRead.appendFromFile(secondFileWrite);
      
      assertTrue(poseSequencesAreEqual(poseSequenceFirstRead, poseSequenceSecondRead));
   }
   
   @Test
   private boolean poseSequencesAreEqual(PosePlaybackRobotPoseSequence expected, PosePlaybackRobotPoseSequence actual)
   {
      ArrayList<PosePlaybackRobotPose> expectedPose = expected.getPoseSequence();
      ArrayList<PosePlaybackRobotPose> actualPose = actual.getPoseSequence();
      
      // check that number of poses is equal
      if(expectedPose.size() != actualPose.size())
      {
         System.err.println("Number of poses does not match between read/write compare");
         return false;
      }
      
      for(int i = 0; i < expectedPose.size(); i++)
      {
         double[] expectedAngles = expectedPose.get(i).getJointAngles();
         double[] actualAngles = actualPose.get(i).getJointAngles();
         
         if(expectedAngles.length != actualAngles.length)
         {
            System.err.println("Number of joint angles does not match between read/write compare");
            return false;
         }
         
         for(int j = 0; j < expectedAngles.length; j++)
         {
            if(Math.abs(expectedAngles[j] - actualAngles[j]) > 0.00051) // numbers are rounded to nearest third decimal, the additional 0.00001 is the epsilon error
            {
               System.err.println("Joint angle values do not match between read/write compare, within rounding error");
               return false;
            }
         }
      }
      
      return true;
   }
}
