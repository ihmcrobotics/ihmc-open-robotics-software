package us.ihmc.darpaRoboticsChallenge.posePlayback;

import static org.junit.Assert.assertTrue;

import java.io.ByteArrayOutputStream;
import java.io.StringReader;
import java.util.Random;

import org.junit.Test;

import us.ihmc.SdfLoader.JaxbSDFLoader;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.darpaRoboticsChallenge.DRCLocalConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCRobotSDFLoader;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;

public class PosePlaybackRobotPoseSequenceTest
{
   @Test
   public void testReadAndWriteWithRandomSequence()
   {
      DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.DRC_NO_HANDS, DRCLocalConfigParameters.RUNNING_ON_REAL_ROBOT);

      DRCRobotJointMap jointMap = robotModel.getJointMap();
      JaxbSDFLoader sdfLoader = DRCRobotSDFLoader.loadDRCRobot(jointMap, false);

      SDFFullRobotModel fullRobotModel = sdfLoader.createFullRobotModel(jointMap);

      int numberOfPoses = 5;
      double delay = 0.3;
      double trajectoryTime = 1.0;

      Random random = new Random(1776L);
      PosePlaybackRobotPoseSequence sequence = PosePlaybackExampleSequence.createRandomPlaybackPoseSequence(random, fullRobotModel, numberOfPoses, delay,
                                                  trajectoryTime);

      ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
      PosePlaybackRobotPoseSequence.writeToOutputStream(sequence, outputStream);

      String outputAsString = outputStream.toString();

//      System.out.println(outputAsString);

      PosePlaybackRobotPoseSequence sequenceTwo = new PosePlaybackRobotPoseSequence(fullRobotModel);

      StringReader reader = new StringReader(outputAsString);
      PosePlaybackRobotPoseSequence.appendFromFile(sequenceTwo, reader);

      double jointEpsilon = 1e-7;
      double timeEpsilon = 1e-7;
      assertTrue(sequence.epsilonEquals(sequenceTwo, jointEpsilon, timeEpsilon));

   }

}
