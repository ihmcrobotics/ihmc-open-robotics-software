package us.ihmc.avatar.posePlayback;

import static us.ihmc.robotics.Assert.*;

import java.io.ByteArrayOutputStream;
import java.io.StringReader;
import java.util.Random;

import org.junit.jupiter.api.Test;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.robotModels.FullHumanoidRobotModel;

public abstract class PlaybackPoseSequenceDRCTest implements MultiRobotTestInterface
{
   @Test
   public void testReadAndWriteWithRandomSequence()
   {
      DRCRobotModel robotModel = getRobotModel();
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();

      int numberOfPoses = 5;
      double delay = 0.3;
      double trajectoryTime = 1.0;

      Random random = new Random(1776L);
      PlaybackPoseSequence sequence = PosePlaybackExampleSequence.createRandomPlaybackPoseSequence(random, fullRobotModel, numberOfPoses, delay, trajectoryTime);

      ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
      PlaybackPoseSequenceWriter.writeToOutputStream(sequence, outputStream);

      String outputAsString = outputStream.toString();

//      System.out.println(outputAsString);

      PlaybackPoseSequence sequenceTwo = new PlaybackPoseSequence(fullRobotModel);

      StringReader reader = new StringReader(outputAsString);
      PlaybackPoseSequenceReader.appendFromFile(sequenceTwo, reader);

      double jointEpsilon = 1e-7;
      double timeEpsilon = 1e-7;
      assertTrue(sequence.epsilonEquals(sequenceTwo, jointEpsilon, timeEpsilon));
   }
}
