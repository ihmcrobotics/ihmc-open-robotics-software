package us.ihmc.robotEnvironmentAwareness.slam;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import gnu.trove.list.array.TLongArrayList;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

public class SLAMHistory
{
   private static final String SLAM_HISTORY_EXTENSION = ".csv";
   private static final String FILE_NAME_HEADER = "slam_history_";
   /**
    * <p>
    * frame id: time stamp
    * <p>
    * original sensor pose
    * <p>
    * final sensor pose
    * <p>
    * rigid body transformer
    * <p>
    * computation time
    * <p>
    * frame waiting time: (time period between arrived and start to compute)
    * <p>
    * number of iterations
    * <p>
    * number of surfel
    * <p>
    * initial distance
    * <p>
    * final distance
    * <p>
    * parameters
    */
   private final TLongArrayList timeStamps = new TLongArrayList();
   private final List<RigidBodyTransformReadOnly> uncorrectedLocalFrames = new ArrayList<>();
   private final List<RigidBodyTransformReadOnly> correctedLocalFrames = new ArrayList<>();

   private final List<DriftCorrectionResult> driftCorrectionResults = new ArrayList<DriftCorrectionResult>();

   public void clearHistory()
   {
      timeStamps.clear();
      uncorrectedLocalFrames.clear();
      correctedLocalFrames.clear();
      driftCorrectionResults.clear();
   }

   public void addLatestFrameHistory(SLAMFrame latestFrame)
   {
      timeStamps.add(latestFrame.getTimeStamp());
      uncorrectedLocalFrames.add(latestFrame.getUncorrectedLocalPoseInWorld());
      correctedLocalFrames.add(latestFrame.getCorrectedLocalPoseInWorld());
   }

   public void addDriftCorrectionHistory(DriftCorrectionResult latestResult)
   {
      DriftCorrectionResult result = new DriftCorrectionResult();
      result.set(latestResult);
      driftCorrectionResults.add(result);
   }

   //TODO : add RigidBodyTransform.
   public void export(Path path)
   {
      File sensorPoseFile = new File(path.toFile(), FILE_NAME_HEADER + System.currentTimeMillis() + SLAM_HISTORY_EXTENSION);
      FileWriter sensorPoseFileWriter;
      try
      {
         sensorPoseFileWriter = new FileWriter(sensorPoseFile);
         StringBuilder builder = new StringBuilder("");
         builder.append("timeStamps" + ",");
         builder.append("success" + ",");
         builder.append("computationTime (sec)" + ",");
         builder.append("icpIterations" + ",");
         builder.append("numberOfSurfels" + ",");
         builder.append("numberOfCorrespondances" + ",");
         builder.append("initialDistance" + ",");
         builder.append("finalDistance" + "\n");
         for (int i = 0; i < timeStamps.size(); i++)
         {
            builder.append(timeStamps.get(i) + ",");
            DriftCorrectionResult driftCorrectionResult = driftCorrectionResults.get(i);
            builder.append(driftCorrectionResult.isSuccess() + ",");
            builder.append(driftCorrectionResult.getComputationTime() + ",");
            builder.append(driftCorrectionResult.getIcpIterations() + ",");

            builder.append(driftCorrectionResult.getNumberOfSurfels() + ",");
            builder.append(driftCorrectionResult.getNumberOfCorrespondances() + ",");
            builder.append(driftCorrectionResult.getInitialDistance() + ",");
            builder.append(driftCorrectionResult.getFinalDistance() + "\n");

         }
         sensorPoseFileWriter.write(builder.toString());
         sensorPoseFileWriter.close();
      }
      catch (IOException e1)
      {
         e1.printStackTrace();
      }
   }
}
