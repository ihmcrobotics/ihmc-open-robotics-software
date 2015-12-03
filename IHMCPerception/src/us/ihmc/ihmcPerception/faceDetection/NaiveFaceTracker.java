package us.ihmc.ihmcPerception.faceDetection;

public class NaiveFaceTracker
{
   private final OpenCVFaceDetector faceDetector;

   public NaiveFaceTracker(double scale)
   {
      faceDetector = new OpenCVFaceDetector(scale);
   }
}
