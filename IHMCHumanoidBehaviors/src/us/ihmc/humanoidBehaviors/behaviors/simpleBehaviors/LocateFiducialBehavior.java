package us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors;

import boofcv.abst.fiducial.FiducialDetector;
import boofcv.factory.fiducial.ConfigFiducialBinary;
import boofcv.factory.fiducial.FactoryFiducial;
import boofcv.factory.filter.binary.ConfigThreshold;
import boofcv.factory.filter.binary.ThresholdType;
import boofcv.io.image.ConvertBufferedImage;
import boofcv.struct.image.GrayF32;
import boofcv.struct.image.ImageType;
import georegression.geometry.ConvertRotation3D_F64;
import georegression.struct.EulerType;
import georegression.struct.se.Se3_F64;
import us.ihmc.communication.producers.JPEGDecompressor;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.sensing.VideoPacket;
import us.ihmc.ihmcPerception.OpenCVTools;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.LongYoVariable;
import us.ihmc.tools.nativelibraries.NativeLibraryLoader;

import java.awt.image.BufferedImage;

public class LocateFiducialBehavior extends AbstractBehavior
{
   static
   {
      NativeLibraryLoader.loadLibrary("org.opencv", OpenCVTools.OPEN_CV_LIBRARY_NAME);
   }

   private final ConcurrentListeningQueue<VideoPacket> videoPacketQueue = new ConcurrentListeningQueue<>();
   private final JPEGDecompressor jpegDecompressor = new JPEGDecompressor();
   private BufferedImage latestUnmodifiedCameraImage;
   private final FiducialDetector<GrayF32> detector;

   private final LongYoVariable fiducialToSearchFor = new LongYoVariable("fiducialToSearchFor", registry);
   private final LongYoVariable fiducialFound = new LongYoVariable("fiducialFound", registry);
   private final DoubleYoVariable fiducialPositionX = new DoubleYoVariable("fiducialPositionX", registry);
   private final DoubleYoVariable fiducialPositionY = new DoubleYoVariable("fiducialPositionY", registry);
   private final DoubleYoVariable fiducialPositionZ = new DoubleYoVariable("fiducialPositionZ", registry);
   private final DoubleYoVariable fiducialEulerRotX = new DoubleYoVariable("fiducialEulerRotX", registry);
   private final DoubleYoVariable fiducialEulerRotY = new DoubleYoVariable("fiducialEulerRotY", registry);
   private final DoubleYoVariable fiducialEulerRotZ = new DoubleYoVariable("fiducialEulerRotZ", registry);

   Se3_F64 targetToSensor = new Se3_F64();
   double[] eulerAngles = new double[3];

   public LocateFiducialBehavior(CommunicationBridgeInterface outgoingCommunicationBridge, double fiducialWidth)
   {
      super(outgoingCommunicationBridge);
      attachNetworkListeningQueue(videoPacketQueue, VideoPacket.class);
      detector = FactoryFiducial.squareBinary(new ConfigFiducialBinary(fiducialWidth), ConfigThreshold.local(ThresholdType.LOCAL_SQUARE, 10), GrayF32.class);

      fiducialFound.set(-1);
   }

   @Override public void doControl()
   {
      VideoPacket videoPacket = videoPacketQueue.getLatestPacket();
      latestUnmodifiedCameraImage = jpegDecompressor.decompressJPEGDataToBufferedImage(videoPacket.getData());
      detectFiducials();
   }

   public void setFiducialToLookFor(long fiducialToSearchFor)
   {
      this.fiducialToSearchFor.set(fiducialToSearchFor);
   }

   private void detectFiducials()
   {
      GrayF32 formattedImage = ConvertBufferedImage.convertFrom(latestUnmodifiedCameraImage, true, ImageType.single(GrayF32.class));
      detector.detect(formattedImage);

      boolean foundDesiredFiducial = false;
      for(int i = 0; i < detector.totalFound(); i++)
      {
         long fiducialId = detector.getId(i);
         if(fiducialId == fiducialToSearchFor.getLongValue())
         {
            storeFiducial(i);
            foundDesiredFiducial = true;
         }
      }

      if(!foundDesiredFiducial && detector.totalFound() > 0)
      {
         storeFiducial(0);
      }
   }
   private void storeFiducial(int fiducialIndexInDetector)
   {
      detector.getFiducialToCamera(fiducialIndexInDetector, targetToSensor);
      fiducialFound.set(detector.getId(fiducialIndexInDetector));

      fiducialPositionX.set(targetToSensor.getX());
      fiducialPositionY.set(targetToSensor.getY());
      fiducialPositionZ.set(targetToSensor.getZ());

      ConvertRotation3D_F64.matrixToEuler(targetToSensor.R, EulerType.XYZ, eulerAngles);

      fiducialEulerRotX.set(eulerAngles[0]);
      fiducialEulerRotY.set(eulerAngles[1]);
      fiducialEulerRotZ.set(eulerAngles[2]);
   }

   @Override public boolean isDone()
   {
      return fiducialFound.getLongValue() > 0;
   }
}
