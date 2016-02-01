package us.ihmc.commonWalkingControlModules.calibration.virtualChain;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.centerOfMass.CenterOfMassEstimator;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class VirtualChainCenterOfMassEstimator implements CenterOfMassEstimator
{   
   private final ReferenceFrame baseFrame;
   private ArrayList<FrameVector> virtualChainParameterVectors;


   public VirtualChainCenterOfMassEstimator(ReferenceFrame baseFrame, ArrayList<FrameVector> virtualChainParameterVectors)
   {

      this.baseFrame = baseFrame;
      this.virtualChainParameterVectors = virtualChainParameterVectors;
   }

   public VirtualChainCenterOfMassEstimator(ReferenceFrame baseFrame, String fileName, ArrayList<ReferenceFrame> virtualChainFrames)
   {

      this.baseFrame = baseFrame;
      this.virtualChainParameterVectors = new ArrayList<FrameVector>();

      try
      {
         readVirtualChainParameterFile(fileName, virtualChainFrames);
      }
      catch (IOException e)
      {
         System.out.println("Error while reading the virtual chain parameter file");
      }
   }

   private void readVirtualChainParameterFile(String fileName, ArrayList<ReferenceFrame> virtualChainFrames) throws IOException
   {
      File currentDirectory = new File(".");
      String filePath = currentDirectory.getCanonicalPath()
                        + "/../CommonWalkingControlModules/src/us/ihmc/commonWalkingControlModules/calibration/virtualCoMChain/";

      File file = new File(filePath + fileName + ".vcp");
      BufferedReader reader = null;

      try
      {
         reader = new BufferedReader(new FileReader(file));
      }
      catch (FileNotFoundException e)
      {
         e.printStackTrace();

         throw new RuntimeException("Unable to read the virtual chain parameters file " + file.toString());
      }

      for (int i = 0; i < virtualChainFrames.size(); i++)
      {
         String line;
         double[] frameVectorComponents = new double[3];
         FrameVector frameVector = new FrameVector(virtualChainFrames.get(i));

         for (int j = 0; j < 3; j++)
         {
            line = reader.readLine();
            frameVectorComponents[j] = Double.parseDouble(line.trim());
         }

         frameVector.set(frameVectorComponents[0], frameVectorComponents[1], frameVectorComponents[2]);
         virtualChainParameterVectors.add(frameVector);
      }

      if (reader != null)
         reader.close();

      System.out.println("Virtual chain parameters have been successfully read from the file " + file.toString());
   }


   public FramePoint getCenterOfMassInFrame(ReferenceFrame desiredFrame)
   {
      FramePoint ret = new FramePoint(baseFrame);
      FrameVector tempFrameVector = new FrameVector(ReferenceFrame.getWorldFrame());

      for (int i = 0; i < virtualChainParameterVectors.size(); i++)
      {
         tempFrameVector.setIncludingFrame(virtualChainParameterVectors.get(i));
         tempFrameVector.changeFrame(baseFrame);
         ret.add(tempFrameVector);
      }

      ret.changeFrame(desiredFrame);


//     double ankleHeight = 0.055;
//     ret.setZ(ret.getZ() - ankleHeight);

      return ret;
   }

   public String toString()
   {
      String ret = "";

      for (FrameVector frameVector : virtualChainParameterVectors)
      {
         ret = ret + frameVector + "\n";
      }

      return ret;
   }

}
