package us.ihmc.commonWalkingControlModules.calibration.virtualCoMChain;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.centerOfMass.CenterOfMassEstimator;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

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

      System.out.println("Virtual chain parameters have been successfully read from the file " + file.toString());
   }


   public FramePoint getCenterOfMassInFrame(ReferenceFrame desiredFrame)
   {
      FramePoint ret = new FramePoint(baseFrame);
      FrameVector tempFrameVector = new FrameVector(ReferenceFrame.getWorldFrame());

      for (int i = 0; i < virtualChainParameterVectors.size(); i++)
      {
         tempFrameVector.set(virtualChainParameterVectors.get(i));
         tempFrameVector.changeFrame(baseFrame);
         ret.add(tempFrameVector);
      }

      ret.changeFrame(desiredFrame);

      // Need to remove the ankle height because CoP is computed from the ankle torque and not from the ground reaction forces existing
      ret.setZ(ret.getZ() - 0.055);

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
