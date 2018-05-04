package us.ihmc.atlas.calib;

import java.awt.Color;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.HashMap;
import java.util.Map;

import org.apache.tools.zip.ZipFile;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFramePoseUsingYawPitchRoll;

public class AtlasCalibrationDataViewer extends AtlasKinematicCalibrator
{
   //YoVariables for Display
   private final YoFramePoint3D ypLeftEE, ypRightEE;
   private final YoFramePoseUsingYawPitchRoll yposeLeftEE, yposeRightEE;
   Map<String, YoDouble> yoQout = new HashMap<>();
   Map<String, YoDouble> yoQdiff = new HashMap<>();

   public AtlasCalibrationDataViewer(DRCRobotModel robotModel)
   {
      super(robotModel);
      ypLeftEE = new YoFramePoint3D("leftEE", ReferenceFrame.getWorldFrame(), registry);
      ypRightEE = new YoFramePoint3D("rightEE", ReferenceFrame.getWorldFrame(), registry);
      yposeLeftEE = new YoFramePoseUsingYawPitchRoll("leftPoseEE", "", ReferenceFrame.getWorldFrame(), registry);
      yposeRightEE = new YoFramePoseUsingYawPitchRoll("rightPoseEE", "", ReferenceFrame.getWorldFrame(), registry);
   }

   @Override
   protected void setupYoGraphics()
   {
      double transparency = 0.5;
      double scale = 0.02;
      YoGraphicPosition dgpLeftEE = new YoGraphicPosition("dgpLeftEE", ypLeftEE, scale, new YoAppearanceRGBColor(Color.BLUE, transparency));
      YoGraphicPosition dgpRightEE = new YoGraphicPosition("dgpRightEE", ypRightEE, scale, new YoAppearanceRGBColor(Color.RED, transparency));

      scs.addYoGraphic(dgpLeftEE);
      scs.addYoGraphic(dgpRightEE);

      YoGraphicCoordinateSystem dgPoseLeftEE = new YoGraphicCoordinateSystem("dgposeLeftEE", yposeLeftEE, 5 * scale);
      YoGraphicCoordinateSystem dgPoseRightEE = new YoGraphicCoordinateSystem("dgposeRightEE", yposeRightEE, 5 * scale);
      scs.addYoGraphic(dgPoseLeftEE);
      scs.addYoGraphic(dgPoseRightEE);
   }

   @Override
   protected void updateYoGraphics(int index)
   {
      FramePoint3D leftEE = new FramePoint3D(fullRobotModel.getEndEffectorFrame(RobotSide.LEFT, LimbName.ARM), 0, 0.13, 0);
      FramePoint3D rightEE = new FramePoint3D(fullRobotModel.getEndEffectorFrame(RobotSide.RIGHT, LimbName.ARM), 0, -0.13, 0);
      
      leftEE.changeFrame(CalibUtil.world);
      rightEE.changeFrame(CalibUtil.world);

      ypLeftEE.set(leftEE);
      ypRightEE.set(rightEE);

      yposeLeftEE.set(leftEE, new FrameQuaternion(CalibUtil.world));
      yposeRightEE.set(rightEE, new FrameQuaternion(CalibUtil.world));
   }

   public void createQoutYoVariables()
   {
      Map<String, Double> qout0 = (Map) qout.get(0);
      for (String jointName : qout0.keySet())
      {
         yoQout.put(jointName, new YoDouble("qout_" + jointName, registry));
         yoQdiff.put(jointName, new YoDouble("qdiff_" + jointName, registry));
      }

   }

   public void updateQoutYoVariables(int index)
   {
      for (String jointName : qout.get(0).keySet())
      {
         yoQout.get(jointName).set((double) qout.get(index).get(jointName));
         yoQdiff.get(jointName).set((double) qout.get(index).get(jointName) - (double) q.get(index).get(jointName));
      }

   }

   public void loadData(String calib_file)
   {

      BufferedReader reader = null;
      try
      {
         if (calib_file.contains("zip"))
         {
            ZipFile zip = new ZipFile(calib_file);
            reader = new BufferedReader(new InputStreamReader(zip.getInputStream(zip.getEntries().nextElement())));
         }
         else
         {
            reader = new BufferedReader(new FileReader(calib_file));
         }
      }
      catch (IOException e1)
      {
         System.out.println("Cannot load calibration file " + calib_file);
         e1.printStackTrace();
         return;
      }

      String line;
      final int numJoints = 28;
      System.out.println("total joints should be " + numJoints);
      try
      {
         while ((line = reader.readLine()) != null)
         {
            if (line.matches("^entry.*"))
            {
               Map<String, Double> q_ = new HashMap<>();
               Map<String, Double> qout_ = new HashMap<>();

               for (int i = 0; i < numJoints; i++)
               {
                  line = reader.readLine();
                  if (line != null)
                  {
                     String[] items = line.split("\\s");

                     if (items[0].equals("neck_ay"))
                        items[0] = "neck_ry";
                     q_.put(items[0], new Double(items[1]));
                     qout_.put(items[0], new Double(items[2]));
                  }
                  else
                  {
                     System.out.println("One ill-formed data entry");
                     break;
                  }

               }

               if (q_.size() == numJoints)
                  q.add((Map) q_);
               if (qout_.size() == numJoints)
                  qout.add((Map) qout_);
            }
         }
      }
      catch (IOException e1)
      {
         System.err.println("File reading error");
         e1.printStackTrace();
      }
      System.out.println("total entry loaded q/qout " + q.size() + "/" + qout.size());
   }

   /**
    * @param args
    */
   public static void main(String[] args)
   {
	  final AtlasRobotVersion ATLAS_ROBOT_VERSION = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS;
	  DRCRobotModel robotModel = new AtlasRobotModel(ATLAS_ROBOT_VERSION, RobotTarget.SCS, false);
	  
      AtlasWristLoopKinematicCalibrator calib = new AtlasWristLoopKinematicCalibrator(robotModel);
      calib.loadData("data/manip_motions/log4.zip");
      calib.createQoutYoVariables();

      calib.createDisplay(calib.q.size());

      Map<String, Double> q0 = new HashMap<String, Double>();
      
      for(String key: calib.q.get(0).keySet())
         q0.put(key, 0.0);

      
      CalibUtil.setRobotModelFromData(calib.fullRobotModel,q0);
//    
//      for(int i=0;i<calib.q.size();i++)
//      {
//         CalibUtil.setRobotModelFromData(calib.fullRobotModel, (Map)calib.q.get(i));
//         calib.updateQoutYoVariables(i);
//         calib.displayUpdate(i);
//      }

   }

}
