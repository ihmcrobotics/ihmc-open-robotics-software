package us.ihmc.atlas.calib;

import static java.lang.Double.parseDouble;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.event.ItemEvent;
import java.awt.event.ItemListener;
import java.awt.image.BufferedImage;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.Properties;

import javax.imageio.ImageIO;
import javax.swing.ImageIcon;
import javax.swing.JCheckBox;
import javax.swing.JLabel;
import javax.swing.JPanel;

import org.ddogleg.optimization.FactoryOptimization;
import org.ddogleg.optimization.UnconstrainedLeastSquares;
import org.ddogleg.optimization.UtilOptimize;

import boofcv.abst.fiducial.calib.CalibrationDetectorChessboard;
import boofcv.abst.fiducial.calib.ConfigChessboard;
import boofcv.alg.geo.PerspectiveOps;
import boofcv.factory.calib.FactoryCalibrationTarget;
import boofcv.io.UtilIO;
import boofcv.struct.calib.IntrinsicParameters;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.simulationconstructionset.IndexChangedListener;

public class AtlasHeadLoopKinematicCalibrator extends AtlasKinematicCalibrator
{   
   public static final String TARGET_TO_CAMERA_KEY = "targetToCamera";
   public static final String CAMERA_IMAGE_KEY = "cameraImage";
   public static final String CHESSBOARD_DETECTIONS_KEY = "chessboardDetections";

   public static final boolean USE_LEFT_ARM = false;

   //YoVariables for Display
   private final YoFramePoint ypLeftEE, ypRightEE;
   private final YoFramePose yposeLeftEE, yposeRightEE, yposeBoard, yposeLeftCamera;
   private final ArrayList<Map<String, Object>> metaData;
   final ReferenceFrame cameraFrame;

   public static final RobotSide activeSide = USE_LEFT_ARM ? RobotSide.LEFT : RobotSide.RIGHT;

   RigidBodyTransform targetToEE = new RigidBodyTransform();

   protected final Map<String, Double> qbias = new HashMap<>();

   private ImageIcon iiDisplay = null;
   private boolean alignCamera = true;

   private IntrinsicParameters intrinsic;
   private CalibrationDetectorChessboard calibGrid = FactoryCalibrationTarget.detectorChessboard(new ConfigChessboard(
         DetectChessboardInKinematicsData.boardWidth, DetectChessboardInKinematicsData.boardHeight, 0.03));

   public AtlasHeadLoopKinematicCalibrator(DRCRobotModel robotModel)
   {
      super(robotModel);
      ypLeftEE = new YoFramePoint("leftEE", ReferenceFrame.getWorldFrame(), registry);
      ypRightEE = new YoFramePoint("rightEE", ReferenceFrame.getWorldFrame(), registry);
      yposeLeftEE = new YoFramePose("leftPoseEE", "", ReferenceFrame.getWorldFrame(), registry);
      yposeRightEE = new YoFramePose("rightPoseEE", "", ReferenceFrame.getWorldFrame(), registry);
      yposeBoard = new YoFramePose("board", "", ReferenceFrame.getWorldFrame(), registry);
      yposeLeftCamera = new YoFramePose("leftCamera", "", ReferenceFrame.getWorldFrame(), registry);

      cameraFrame = fullRobotModel.getCameraFrame("stereo_camera_left");
      metaData = new ArrayList<>();
   }

   @Override
   protected void setupDynamicGraphicObjects()
   {
      //standard SCS Dynamic Graphics Object - automatically updated to the associated yoVariables
      double transparency = 0.5;
      double scale = 0.02;
      YoGraphicPosition dgpLeftEE = new YoGraphicPosition("dgpLeftEE", ypLeftEE, scale, new YoAppearanceRGBColor(Color.BLUE, transparency));
      YoGraphicPosition dgpRightEE = new YoGraphicPosition("dgpRightEE", ypRightEE, scale, new YoAppearanceRGBColor(Color.RED, transparency));

      scs.addYoGraphic(dgpLeftEE);
      scs.addYoGraphic(dgpRightEE);

      YoGraphicCoordinateSystem dgPoseLeftEE = new YoGraphicCoordinateSystem("dgposeLeftEE", yposeLeftEE, 5 * scale);
      YoGraphicCoordinateSystem dgPoseRightEE = new YoGraphicCoordinateSystem("dgposeRightEE", yposeRightEE, 5 * scale);
      YoGraphicCoordinateSystem dgPoseBoard = new YoGraphicCoordinateSystem("dgposeBoard", yposeBoard, 5 * scale);
      YoGraphicCoordinateSystem dgPoseLeftCamera = new YoGraphicCoordinateSystem("dgposeLeftCamera", yposeLeftCamera, 5 * scale);
      scs.addYoGraphic(dgPoseLeftEE);
      scs.addYoGraphic(dgPoseRightEE);
      scs.addYoGraphic(dgPoseBoard);
      scs.addYoGraphic(dgPoseLeftCamera);

      //Homemade Image Display Panel - updated by the IndexChangedListener 
      iiDisplay = new ImageIcon();
      JPanel panel = new JPanel(new BorderLayout());
      final JLabel lblDisplay = new JLabel("", iiDisplay, JLabel.CENTER);
      panel.add(lblDisplay, BorderLayout.CENTER);
      scs.addExtraJpanel(panel, "Image", true);
      scs.getDataBuffer().attachIndexChangedListener(new IndexChangedListener()
      {
         @Override
         public void indexChanged(int newIndex, double newTime)
         {
            int index = (newIndex + q.size() - 1) % q.size();
            CalibUtil.setRobotModelFromData(fullRobotModel, q.get(index), qbias);
            updateBoard(index);
            lblDisplay.repaint();
            if (alignCamera)
               scsAlignCameraToRobotCamera();
         }
      });

      //Set Camera Info
      String intrinsicFile = "../DarpaRoboticsChallenge/data/calibration_images/intrinsic_ros.xml";
      IntrinsicParameters intrinsic = UtilIO.loadXML(intrinsicFile);
      double fovh = Math.atan(intrinsic.getCx() / intrinsic.getFx()) + Math.atan((intrinsic.width - intrinsic.getCx()) / intrinsic.getFx());
      System.out.println("Set fov to " + Math.toDegrees(fovh) + "degs from " + intrinsicFile);
      scs.setFieldOfView(fovh);
      scs.maximizeMainWindow();

      JCheckBox chkAlignCamera = new JCheckBox("AlignCamera", alignCamera);
      chkAlignCamera.addItemListener(new ItemListener()
      {

         @Override
         public void itemStateChanged(ItemEvent e)
         {
            alignCamera = !alignCamera;
            if (alignCamera)
               scsAlignCameraToRobotCamera();
         }
      });
      scs.addCheckBox(chkAlignCamera);
   }

   @Override
   protected void updateDynamicGraphicsObjects(int index)
   {
      /*put yo-variablized objects here */
      FramePoint leftEE = new FramePoint(fullRobotModel.getEndEffectorFrame(RobotSide.LEFT, LimbName.ARM), 0, 0.13, 0);
      FramePoint rightEE = new FramePoint(fullRobotModel.getEndEffectorFrame(RobotSide.RIGHT, LimbName.ARM), 0, -0.13, 0);

      leftEE.changeFrame(CalibUtil.world);
      rightEE.changeFrame(CalibUtil.world);

      ypLeftEE.set(leftEE);
      ypRightEE.set(rightEE);

      yposeLeftEE.set(leftEE, new FrameOrientation(CalibUtil.world));
      yposeRightEE.set(rightEE, new FrameOrientation(CalibUtil.world));

      updateBoard(index);
   }

   private void scsAlignCameraToRobotCamera()
   {
      //Camera Pos(behind the eye 10cm), Fix(Eye farme origin)
      FramePoint cameraPos = new FramePoint(cameraFrame, -0.01, 0, 0);
      FramePoint cameraFix = new FramePoint(cameraFrame);

      cameraPos.changeFrame(CalibUtil.world);
      cameraFix.changeFrame(CalibUtil.world);
      scs.setCameraPosition(cameraPos.getX(), cameraPos.getY(), cameraPos.getZ());
      scs.setCameraFix(cameraFix.getX(), cameraFix.getY(), cameraFix.getZ());
   }

   private void updateBoard(int index)
   {
      //update camera pose display
      RigidBodyTransform imageToCamera = new RigidBodyTransform(new double[]{0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1});
      ReferenceFrame cameraImageFrame = ReferenceFrame.
            constructBodyFrameWithUnchangingTransformToParent("cameraImage", cameraFrame, imageToCamera);
      FramePose poseLeftCamera = new FramePose(cameraImageFrame);
      poseLeftCamera.changeFrame(CalibUtil.world);
      yposeLeftCamera.set(poseLeftCamera);

      //update board
      Map<String, Object> mEntry = metaData.get(index);
      RigidBodyTransform targetToCamera = new RigidBodyTransform((RigidBodyTransform) mEntry.get(TARGET_TO_CAMERA_KEY)); //in camera frame
//      System.out.println("Original Rot\n"+targetToCamera);

      //update
      FramePose poseRightCamera = new FramePose(cameraImageFrame, targetToCamera);
      poseRightCamera.changeFrame(CalibUtil.world);
      yposeBoard.set(poseRightCamera);
//      System.out.println("Index: "+ index);
//      System.out.println(targetToCamera);

      //image update
      BufferedImage work = renderEEinImage(cameraImageFrame, (BufferedImage) mEntry.get(CAMERA_IMAGE_KEY));

      RigidBodyTransform kinematicsTargetToCamera = computeKinematicsTargetToCamera(cameraImageFrame);
      renderCalibrationPoints(kinematicsTargetToCamera, work);

      iiDisplay.setImage(work);
   }

   private BufferedImage renderEEinImage(ReferenceFrame cameraImageFrame, BufferedImage original)
   {
      BufferedImage work = new BufferedImage(original.getWidth(), original.getHeight(), original.getType());
      Graphics2D g2 = work.createGraphics();
      g2.drawImage(original, 0, 0, null);

      double magicNumber = USE_LEFT_ARM ? 0.13 : -0.13;

      FramePoint activeArmEEtoCamera = new FramePoint(fullRobotModel.getEndEffectorFrame(activeSide, LimbName.ARM), 0, magicNumber, 0); // todo look at this later
      activeArmEEtoCamera.changeFrame(cameraImageFrame);
      Point3D activeArmEEinImageFrame = activeArmEEtoCamera.getPoint();

      Point2D_F64 norm = new Point2D_F64(activeArmEEinImageFrame.getX() / activeArmEEinImageFrame.getZ(), activeArmEEinImageFrame.getY() / activeArmEEinImageFrame.getZ());
      Point2D_F64 pixel = new Point2D_F64();

      PerspectiveOps.convertNormToPixel(intrinsic, norm, pixel);

      // visualization
      int r = 10;
      int w = r * 2 + 1;
      int x = (int) (pixel.x + 0.5);
      int y = (int) (pixel.y + 0.5);

      g2.setColor(Color.BLACK);
      g2.fillOval(x - r - 2, y - r - 2, w + 4, w + 4);
      g2.setColor(Color.orange);
      g2.fillOval(x - r, y - r, w, w);

      return work;
   }

   private RigidBodyTransform computeKinematicsTargetToCamera(ReferenceFrame cameraImageFrame)
   {

//      DenseMatrix64F rotY = RotationMatrixGenerator.setToPitchMatrix(Math.PI/2,null);
//      DenseMatrix64F rotZ = RotationMatrixGenerator.rotZ(-Math.PI / 2, null);
//      DenseMatrix64F rot = new DenseMatrix64F(3,3);
//      CommonOps.mult(rotZ, rotY, rot);
//
//      System.out.println(rot);

//      targetToEE.setRotation(rot);
//      targetToEE.setTranslation(new Vector3d(-0.061, 0.13, 0.205));

      ReferenceFrame activeArmEEFrame = fullRobotModel.getEndEffectorFrame(activeSide, LimbName.ARM);
      ReferenceFrame boardFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent("boardFrame", activeArmEEFrame, targetToEE);
      return boardFrame.getTransformToDesiredFrame(cameraImageFrame);

//      FramePoint leftEEtoCamera=new FramePoint(fullRobotModel.getEndEffectorFrame(RobotSide.LEFT, LimbName.ARM)  ,0, 0.13,0);
//      leftEEtoCamera.changeFrame(cameraImageFrame);
   }

   private void renderCalibrationPoints(RigidBodyTransform targetToCamera, BufferedImage output)
   {

      Graphics2D g2 = output.createGraphics();

      // dot size
      int r = 4;
      int w = r * 2 + 1;

      // Points in chessboard frame
      Point2D_F64 norm = new Point2D_F64();
      Point2D_F64 pixel = new Point2D_F64();

      int index = 0;
      for (Point2D_F64 p : calibGrid.getLayout())
      {
         // convert to camera frame
         Point3D p3 = new Point3D(p.x, p.y, 0);
         targetToCamera.transform(p3);

         // convert to pixels
         norm.set(p3.getX() / p3.getZ(), p3.getY() / p3.getZ());
         PerspectiveOps.convertNormToPixel(intrinsic, norm, pixel);

         int x = (int) (pixel.x + 0.5);
         int y = (int) (pixel.y + 0.5);

         if (index++ == 0)
         {
            g2.setColor(Color.CYAN);
         } else
         {
            g2.setColor(Color.BLUE);
         }
         g2.fillOval(x - r, y - r, w, w);
      }


   }

   private ArrayList<OneDoFJoint> getArmJoints()
   {
      ArrayList<OneDoFJoint> armJoints = new ArrayList<OneDoFJoint>();
      for (int i = 0; i < joints.length; i++)
      {
         if (joints[i].getName().matches(".*arm.*"))
         {
            armJoints.add(joints[i]);
            if (DEBUG)
               System.out.println("arm " + i + " " + joints[i].getName());
         }

      }
      return armJoints;
   }

   public void optimizeData()
   {
      KinematicCalibrationHeadLoopResidual function = new KinematicCalibrationHeadLoopResidual(fullRobotModel, USE_LEFT_ARM, intrinsic, calibGrid, metaData, q);

      UnconstrainedLeastSquares optimizer = FactoryOptimization.leastSquaresLM(1e-3, true);

      double input[] = new double[function.getNumOfInputsN()];

      // give it an initial estimate for the translation
//      input[input.length-4]=-0.061;
//      input[input.length-3]=0.13;
//      input[input.length-2]=0.205;

      optimizer.setFunction(function, null);
      optimizer.initialize(input, 1e-12, 1e-12);

      System.out.println("Initial optimziation error = " + optimizer.getFunctionValue());

      UtilOptimize.process(optimizer, 500);

      double found[] = optimizer.getParameters();

      System.out.println("Final optimziation error =   " + optimizer.getFunctionValue());

      java.util.List<String> jointNames = function.getCalJointNames();

      targetToEE = KinematicCalibrationHeadLoopResidual.computeTargetToEE(found, jointNames.size(), USE_LEFT_ARM);

      for (int i = 0; i < jointNames.size(); i++)
      {
         qbias.put(jointNames.get(i), found[i]);
         System.out.println(jointNames.get(i) + " bias: " + Math.toDegrees(found[i]));
      }
      System.out.println("board to wrist rotY:" + found[found.length - 1]);
   }

   public void loadData(String directory) throws IOException
   {
      intrinsic = UtilIO.loadXML("../DarpaRoboticsChallenge/data/calibration_images/intrinsic_ros.xml");

      File[] files = new File(directory).listFiles();
      if (files == null)
      {
         System.out.println("Cannot list files in " + directory);
         return;
      }

      Arrays.sort(files);

//      files = new File[]{files[3],files[20]};
      for (File f : files)
      {
         if (!f.isDirectory())
            continue;
         System.out.println("datafolder:" + f.toString());

         Map<String, Object> mEntry = new HashMap<>();
         Map<String, Double> qEntry = new HashMap<>();
         Map<String, Double> qoutEntry = new HashMap<>();

         if (!loadData(f, mEntry, qEntry, qoutEntry, true))
            continue;

         metaData.add(mEntry);
         q.add(qEntry);

      }
   }

   public static boolean loadData(File f, Map<String, Object> mEntry, Map<String, Double> qEntry, Map<String, Double> qoutEntry,
                                  boolean loadImages) throws IOException
   {
      File fileTarget = new File(f, "target.txt");

      if (!fileTarget.exists() || fileTarget.length() == 0)
         return false;

      // parse targetToCamera transform
      BufferedReader reader = new BufferedReader(new FileReader(fileTarget));

      Se3_F64 targetToCamera = new Se3_F64();

      reader.readLine();         // skip comments

      //read rotation
      String row0[] = reader.readLine().split(" ");
      String row1[] = reader.readLine().split(" ");
      String row2[] = reader.readLine().split(" ");

      for (int col = 0; col < 3; col++)
      {
         targetToCamera.getR().set(0, col, parseDouble(row0[col]));
         targetToCamera.getR().set(1, col, parseDouble(row1[col]));
         targetToCamera.getR().set(2, col, parseDouble(row2[col]));
      }

      //read translation
      reader.readLine();
      String line = reader.readLine();
      if (line == null) { return false; }

      String s[] = line.split(" ");
      targetToCamera.getT().set(parseDouble(s[0]), parseDouble(s[1]), parseDouble(s[2]));

      // read calibration point stuff
      reader.readLine();
      reader.readLine();
      ArrayList<Point2D_F64> detections = new ArrayList<>();
      while (true)
      {
         line = reader.readLine();
         if (line == null)
            break;
         s = line.split(" ");
         Point2D_F64 p = new Point2D_F64();
         p.x = Double.parseDouble(s[0]);
         p.y = Double.parseDouble(s[1]);
         detections.add(p);
      }
      mEntry.put(CHESSBOARD_DETECTIONS_KEY, detections);

      //copy Translation and Rotation
      RigidBodyTransform transform = new RigidBodyTransform();
      Vector3D_F64 T = targetToCamera.T;
      transform.setTranslation(new Vector3D(T.x, T.y, T.z));

      RotationMatrix matrix3d = new RotationMatrix(targetToCamera.getR());
      transform.setRotation(matrix3d);
      mEntry.put(TARGET_TO_CAMERA_KEY, transform);

      //load image
      if (loadImages)
         mEntry.put(CAMERA_IMAGE_KEY, ImageIO.read(new File(f, "/detected.jpg")));

      // load joint angles
      Properties properties = new Properties();
      properties.load(new FileReader(new File(f, "q.m")));

      for (Map.Entry e : properties.entrySet())
      {
         qEntry.put((String) e.getKey(), Double.parseDouble((String) e.getValue()));
      }

      properties = new Properties();
      properties.load(new FileReader(new File(f, "qout.m")));

      for (Map.Entry e : properties.entrySet())
      {
         qoutEntry.put((String) e.getKey(), Double.parseDouble((String) e.getValue()));
      }

      return true;
   }


   public static void main(String[] arg) throws InterruptedException, IOException
   {
	  final AtlasRobotVersion ATLAS_ROBOT_VERSION = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS;
	  
	  DRCRobotModel robotModel = new AtlasRobotModel(ATLAS_ROBOT_VERSION, DRCRobotModel.RobotTarget.REAL_ROBOT, true);
	  
      AtlasHeadLoopKinematicCalibrator calib = new AtlasHeadLoopKinematicCalibrator(robotModel);
      calib.loadData("data/armCalibratoin20131209/calibration_right");
      calib.optimizeData();

      // calJointNames order is the prm order
//      KinematicCalibrationWristLoopResidual residualFunc = calib.getArmLoopResidualObject();
//      double[] prm = new double[residualFunc.getN()];
//      double[] residual0 = residualFunc.calcResiduals(prm);
//      calib.calibrate(residualFunc,prm, 100);
//      double[] residual = residualFunc.calcResiduals(prm);
//
//
//      //display prm in readable format
//      Map<String,Double> qoffset= residualFunc.prmArrayToJointMap(prm);
//      for(String jointName: qoffset.keySet())
//      {
//         System.out.println("jointAngleOffsetPreTransmission.put(AtlasJointId.JOINT_" + jointName.toUpperCase()+", "+qoffset.get(jointName)+");");
//         //System.out.println(jointName + " "+ qoffset.get(jointName));
//      }
//      System.out.println("wristSpacing "+prm[prm.length-1]);

      //push data to visualizer
      boolean start_scs = true;
      if (start_scs)
      {
         //Yovariables for display
//         YoFramePose yoResidual0 = new YoFramePose("residual0", "", ReferenceFrame.getWorldFrame(),calib.registry);
//         YoFramePose yoResidual = new YoFramePose("residual", "",ReferenceFrame.getWorldFrame(),calib.registry);

         calib.createDisplay(calib.q.size());

         for (int i = 0; i < calib.q.size(); i++)
         {
            CalibUtil.setRobotModelFromData(calib.fullRobotModel, (Map) calib.q.get(i));
//            CalibUtil.setRobotModelFromData(calib.fullRobotModel, CalibUtil.addQ(calib.q.get(i),qoffset));
//            yoResidual0.setXYZYawPitchRoll(Arrays.copyOfRange(residual0, i*RESIDUAL_DOF, i*RESIDUAL_DOF+6));
//            yoResidual.setXYZYawPitchRoll(Arrays.copyOfRange(residual, i*RESIDUAL_DOF, i*RESIDUAL_DOF+6));
            calib.displayUpdate(i);
         }
      } //viz
   }
}
