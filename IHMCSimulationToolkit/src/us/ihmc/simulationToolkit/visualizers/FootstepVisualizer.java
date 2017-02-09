package us.ihmc.simulationToolkit.visualizers;

import java.awt.Color;
import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Vector3d;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicText;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.BagOfSingleFootstepVisualizers;
import us.ihmc.humanoidRobotics.footstep.FootSpoof;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.SimplePathParameters;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.TurningThenStraightFootstepGenerator;
import us.ihmc.humanoidRobotics.footstep.footstepSnapper.AtlasFootstepSnappingParameters;
import us.ihmc.humanoidRobotics.footstep.footstepSnapper.ConvexHullFootstepSnapper;
import us.ihmc.humanoidRobotics.footstep.footstepSnapper.FootstepSnapper;
import us.ihmc.humanoidRobotics.footstep.footstepSnapper.FootstepSnappingParameters;
import us.ihmc.humanoidRobotics.footstep.footstepSnapper.SimpleFootstepValueFunction;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.simulationconstructionset.ExitActionListener;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.tools.thread.ThreadTools;

public class FootstepVisualizer
{
   private static boolean AUTO_CLOSE_SCS_FOR_TESTING = false;
   final static boolean DEBUG = false;

   private final YoVariableRegistry registry = new YoVariableRegistry("FootstepVisualizer");
   private final SimulationConstructionSet scs;
   private final Robot nullRobot;
   
   private final YoGraphicsListRegistry yoGraphicsListRegistry;
   private final BagOfSingleFootstepVisualizers bagOfSingleFootstepVisualizers;


   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private YoFramePoint textPoint;
   private boolean firstFocus = true;
   private double focusXmax;
   private double focusYmax;
   private double focusXmin;
   private double focusYmin;
   private double focusX;
   private double focusY;

   public FootstepVisualizer(GroundProfile3D groundProfile, Graphics3DObject linkGraphics, int maxNumberOfFootstepsPerSide, int maxContactPointsPerFoot, String description)
   {
      nullRobot = new Robot("FootstepVisualizerRobot");
      
      if (groundProfile != null)
      {
         GroundContactModel gcModel = new LinearGroundContactModel(nullRobot, nullRobot.getRobotsYoVariableRegistry());
         gcModel.setGroundProfile3D(groundProfile);
         nullRobot.setGroundContactModel(gcModel);
      }
      
      scs = new SimulationConstructionSet(nullRobot);
      
      if (linkGraphics != null)
      {
         scs.setGroundVisible(false);
         scs.addStaticLinkGraphics(linkGraphics);
      }
      
      printifdebug("Attaching exit listener");

      scs.setDT(1, 1);
      yoGraphicsListRegistry = new YoGraphicsListRegistry();

      bagOfSingleFootstepVisualizers = new BagOfSingleFootstepVisualizers(maxNumberOfFootstepsPerSide, maxContactPointsPerFoot, registry, yoGraphicsListRegistry);

      addText(scs, yoGraphicsListRegistry, description);
   }
   
   public void startVisualizer()
   {
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      scs.addYoVariableRegistry(registry);
      scs.setupGraphGroup("step times", new String[][]
      {
         {"t"}
      });
      scs.startOnAThread();
      scs.tickAndUpdate();
   }
   
   public SimulationConstructionSet getSimulationConstructionSet()
   {
      return scs;
   }
   
   public void visualizeInitialFootsteps(SideDependentList<? extends ContactablePlaneBody> bipedFeet, SideDependentList<Footstep> initialFeet)
   {
      bagOfSingleFootstepVisualizers.visualizeFootstep(initialFeet.get(RobotSide.LEFT), bipedFeet.get(RobotSide.LEFT));
      bagOfSingleFootstepVisualizers.visualizeFootstep(initialFeet.get(RobotSide.RIGHT), bipedFeet.get(RobotSide.RIGHT));
   }
   
   public void visualizeFootsteps(SideDependentList<? extends ContactablePlaneBody> bipedFeet, List<Footstep> footsteps)
   {
      for (Footstep footstep : footsteps)
      {
         RobotSide robotSide = footstep.getRobotSide();
         visualizeFootstep(bipedFeet.get(robotSide), footstep);
      }
   }
   
   public void visualizeFootstep(ContactablePlaneBody bipedFoot, Footstep footstep)
   {
      bagOfSingleFootstepVisualizers.visualizeFootstep(footstep, bipedFoot);

      nullRobot.setTime(nullRobot.getTime() + scs.getDT());

      FramePoint solePositon = new FramePoint(footstep.getSoleReferenceFrame());
      solePositon.changeFrame(worldFrame);

      updateFocus(solePositon);

      scs.setCameraFix(focusX, focusY, 0.0);
      scs.setCameraPosition(focusX, focusY - 1.5, 6.0);
      scs.tickAndUpdate();
   }

   public void waitForSCSToClose()
   {
      scs.disableSystemExit();
      ExitChecker exitCheckerRunnable = new ExitChecker();
      scs.attachExitActionListener(exitCheckerRunnable);    // Make sure exit listener attached before starting scs in FootstepVisualizer constructor
      waitForSCSToClose(scs, exitCheckerRunnable);
   }

   private static void deleteFirstDataPointAndCropData(SimulationConstructionSet scs)
   {
      scs.gotoInPointNow();
      scs.tick(1);
      scs.setInPoint();
      scs.cropBuffer();
      scs.gotoOutPointNow();
   }
   

   private void updateFocus(FramePoint point)
   {
      double x = point.getX();
      double y = point.getY();
      if (firstFocus)
      {
         focusXmin = x;
         focusYmin = y;
         focusXmax = x;
         focusYmax = y;
         firstFocus = false;
      }
      else
      {
         focusXmin = Math.min(focusXmin, x);
         focusYmin = Math.min(focusYmin, y);
         focusXmax = Math.max(focusXmax, x);
         focusYmax = Math.max(focusYmax, y);
      }

      focusX = (focusXmax + focusXmin) / 2.0;
      focusY = (focusYmax + focusYmin) / 2.0;

      if (textPoint != null)
      {
         textPoint.setX(focusX);
         textPoint.setY(focusYmax + 0.2);
      }
   }

   private void addText(SimulationConstructionSet scs, YoGraphicsListRegistry yoGraphicsListRegistry, String string)
   {
      if (string == null)
         return;

      YoGraphicsList yoGraphicsList = new YoGraphicsList("TextDescription");

      textPoint = new YoFramePoint("Desc", worldFrame, registry);
      textPoint.set(focusX, focusY, -.003);
      YoFrameOrientation orientation = new YoFrameOrientation("Desc", worldFrame, registry);
      double scale = 0.5;
      YoGraphicText desc = new YoGraphicText("FootstepDescription", string, textPoint, orientation, scale, Color.WHITE, Color.BLACK);
      yoGraphicsList.add(desc);
      if (yoGraphicsListRegistry != null)
         yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
   }

   private synchronized static void waitForSCSToClose(SimulationConstructionSet scs, ExitChecker exitCheckerRunnable)
   {
      boolean firstRun = true;
      while (exitCheckerRunnable.stillRunning())
      {
         if (firstRun)
         {
            printifdebug("Entered wait loop");

            firstRun = false;

            if (AUTO_CLOSE_SCS_FOR_TESTING)
            {
               try
               {
                  Thread.sleep(1000);
               }
               catch (InterruptedException e)
               {
                  e.printStackTrace();
               }

               scs.closeAndDispose();
            }
         }

         try
         {
            FootstepVisualizer.class.wait();
         }
         catch (InterruptedException e)
         {
            e.printStackTrace();
         }

         ThreadTools.sleep(1000L);
      }

      printifdebug("Exited wait loop");
   }

   private static void printifdebug(String debugMessage)
   {
      if (DEBUG)
         System.out.println(debugMessage);
   }


   private class ExitChecker implements ExitActionListener
   {
      boolean stillRunning = true;

      public synchronized void exitActionPerformed()
      {
         printifdebug("Exit action performed");
         stillRunning = false;

         synchronized (FootstepVisualizer.class)
         {
            FootstepVisualizer.class.notifyAll();
         }
      }

      public synchronized boolean stillRunning()
      {
         return stillRunning;
      }
   }


   public static void main(String[] args)
   {
//    boolean CHECK_MEMORY = false;
//
//    if (CHECK_MEMORY)
//       MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB("FootstepVisualizer" + " before test.");

      double xToAnkle = -0.15;
      double yToAnkle = 0.02;
      double zToAnkle = 0.21;
      double footForward = 0.25;
      double footBack = 0.05;
      double footHalfWidth = 0.05;
      double coefficientOfFriction = 0.0;
      ContactablePlaneBody leftContactableFoot = new FootSpoof("leftFoot", xToAnkle, yToAnkle, zToAnkle, footForward, footBack, footHalfWidth,
                                                    coefficientOfFriction);
      ContactablePlaneBody rightContactableFoot = new FootSpoof("rightFoot", xToAnkle, yToAnkle, zToAnkle, footForward, footBack, footHalfWidth,
                                                     coefficientOfFriction);

      SideDependentList<ContactablePlaneBody> contactableFeet = new SideDependentList<ContactablePlaneBody>();
      {
         contactableFeet.set(RobotSide.LEFT, leftContactableFoot);
         contactableFeet.set(RobotSide.RIGHT, rightContactableFoot);
      }

      SideDependentList<RigidBody> feetRigidBodies = new SideDependentList<RigidBody>();
      SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<ReferenceFrame>();
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            feetRigidBodies.put(robotSide, contactableFeet.get(robotSide).getRigidBody());
            soleFrames.put(robotSide, contactableFeet.get(robotSide).getSoleFrame());
         }
      }

      new FramePose2d(worldFrame, new Point2d(0.0, 0.0), 0);
      FramePoint2d endPoint = new FramePoint2d(worldFrame, 3.0, 0.0);

      SimplePathParameters pathType = new SimplePathParameters(0.5, 0.2, 0.0, Math.PI / 6, Math.PI / 12, 0.3);
      
      FootstepSnappingParameters snappingParameters = new AtlasFootstepSnappingParameters();
      FootstepSnapper footstepSnapper = new ConvexHullFootstepSnapper(new SimpleFootstepValueFunction(snappingParameters), snappingParameters);
      
//      FootstepSnapper footstepSnapper = new SimpleFootstepSnapper();


      FramePose2d leftFootPose2d = new FramePose2d(worldFrame, new Point2d(0.0, 0.1), 0);
      Footstep leftStart = footstepSnapper.generateFootstepWithoutHeightMap(leftFootPose2d, feetRigidBodies.get(RobotSide.LEFT),
                              soleFrames.get(RobotSide.LEFT), RobotSide.LEFT, 0.0, new Vector3d(0.0, 0.0, 1.0));
      FramePose2d rightFootPose2d = new FramePose2d(worldFrame, new Point2d(0.0, -0.1), 0);
      Footstep rightStart = footstepSnapper.generateFootstepWithoutHeightMap(rightFootPose2d, feetRigidBodies.get(RobotSide.RIGHT),
                               soleFrames.get(RobotSide.RIGHT), RobotSide.RIGHT, 0.0, new Vector3d(0.0, 0.0, 1.0));

      TurningThenStraightFootstepGenerator generator = new TurningThenStraightFootstepGenerator(feetRigidBodies, soleFrames, endPoint, pathType,
                                                          RobotSide.RIGHT);

      String description = "Footstep Visualizer Main";
      List<Footstep> footsteps = generator.generateDesiredFootstepList();

      SideDependentList<Footstep> initialFootsteps = new SideDependentList<Footstep>(leftStart, rightStart);

      GroundProfile3D groundProfile = null;
      Graphics3DObject linkGraphics = null;

      int maxNumberOfFootstepsPerSide = 100;
      int maxContactPointsPerFoot = 10;
      FootstepVisualizer footstepVisualizer = new FootstepVisualizer(groundProfile, linkGraphics, maxNumberOfFootstepsPerSide, maxContactPointsPerFoot, description);
      footstepVisualizer.startVisualizer();
      
      footstepVisualizer.visualizeInitialFootsteps(contactableFeet, initialFootsteps);
      footstepVisualizer.visualizeFootsteps(contactableFeet, footsteps);


//    if (CHECK_MEMORY)
//    {
//       for (int i = 1; i < 10; i++)
//       {
//          showFootsteps("Mem test " + i, initialContactableFeet, generator.generateDesiredFootstepList(), new SideDependentList<Footstep>(leftStart, rightStart));
//          if (i % 3 == 0)
//             MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB("FootstepVisualizer" + " during test " + i);
//       }
//
//       System.gc();
//
//       try
//       {
//          Thread.sleep(2000);
//       }
//       catch (InterruptedException e)
//       {
//       }
//
//       int mem = MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB("FootstepVisualizer" + " after test.");
//       if (mem > 50)
//          System.err.println("Memory greater than 50MB");
//    }
   }





   public Robot getRobot()
   {
      return nullRobot;
   }

   public YoGraphicsListRegistry getGraphicsListRegistry()
   {
      return yoGraphicsListRegistry;
   }

}
