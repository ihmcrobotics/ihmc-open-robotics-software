package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Random;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.RectangularContactableBody;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.ConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTestTools;
import us.ihmc.utilities.screwTheory.SixDoFJoint;

import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsList;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicYoFramePolygon;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameConvexPolygon2d;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePose;
import com.yobotics.simulationconstructionset.util.trajectory.OverheadPath;
import com.yobotics.simulationconstructionset.util.trajectory.StraightLineOverheadPath;

public class FootstepGeneratorVisualizer
{
   private final YoVariableRegistry registry = new YoVariableRegistry("FootstepGeneratorVisualizer");
   
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final boolean DEBUG = true;
   
   private final ArrayList<YoFramePose> contactPoses = new ArrayList<YoFramePose>();
   private final ArrayList<YoFrameConvexPolygon2d> contactPolygonsWorld = new ArrayList<YoFrameConvexPolygon2d>();

   private final HashMap<String, YoFramePose> contactPosesHashMap = new HashMap<String, YoFramePose>();
   private final HashMap<String, YoFrameConvexPolygon2d> contactPolygonsHashMap = new HashMap<String, YoFrameConvexPolygon2d>();
   
   
   public FootstepGeneratorVisualizer(int maxNumberOfContacts, int maxPointsPerContact, YoVariableRegistry parentRegistry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      DynamicGraphicObjectsList dynamicGraphicObjectsList = new DynamicGraphicObjectsList("FootstepGeneratorVisualizer");

      AppearanceDefinition[] appearances = new AppearanceDefinition[]{YoAppearance.Red(), YoAppearance.Green(), YoAppearance.Blue(), YoAppearance.Purple()};
      
      for (int i=0; i<maxNumberOfContacts; i++)
      {
         YoFramePose contactPose = new YoFramePose("contactPose" + i, "", worldFrame, registry);
         contactPoses.add(contactPose);
      
         YoFrameConvexPolygon2d contactPolygon = new YoFrameConvexPolygon2d("contactPolygon" + i, "", worldFrame, maxPointsPerContact, registry);
         contactPolygonsWorld.add(contactPolygon);
      
         DynamicGraphicYoFramePolygon dynamicGraphicPolygon = new DynamicGraphicYoFramePolygon("contactPolygon"+i, contactPolygon, contactPose, 1.0, appearances[i % appearances.length]);
         dynamicGraphicObjectsList.add(dynamicGraphicPolygon);
      }    
      
      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjectsList(dynamicGraphicObjectsList);
      parentRegistry.addChild(registry);
   }
   
   
   public void addFootstepsAndTickAndUpdate(SimulationConstructionSet scs, List<Footstep> footsteps)
   {
      for (Footstep footstep : footsteps)
      {
         addFootstep(footstep);
         scs.tickAndUpdate();
      }
      
   }
   
   public void addFootstep(Footstep footstep)
   {
      printIfDebug("Adding footstep " + footstep);
      
      RigidBody rigidBody = footstep.getBody();
      FramePose footstepPose = footstep.getPoseCopy();
      
      String name = rigidBody.getName();
      YoFramePose contactPose = contactPosesHashMap.get(name);
      YoFrameConvexPolygon2d contactPolygon = contactPolygonsHashMap.get(name);
      
      if (contactPose == null)
      {
         printIfDebug("Associating new rigidBody " + name);
         contactPose = contactPoses.remove(0);
         contactPolygon = contactPolygonsWorld.remove(0);

         contactPosesHashMap.put(name, contactPose);
         contactPolygonsHashMap.put(name, contactPolygon);

         // Keep the contact points in sole frame:
         List<FramePoint> expectedContactPoints = footstep.getExpectedContactPoints();
         ArrayList<Point2d> contactPointsInSoleFrame = new ArrayList<Point2d>();

         for (FramePoint contactPoint : expectedContactPoints)
         {
            contactPointsInSoleFrame.add(new Point2d(contactPoint.getX(), contactPoint.getY()));
         }

         ConvexPolygon2d convexPolygon2d = new ConvexPolygon2d(contactPointsInSoleFrame);
         contactPolygon.setConvexPolygon2d(convexPolygon2d);
      }
      
      else
      {
         printIfDebug("Found association for rigidBody " + name);
      }
      
      FramePose footstepPoseInWorld = footstepPose.changeFrameCopy(worldFrame);
      FramePoint position = footstepPoseInWorld.getPosition();
      position.setZ(position.getZ() + 0.0005);
      footstepPoseInWorld.setPosition(position);
      
      contactPose.set(footstepPoseInWorld);
   }
   
   private void printIfDebug(String string)
   {
      if (DEBUG) System.out.println(string);
   }


   public static void main(String[] args)
   {
      Robot nullRobot = new Robot("FootstepVisualizerRobot");
      SimulationConstructionSet scs = new SimulationConstructionSet(nullRobot);
      scs.setDT(0.25, 1);
      
      int maxNumberOfContacts = 2;
      int maxPointsPerContact = 4;
      YoVariableRegistry rootRegistry = scs.getRootRegistry();
      DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry();
      FootstepGeneratorVisualizer footstepGeneratorVisualizer = new FootstepGeneratorVisualizer(maxNumberOfContacts, maxPointsPerContact, rootRegistry, dynamicGraphicObjectsListRegistry);
   
      double heading = Math.PI;
      double stepAngle = Math.PI/16.0;
      double stepWidth = 0.2;
     
      SideDependentList<ContactablePlaneBody> bipedFeet = new SideDependentList<ContactablePlaneBody>();
      SideDependentList<PoseReferenceFrame> soleFrames = new  SideDependentList<PoseReferenceFrame>();
      SideDependentList<SixDoFJoint> sixDoFJoints = new  SideDependentList<SixDoFJoint>();
      
      RigidBody rootBody = new RigidBody("Root", ReferenceFrame.getWorldFrame());
      Random random = new Random(1776L);
      
      for (RobotSide robotSide : RobotSide.values())
      {
         PoseReferenceFrame soleFrame = new PoseReferenceFrame(robotSide.getCamelCaseNameForStartOfExpression() + "Frame", worldFrame);
         soleFrames.set(robotSide, soleFrame);
         
         SixDoFJoint sixDoFJoint = new SixDoFJoint("SixDofJoint" + robotSide, rootBody, rootBody.getBodyFixedFrame());
         sixDoFJoints.set(robotSide, sixDoFJoint);
         
         RigidBody rigidBody = ScrewTestTools.addRandomRigidBody(robotSide.getCamelCaseNameForStartOfExpression() + "Foot", random, sixDoFJoint);
         double forward = 0.2;
         double back = -0.2;
         double left = 0.1;
         double right = -0.1;
         RectangularContactableBody contactableBody = new RectangularContactableBody(rigidBody, soleFrame, forward, back, left, right);
         
         bipedFeet.set(robotSide, contactableBody);
      }
      
      FramePose startStanceFootPose = new FramePose(worldFrame, new Point3d(0.0, 0.2, 0.0), new Quat4d());
      FramePose startSwingFootPose = new FramePose(worldFrame, new Point3d(0.0, -0.2, 0.0), new Quat4d());
      ContactablePlaneBody leftFoot = bipedFeet.get(RobotSide.LEFT);
      ContactablePlaneBody rightFoot = bipedFeet.get(RobotSide.RIGHT);
   
      soleFrames.get(RobotSide.LEFT).updatePose(startStanceFootPose);
      soleFrames.get(RobotSide.RIGHT).updatePose(startSwingFootPose);
      
      soleFrames.get(RobotSide.LEFT).update();
      soleFrames.get(RobotSide.RIGHT).update();
      
      Point3d stanceAnklePosition = startStanceFootPose.getPosition().getPointCopy();
      Point3d swingAnklePosition = startSwingFootPose.getPosition().getPointCopy();
      
      // No idea why this isn't changing anything
//    stanceAnklePosition.setZ(stanceAnklePosition.getZ() + 0.1);
//    swingAnklePosition.setZ(swingAnklePosition.getZ() + 0.1);
      
      sixDoFJoints.get(RobotSide.LEFT).setPosition(stanceAnklePosition);
      sixDoFJoints.get(RobotSide.RIGHT).setPosition(swingAnklePosition);
      
      Footstep startStanceFootstep = new Footstep(leftFoot.getRigidBody(), startStanceFootPose, leftFoot.getContactPoints());
      Footstep startSwingFootstep = new Footstep(rightFoot.getRigidBody(), startSwingFootPose, rightFoot.getContactPoints());
     
      dynamicGraphicObjectsListRegistry.addDynamicGraphicsObjectListsToSimulationConstructionSet(scs);


//      ArrayList<Footstep> footsteps = FootstepUtils.makeTurningFootsteps(heading, stepAngle, stepWidth, startStanceFootstep, startSwingFootstep, bipedFeet);
      Linear2DPathBasedDesiredFootstepListGenerator generator = new Linear2DPathBasedDesiredFootstepListGenerator(worldFrame, bipedFeet);
            
      generator.setSwingStart(startSwingFootstep);
      
      FramePoint2d startPoint = new FramePoint2d(worldFrame, new Point2d(0.0, 0.0));
      FramePoint2d endPoint = new FramePoint2d(worldFrame, new Point2d(-5.0, 0.0));
      
      OverheadPath footstepPath = new StraightLineOverheadPath(startPoint, endPoint);
      generator.setFootstepPath(footstepPath);
      
      List<Footstep> footsteps = generator.generateDesiredFootstepList(startStanceFootstep);

//      ArrayList<Footstep> footsteps = FootstepUtils.(heading, stepAngle, stepWidth, startStanceFootstep, startSwingFootstep, bipedFeet);
     
      footstepGeneratorVisualizer.addFootstepsAndTickAndUpdate(scs, footsteps);
         
      scs.startOnAThread();
      
      deleteFirstDataPointAndCropData(scs);
   }

   private static void deleteFirstDataPointAndCropData(SimulationConstructionSet scs)
   {
      scs.gotoInPointNow();
      scs.tick(1);
      scs.setInPoint();
      scs.cropBuffer();
   }


  
}
