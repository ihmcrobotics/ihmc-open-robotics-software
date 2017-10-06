package us.ihmc.exampleSimulations.stickRobot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;



public class StickRobotVisualizer
{
   private static final boolean SHOW_ELLIPSOIDS = false;
   private static final boolean SHOW_COORDINATES_AT_JOINT_ORIGIN = false;

   private SimulationConstructionSet scs;

   public StickRobotVisualizer()
   {
      // The first argument suggests that the robot is loaded for the SCS platform
      // The default model of the robot from the resources directory would be called.
      // It would create the robot description from the SDF file
      StickRobotModel robotModel = new StickRobotModel(RobotTarget.SCS, false);

      // Setting the pelvis joint
      FloatingRootJointRobot stickRobot = robotModel.createHumanoidFloatingRootJointRobot(false);

      // Setting the pelvis position in world frame. It has an offset of 0.75 in the z-axis
      stickRobot.setPositionInWorld(new Vector3D(0, 0, 0.75));

      if (SHOW_ELLIPSOIDS)
      {
         addIntertialEllipsoidsToVisualizer(stickRobot);
      }

      if (SHOW_COORDINATES_AT_JOINT_ORIGIN)
         addJointAxis(stickRobot);

      //            FullRobotModel fullRobotModel = robotModel.createFullRobotModel();

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      // Enable this to view the inertia ellipsoids
      //      CommonInertiaEllipsoidsVisualizer inertiaVis = new CommonInertiaEllipsoidsVisualizer(fullRobotModel.getElevator(), yoGraphicsListRegistry);
      //      inertiaVis.update();

      scs = new SimulationConstructionSet(stickRobot);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      scs.setGroundVisible(false);
      scs.startOnAThread();

   }

   private void addIntertialEllipsoidsToVisualizer(FloatingRootJointRobot stickRobot)
   {
      ArrayList<Joint> joints = new ArrayList<>();
      joints.add(stickRobot.getRootJoint());

      HashSet<Link> links = getAllLinks(joints, new HashSet<Link>());

      for (Link l : links)
      {
         AppearanceDefinition appearance = YoAppearance.Green();
         appearance.setTransparency(0.6);
         l.addEllipsoidFromMassProperties(appearance);
         l.addCoordinateSystemToCOM(0.5);
         //         l.addBoxFromMassProperties(appearance);
      }
   }

   private HashSet<Link> getAllLinks(ArrayList<Joint> joints, HashSet<Link> links)
   {
      for (Joint j : joints)
      {
         links.add(j.getLink());

         if (!j.getChildrenJoints().isEmpty())
         {
            links.addAll(getAllLinks(j.getChildrenJoints(), links));
         }
      }

      return links;
   }

   public void addJointAxis(FloatingRootJointRobot stickRobot)
   {

      ArrayList<OneDegreeOfFreedomJoint> joints = new ArrayList<>(Arrays.asList(stickRobot.getOneDegreeOfFreedomJoints()));

      for (OneDegreeOfFreedomJoint joint : joints)
      {
         Graphics3DObject linkGraphics = new Graphics3DObject();
         linkGraphics.addCoordinateSystem(0.5);
         linkGraphics.combine(joint.getLink().getLinkGraphics());
         joint.getLink().setLinkGraphics(linkGraphics);
      }
   }

   public static void main(String[] args)
   {
      new StickRobotVisualizer();
   }

}
