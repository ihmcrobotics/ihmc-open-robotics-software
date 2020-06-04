package us.ihmc.exampleSimulations.genericQuadruped;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.exampleSimulations.genericQuadruped.model.GenericQuadrupedModelFactory;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class GenericQuadrupedSDFViewer
{
   private static final boolean SHOW_INERTIA_ELLIPSOIDS = true;

   public static void main(String[] args)
   {
      GenericQuadrupedModelFactory modelFactory = new GenericQuadrupedModelFactory();
      FloatingRootJointRobot sdfRobot = new FloatingRootJointRobot(modelFactory.getRobotDescription());
      sdfRobot.setPositionInWorld(new Vector3D(0.0, 0.0, 0.6));
      sdfRobot.setDynamic(false);

      if(SHOW_INERTIA_ELLIPSOIDS)
         addInertialEllipsoidsToVisualizer(sdfRobot);

      SimulationConstructionSet scs = new SimulationConstructionSet(sdfRobot);
      scs.setGroundVisible(false);
      scs.setDT(1e-4, 1);
      scs.startOnAThread();
   }

   private static void addInertialEllipsoidsToVisualizer(Robot robot)
   {
      ArrayList<Joint> joints = new ArrayList<>();
      joints.add(robot.getRootJoints().get(0));

      HashSet<Link> links = getAllLinks(joints, new HashSet<Link>());

      for (Link l : links)
      {
         AppearanceDefinition appearance = YoAppearance.Green();
         appearance.setTransparency(0.6);
         l.addEllipsoidFromMassProperties(appearance);
      }
   }

   private static HashSet<Link> getAllLinks(List<Joint> joints, HashSet<Link> links)
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

}
