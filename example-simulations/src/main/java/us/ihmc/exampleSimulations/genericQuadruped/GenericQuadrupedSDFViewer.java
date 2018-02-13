package us.ihmc.exampleSimulations.genericQuadruped;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.exampleSimulations.genericQuadruped.model.GenericQuadrupedModelFactory;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationconstructionset.*;

import java.util.ArrayList;
import java.util.HashSet;

public class GenericQuadrupedSDFViewer
{
   private static final boolean SHOW_INERTIA_ELLIPSOIDS = true;

   public static void main(String[] args)
   {
      GenericQuadrupedModelFactory modelFactory = new GenericQuadrupedModelFactory();
      FloatingRootJointRobot sdfRobot = new FloatingRootJointRobot(modelFactory.createSdfRobot());
      sdfRobot.setPositionInWorld(new Vector3D(0.0, 0.0, 0.6));

      if(SHOW_INERTIA_ELLIPSOIDS)
         addInertialEllipsoidsToVisualizer(sdfRobot);

      SimulationConstructionSet scs = new SimulationConstructionSet(sdfRobot);
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

   private static HashSet<Link> getAllLinks(ArrayList<Joint> joints, HashSet<Link> links)
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
