package us.ihmc.exampleSimulations.creatingLinks;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.input.SelectedListener;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.tools.inputDevices.keyboard.ModifierKeyInterface;

public class LinkExampleSimulation
{
   private SimulationConstructionSet sim;

   private static final double CUBE_L = 0.2, CUBE_W = 0.1, CUBE_H = 0.3;
   private static final double SPHERE_R = 0.15;

   private static final double OFFSET = 1.2, COORD_LENGTH = 0.5;
   private static final double WEDGE_X = 0.4, WEDGE_Y = 0.3, WEDGE_Z = 0.2;

   public LinkExampleSimulation()
   {
      /**
       * basic sim set up start
       */

      Robot nullRob = null;
      sim = new SimulationConstructionSet(nullRob);
      sim.setGroundVisible(false);

      // position the camera to view links
      sim.setCameraPosition(6.0, 6.0, 3.0);
      sim.setCameraFix(0.5, 0.5, 0.0);

      /**
       * Add new one link
       */
      Link exampleShapes = exampleShapes();
      sim.addStaticLink(exampleShapes); // add new link to the sim.

      /**
       * Add listener to sim.
       */
      SelectedListener selectedListener = new SelectedListener() // listener
      {

         @Override
         public void selected(Graphics3DNode graphics3dNode,
                              ModifierKeyInterface modifierKeyInterface,
                              Point3DReadOnly location,
                              Point3DReadOnly cameraLocation,
                              QuaternionReadOnly cameraRotation)
         {
            Graphics3DObject sphere = new Graphics3DObject(); // Add sphere where I clicked my mouse.
            sphere.translate(location);
            sphere.addSphere(0.01, YoAppearance.Red());
            sim.addStaticLinkGraphics(sphere);
         }
      };

      sim.attachSelectedListener(selectedListener);

      /**
       * Start the simulation!
       */
      Thread myThread = new Thread(sim);
      myThread.start();
   }

   public static void main(String[] args)
   {
      new LinkExampleSimulation();
   }

   private Link exampleShapes()
   {
      Link links = new Link("example shapes");

      Graphics3DObject linkGraphics = new Graphics3DObject();
      // Cube
      linkGraphics.addCoordinateSystem(COORD_LENGTH);
      linkGraphics.addCube(CUBE_L, CUBE_W, CUBE_H, YoAppearance.Aqua());

      // Sphere
      linkGraphics.translate(CUBE_L / 2, 0.0, 0.0);
      linkGraphics.addCoordinateSystem(COORD_LENGTH);
      linkGraphics.addSphere(SPHERE_R, YoAppearance.Aqua());

      links.setLinkGraphics(linkGraphics);

      return links;
   }
}
