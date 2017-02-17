package us.ihmc.exampleSimulations.multilevelGround;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.util.LinearStickSlipGroundContactModel;

public class MultilevelGroundExampleSimulation
{
   public MultilevelGroundExampleSimulation()
   {
      Robot robot = new Robot("block");

      FloatingJoint rootJoint = new FloatingJoint("root", new Vector3D(), robot);
      Link link = new Link("block");
      link.setMass(1.0);
      link.setMomentOfInertia(0.1, 0.1, 0.1);

      Graphics3DObject linkGraphics = new Graphics3DObject();

      double length = 0.3;
      double width = 0.1;
      double height = 0.05;

      linkGraphics.translate(0.0, 0.0, -height / 2.0);
      linkGraphics.addCube(length, width, height);
      link.setLinkGraphics(linkGraphics);

      rootJoint.setLink(link);

      for (int i = 0; i < 8; i++)
      {
         double xSign = 1.0;
         double ySign = 1.0;
         double zSign = 1.0;

         if (i % 2 == 0)
         {
            xSign = -1.0;
         }

         if ((i / 2) % 2 == 0)
         {
            ySign = -1.0;
         }

         if ((i / 4) % 2 == 0)
         {
            zSign = -1.0;
         }

         GroundContactPoint groundContactPoint = new GroundContactPoint("gc_" + i,
                                                    new Vector3D(xSign * length / 2.0, ySign * width / 2.0, zSign * height / 2.0), robot);
         rootJoint.addGroundContactPoint(groundContactPoint);
      }


      robot.addRootJoint(rootJoint);

      rootJoint.setPosition(0.0, 0.0, 3.0);
      rootJoint.setYawPitchRoll(0.0, 0.0, 0.0);

      LinearStickSlipGroundContactModel groundContactModel = new LinearStickSlipGroundContactModel(robot, robot.getRobotsYoVariableRegistry());
      groundContactModel.setAlphaStickSlip(0.3, 0.2);
      groundContactModel.enableSlipping();
      groundContactModel.enableSurfaceNormal();
      robot.setGroundContactModel(groundContactModel);

      MarbleRunGround marbleRunGround = new MarbleRunGround("marbleRun");
      groundContactModel.setGroundProfile3D(marbleRunGround);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(16384);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      scs.setGroundVisible(false);

      scs.addStaticLinkGraphics(marbleRunGround.getLinkGraphics());
      scs.setSimulateDuration(4.0);

      scs.startOnAThread();
      
      scs.simulate();
   }

   public static void main(String[] args)
   {
      new MultilevelGroundExampleSimulation();
   }
}
