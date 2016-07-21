package us.ihmc.exampleSimulations.fallingObjects;

import us.ihmc.graphics3DAdapter.GroundProfile3D;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.*;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.BumpyGroundProfile;
import us.ihmc.simulationconstructionset.util.ground.WavyGroundProfile;

import javax.vecmath.Vector3d;

public class FallingObjectsRobot extends Robot {

    //private static YoVariableRegistry registry = new YoVariableRegistry("FallingObjectsRobot");

    public FallingObjectsRobot(){
        super("FallingObjectsRobot");

        this.setGravity(0.0,0.0,-9.81);

        for(int x = 0; x < 5; x++){
            FloatingJoint j = createSphereJoint(x);
            this.addRootJoint(j);
        }


        GroundContactModel groundModel = new LinearGroundContactModel(this, 1422, 150.6, 50.0, 1000.0,
                this.getRobotsYoVariableRegistry());
        GroundProfile3D profile = new BumpyGroundProfile();
        groundModel.setGroundProfile3D(profile);
        this.setGroundContactModel(groundModel);

    }
    public FloatingJoint createSphereJoint(int counter){
        double radius = Math.random()*0.2+0.1;
        double mass = 1.0;

        FloatingJoint j = new FloatingJoint("SphereJoint"+counter, counter+"", new Vector3d(Math.random()*10-5,Math.random()*10-5,Math.random()*2+radius*2), this);

        Link l = new Link("SphereLink");
        l.setMass(mass);
        l.setMomentOfInertia(0.4*mass*radius*radius,0.4*mass*radius*radius,0.4*mass*radius*radius);
        Graphics3DObject g = new Graphics3DObject();
        g.addSphere(radius, YoAppearance.Blue());
        l.setLinkGraphics(g);

        j.setLink(l);

        addSphereContactPoints(j, counter, radius);

        return j;
    }
    public void addSphereContactPoints(FloatingJoint j, int counter, double radius){
        int iterator = 0;
        for(double phi = 0; phi < Math.PI; phi += Math.PI/10.0){
            for(double theta = 0; theta < Math.PI*2.0; theta += Math.PI*0.35){
                GroundContactPoint gc1 = new GroundContactPoint("gc"+counter+iterator, new Vector3d(radius*Math.sin(phi)*Math.cos(theta),
                        radius*Math.sin(phi)*Math.sin(theta), radius*Math.cos(phi)), this);
                j.addGroundContactPoint(gc1);
                iterator++;
            }
        }
    }
}
