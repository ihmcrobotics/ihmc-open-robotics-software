package us.ihmc.exampleSimulations.fallingObjects;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.BumpyGroundProfile;

public class FallingObjectsRobot extends Robot {

    public FallingObjectsRobot(){

        super("FallingObjectsRobot");

        this.setGravity(0.0,0.0,-9.81);

        for(int x = 0; x < 6; x++){
            FloatingJoint j;
            if(x%2==0)
                j = createRodJoint(x);
            else
                j = createSphereJoint(x);
            this.addRootJoint(j);
        }


        GroundContactModel groundModel = new LinearGroundContactModel(this, 1422, 150.6, 50.0, 1000.0,
                this.getRobotsYoVariableRegistry());
        GroundProfile3D profile = new BumpyGroundProfile();
        groundModel.setGroundProfile3D(profile);
        this.setGroundContactModel(groundModel);

    }
    public FloatingJoint createRodJoint(int counter){

        double radius = 0.03;
        double length = Math.random()*1+0.5;
        double mass = 12.0;

        FloatingJoint j = new FloatingJoint("RodJoint"+counter, counter+"", new Vector3D(Math.random()*10-5,Math.random()*10-5,Math.random()*2+length), this);

        Link l = new Link("RodLink");
        l.setMass(mass);
        l.setComOffset(0.0,0.0,length/2);
        l.setMomentOfInertia((1.0/3.0)*mass*length*length,(1.0/3.0)*mass*length*length,0.001);
        Graphics3DObject g = new Graphics3DObject();
        g.addCylinder(length, radius);
        l.setLinkGraphics(g);

        j.setLink(l);

        addRodContactLinks(j, counter, radius, length);

        return j;

    }
    public void addRodContactLinks(FloatingJoint j, int counter, double radius, double length){

        int iterator = 0;
        int zCount = 0;
        for(double z = 0; z <= length; z+= length/3){
            for(double theta = 0; theta < Math.PI*2; theta += Math.PI*0.35){
                GroundContactPoint gc1 = new GroundContactPoint("gc"+counter+iterator+zCount, new Vector3D(radius*Math.sin(theta),
                        radius*Math.cos(theta), z), this);
                j.addGroundContactPoint(gc1);
                iterator++;
            }
            zCount++;
        }

    }
    public FloatingJoint createSphereJoint(int counter){

        double radius = Math.random()*0.15+0.15;
        double mass = 1.0;

        FloatingJoint j = new FloatingJoint("SphereJoint"+counter, counter+"", new Vector3D(Math.random()*10-5,Math.random()*10-5,Math.random()*2+radius*2), this);

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
                GroundContactPoint gc1 = new GroundContactPoint("gc"+counter+iterator, new Vector3D(radius*Math.sin(phi)*Math.cos(theta),
                        radius*Math.sin(phi)*Math.sin(theta), radius*Math.cos(phi)), this);
                j.addGroundContactPoint(gc1);
                iterator++;
            }
        }

    }
}
