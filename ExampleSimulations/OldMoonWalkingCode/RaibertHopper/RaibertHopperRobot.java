package us.ihmc.moonwalking.models.RaibertHopper;

import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.simulationconstructionset.FloatingPlanarJoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SliderJoint;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;

/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2009</p>
 *
 * <p>Company: </p>
 *
 * @author not attributable
 * @version 1.0
 */
public class RaibertHopperRobot extends Robot
{
    private static final double LEG_MASS = 0.45, BODY_MASS = 8.1, THIGH_MASS = 0.05;
    private static final double BODY_LENGTH = 1.0, LEG_LENGTH = 0.7,
                                HIP_HEIGHT = 0.5, THIGH_LENGTH = 0.4;
    private static final double BODY_RADIUS = 0.02, LEG_RADIUS = 0.02,
                                THIGH_RADIUS = 0.04;
    private static final double BODY_MOI = 10.0*BODY_MASS*BODY_LENGTH*BODY_LENGTH/12.0, LEG_MOI = LEG_MASS*LEG_LENGTH*LEG_LENGTH/12.0, THIGH_MOI = THIGH_MASS*THIGH_LENGTH*THIGH_LENGTH/12.0;

    YoVariable gc_heel_x, gc_heel_z, gc_heel_dx, gc_heel_dz, gc_heel_fs, gc_heel_tdx, gc_heel_tdz, gc_heel_fx, gc_heel_fz;
    YoVariable gc_heel_int_x, gc_heel_int_z;

    Joint plane, knee;
    PinJoint hip;

    public RaibertHopperRobot()
    {
        super("RaibertHopper");
        this.setGravity(0.0,0.0,-9.81);

        plane = new FloatingPlanarJoint("plane", this, FloatingPlanarJoint.XZ);
        Link body = body();
        plane.setLink(body);
        this.addRootJoint(plane);
        ((FloatingPlanarJoint) plane).setCartesianPosition(0.0, 0.65, 0.0, 0.0);
        ((FloatingPlanarJoint) plane).setRotation(0.0);

        hip = new PinJoint("hip", new Vector3d(), this, Joint.Y);
        Link thigh = thigh();
        hip.setLink(thigh);
        plane.addJoint(hip);
        hip.setInitialState(-0.0, 0.0);

        knee = new SliderJoint("knee", new Vector3d(0.0, 0.0, 0.2), this, Joint.Z);//new Vector3d(0.0, 0.0, -1.0));
        Link leg = leg();
        knee.setLink(leg);
        hip.addJoint(knee);
        //((SliderJoint) knee).setDamping(5.0);
        ((SliderJoint) knee).setLimitStops(0.0, 0.25, 150000.0, 300.0);

        GroundContactPoint gc_heel = new GroundContactPoint("gc_heel", new Vector3d(0.0, 0.0, -LEG_LENGTH), this);
        knee.addGroundContactPoint(gc_heel);
    }

    private Link body()
    {
        Link ret = new Link("body");
        LinkGraphics linkGraphics = new LinkGraphics();
        ret.setLinkGraphics(linkGraphics);
        linkGraphics.addCoordinateSystem(0.2);
        linkGraphics.rotate(Math.PI/2.0, Link.Y);
        linkGraphics.translate(0.0, 0.0, -BODY_LENGTH/2.0);

        ret.setMass(BODY_MASS);
        ret.setComOffset(0.0, 0.0, 0.0);
        ret.setMomentOfInertia(0.0, BODY_MOI, BODY_MOI);
        linkGraphics.addCylinder(BODY_LENGTH, BODY_RADIUS, YoAppearance.Red());
        return ret;
    }

    private Link thigh()
   {
       Link ret = new Link("thigh");
       LinkGraphics linkGraphics = new LinkGraphics();
       ret.setLinkGraphics(linkGraphics);
       linkGraphics.addCoordinateSystem(0.15);
       linkGraphics.translate(0.0, 0.0, -THIGH_LENGTH/2.0);

       ret.setMass(THIGH_MASS);
       ret.setComOffset(0.0, 0.0, 0.0);
       ret.setMomentOfInertia(THIGH_MOI, THIGH_MOI, 0.0);
       linkGraphics.addCylinder(THIGH_LENGTH, THIGH_RADIUS);
       return ret;
   }

    private Link leg()
    {
        Link ret = new Link("leg");
        LinkGraphics linkGraphics = new LinkGraphics();
        ret.setLinkGraphics(linkGraphics);
        linkGraphics.addCoordinateSystem(0.15);
        ret.setMass(LEG_MASS);
        ret.setComOffset(0.0, 0.0, -LEG_LENGTH/2.0);
        ret.setMomentOfInertia(LEG_MOI, LEG_MOI, 0.0);
        linkGraphics.addCylinder(-LEG_LENGTH, LEG_RADIUS, YoAppearance.Green());
        return ret;
    }

}
