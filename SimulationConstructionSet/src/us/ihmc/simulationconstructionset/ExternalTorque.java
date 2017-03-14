package us.ihmc.simulationconstructionset;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;


/**
 * Used to apply a torque to the point
 *
 */
public class ExternalTorque implements java.io.Serializable
{
    private static final long serialVersionUID = 3783457908524107434L;

    // torque in inertial frame
    private final YoFrameVector torque;
    public DoubleYoVariable x, y, z;

    protected final String name;
    protected Joint parentJoint;

    public boolean active;

    public ExternalTorque(String name, YoVariableRegistry registry)
    {
        this.name = name;
        torque = new YoFrameVector(name + "_t", "", ReferenceFrame.getWorldFrame(), registry);

        x = torque.getYoX();
        y = torque.getYoY();
        z = torque.getYoZ();
    }

    public void reset() {
        active = false;
        torque.set(0,0,0);
    }

    public boolean isActive() {
        return active;
    }

    public void setActive(boolean active) {
        this.active = active;
    }

    public Joint getParentJoint() {
        return parentJoint;
    }

    public void setParentJoint(Joint parentJoint) {
        this.parentJoint = parentJoint;
    }

    public YoFrameVector getYoTorque() {
        return torque;
    }

    public double getTorqueX() {
        return x.getDoubleValue();
    }

    public double getTorqueY() {
        return y.getDoubleValue();
    }

    public double getTorqueZ() {
        return z.getDoubleValue();
    }

}
