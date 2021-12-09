package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.HashMap;

public class RemoteControllerStateNetworkingThread extends Thread {
    private ServerSocket serverSocket = null;
    private PrintWriter out = null;
    private BufferedReader in = null;

    private HashMap<String, Double> desiredAngles;
    private HashMap<String, Double> currentAngles;
    private HashMap<String, Double> currentJointSpeeds;
    private ReferenceFrame centerOfMassFrame;
    private FramePoint2D coP;
    private ForceSensorDataReadOnly wristLeft;
    private ForceSensorDataReadOnly wristRight;
    private double time;
    private ReferenceFrame pelvisFrame;
    private MovingReferenceFrame baseFrame;

    public RemoteControllerStateNetworkingThread(int l) {
        desiredAngles = new HashMap<>();
        currentAngles = new HashMap<>();
        currentJointSpeeds = new HashMap<>();
    }

    public void run(){
        try {
            serverSocket = new ServerSocket(15923);
        } catch (IOException e) {
            e.printStackTrace();
        }
        System.out.println("Remote controller listening at " + serverSocket.getInetAddress() + ":" + serverSocket.getLocalPort() + " with " + desiredAngles.keySet().size() + " parameters.");
        while (true) {
            Socket clientSocket = null;
            try {
                clientSocket = serverSocket.accept();
                System.out.println("Remote controller connected");
                out = new PrintWriter(clientSocket.getOutputStream(), true);
                in = new BufferedReader(new InputStreamReader(clientSocket.getInputStream()));
                printSensorReadings();
                double step = this.time;
                while (true) {
                    String desired = in.readLine();
                    if (desired != null) {
                        String[] angles = desired.split(",");
                        for (int i = 0; i < angles.length; i++) {
                            String[] parts = angles[i].split("=");
                            desiredAngles.put(parts[0], Double.parseDouble(parts[1]));
                        }
                    } else {
                        System.out.println("Command was null");
                        clientSocket.close();
                        break;
                    }
                    step++;
                    long sleepTime = (long)(step * 1000. / 30. - this.time * 1000);
                    if (sleepTime > 0) {
                        Thread.sleep(sleepTime);
                    }
                    printSensorReadings();
                }
            } catch (IOException | InterruptedException e) {
                e.printStackTrace();
                if (clientSocket != null) {
                    try {
                        System.out.println("Closing connection");
                        clientSocket.close();
                    } catch (IOException ex) {
                        ex.printStackTrace();
                    }
                }
            }
        }
    }

    private void printSensorReadings() {
        boolean first = true;
        for (String jointName: currentAngles.keySet()) {
            if (!first) {
                out.print(",");
            }
            first = false;
            out.print(jointName + "=" + currentAngles.get(jointName));
        }
        out.print("/");
        first = true;
        for (String jointName: currentJointSpeeds.keySet()) {
            if (!first) {
                out.print(",");
            }
            first = false;
            out.print(jointName + "=" + currentJointSpeeds.get(jointName));
        }
        String copFrame;
        if (this.baseFrame != null) {
            RigidBodyTransform basePose = baseFrame.getTransformToWorldFrame();
            Vector3DBasics translation = basePose.getTranslation();
            Quaternion baseOrientation = new Quaternion(baseFrame.getTransformToWorldFrame().getRotation());
            FrameVector3DReadOnly baseLinearVelocity = baseFrame.getTwistOfFrame().getLinearPart();
            FrameVector3DReadOnly baseAngularVelocity = baseFrame.getTwistOfFrame().getAngularPart();

            copFrame = translation.getX() + "," +
                    translation.getY() + "," +
                    translation.getZ() + "," +
                    baseOrientation.getX() + "," +
                    baseOrientation.getY() + "," +
                    baseOrientation.getZ() + "," +
                    baseOrientation.getS() + "," +
                    baseLinearVelocity.getX() + "," +
                    baseLinearVelocity.getY() + "," +
                    baseLinearVelocity.getZ() + "," +
                    baseAngularVelocity.getX() + "," +
                    baseAngularVelocity.getY() + "," +
                    baseAngularVelocity.getZ();
        } else {
            copFrame = "0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0";
        }
        out.print("/" +
                copFrame +
                "/" + this.coP + "/" + this.wristLeft + "/" + this.wristRight);
        out.println();
    }

    public double getDesiredAngle(String i) {
        return desiredAngles.get(i);
    }

    public void setDesiredAngle(String i, double angle) {
        desiredAngles.put(i, angle);
    }

    public double getCurrentAngle(String i) {
        return currentAngles.get(i);
    }

    public void setCurrentAngle(String i, double angle) {
        currentAngles.put(i, angle);
    }

    public double getJointSpeed(String i) {
        return currentJointSpeeds.get(i);
    }

    public void setJointSpeed(String i, double angle) {
        currentJointSpeeds.put(i, angle);
    }

    public void setCenterOfMassFrame(ReferenceFrame centerOfMassFrame) {
        this.centerOfMassFrame = centerOfMassFrame;
    }

    public ReferenceFrame getCenterOfMassFrame() {
        return centerOfMassFrame;
    }

    public void setCoP(FramePoint2D coP) {
        this.coP = coP;
    }

    public FramePoint2D getCoP() {
        return coP;
    }

    public void setWristLeft(ForceSensorDataReadOnly wristLeft) {
        this.wristLeft = wristLeft;
    }

    public ForceSensorDataReadOnly getWristLeft() {
        return wristLeft;
    }

    public void setWristRight(ForceSensorDataReadOnly wristRight) {
        this.wristRight = wristRight;
    }

    public ForceSensorDataReadOnly getWristRight() {
        return wristRight;
    }

    public void setTime(double time) {
        this.time = time;
    }
    
    public double getTime() {
        return time;
    }

    public void setPelvisFrame(ReferenceFrame pelvisFrame) {
        this.pelvisFrame = pelvisFrame;
    }

    public ReferenceFrame getPelvisFrame() {
        return pelvisFrame;
    }

    public void setBaseFrame(MovingReferenceFrame baseFrame) {
        this.baseFrame = baseFrame;
    }

    public MovingReferenceFrame getBaseFrame() {
        return baseFrame;
    }
}
