package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
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
    private ReferenceFrame centerOfMassFrame;
    private FramePoint2D coP;
    private ForceSensorDataReadOnly wristLeft;
    private ForceSensorDataReadOnly wristRight;

    public RemoteControllerStateNetworkingThread(int l) {
        desiredAngles = new HashMap<>();
        currentAngles = new HashMap<>();
    }

    public void run(){
        try {
            serverSocket = new ServerSocket(15923);
            System.out.println("Remote controller listening at " + serverSocket.getInetAddress() + ":" + serverSocket.getLocalPort() + " with " + desiredAngles.keySet().size() + " parameters.");
            while (true) {
                Socket clientSocket = serverSocket.accept();
                System.out.println("Remote controller connected");
                out = new PrintWriter(clientSocket.getOutputStream(), true);
                in = new BufferedReader(new InputStreamReader(clientSocket.getInputStream()));
                while (true) {
                    printSensorReadings();
                    String desired = in.readLine();
                    if (desired != null) {
                        String[] angles = desired.split(",");
                        for (int i = 0; i < angles.length; i++) {
                            String[] parts = angles[i].split("=");
                            desiredAngles.put(parts[0], Double.parseDouble(parts[1]));
                        }
                        printSensorReadings();
                    } else {
                        System.out.println("Command was null");
                    }
                }
            }
        } catch (IOException e) {
            e.printStackTrace();
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
        out.print("/" +
                this.centerOfMassFrame.getTransformToWorldFrame().getTranslationX() + "," +
                this.centerOfMassFrame.getTransformToWorldFrame().getTranslationY() + "," +
                this.centerOfMassFrame.getTransformToWorldFrame().getTranslationZ() +
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
}
