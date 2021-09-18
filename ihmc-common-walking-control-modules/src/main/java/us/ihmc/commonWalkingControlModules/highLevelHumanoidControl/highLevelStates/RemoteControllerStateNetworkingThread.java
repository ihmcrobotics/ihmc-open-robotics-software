package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;

public class RemoteControllerStateNetworkingThread extends Thread {
    private ServerSocket serverSocket = null;
    private PrintWriter out = null;
    private BufferedReader in = null;

    private double[] desiredAngles;
    private double[] currentAngles;

    public RemoteControllerStateNetworkingThread(int l) {
        desiredAngles = new double[l];
        currentAngles = new double[l];
    }

    public void run(){
        try {
            serverSocket = new ServerSocket(15923);
            System.out.println("Remote controller listening at " + serverSocket.getInetAddress() + ":" + serverSocket.getLocalPort() + " with " + desiredAngles.length + " parameters.");
            Socket clientSocket = serverSocket.accept();
            out = new PrintWriter(clientSocket.getOutputStream(), true);
            in = new BufferedReader(new InputStreamReader(clientSocket.getInputStream()));
            while (true) {
                String desired = in.readLine();
                String[] angles = desired.split(",");
                for (int i = 0; i < angles.length; i++) {
                    desiredAngles[i] = Double.parseDouble(angles[0]);
                }
                for (int i = 0; i < currentAngles.length; i++) {
                    out.print(currentAngles[i] + ", ");
                }
                out.println();

            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public double getDesiredAngle(int i) {
        return desiredAngles[i];
    }

    public double getCurrentAngle(int i) {
        return currentAngles[i];
    }

    public void setCurrentAngle(int i, double currentAngle) {
        this.currentAngles[i] = currentAngle;
    }
}
