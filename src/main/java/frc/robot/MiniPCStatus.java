package frc.robot;

import frc.subsytem.AbstractSubsystem;
import org.littletonrobotics.junction.Logger;

import java.io.IOException;
import java.net.InetAddress;
import java.net.UnknownHostException;

public class MiniPCStatus implements Runnable {

    private boolean miniPCIsConnected;

    @Override
    public void run() {
        while(true) {
            try {
                InetAddress address = InetAddress.getByName(Constants.MINIPC_IP);

                if(address.isReachable(5000)) {
                    // If minipc connected
                    miniPCIsConnected = true;
                    System.out.println("Connected");

                } else {
                    // If minipc not connected
                    miniPCIsConnected = false;
                    System.out.println("Not connected");

                }
            } catch (IOException e) {
                // If minipc not connected
                miniPCIsConnected = false;
                System.out.println("Not connected");

            }
            Logger.getInstance().recordOutput("Mini PC Connection Status", miniPCIsConnected);
        }
    }
}
