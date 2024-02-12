package frc.robot.Subsystems;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PIComunicator extends SubsystemBase
{
    int port = 10101;
    ServerSocket serverSocket;
    Socket piSocket;
    BufferedReader in;
    int x = 0,y = 0;
    OutputStream outputStream;

    public PIComunicator()
    {
        try {
            serverSocket = new ServerSocket(port);
            System.out.println("server is running on port: " + port);
            piSocket = serverSocket.accept();
            System.out.println("client connected on port: " + port);
            in = new BufferedReader(new InputStreamReader(piSocket.getInputStream()));
            outputStream = piSocket.getOutputStream();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public int[] getNote()
    {
        try{
            outputStream.write(1);
            outputStream.flush();

            x = Integer.parseInt(in.readLine());
            y = Integer.parseInt(in.readLine());
        }catch(IOException e){
            e.printStackTrace();
        }
        
        return new int[]{x,y};
    }

    public void close()
    {
        try {
            piSocket.close();
            serverSocket.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}